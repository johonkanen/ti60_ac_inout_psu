// Ti60 data collection: the board's continuous stream (port 5004, every sample of eight
// channels) as a chart recorder. Every millisecond of samples becomes one point per channel,
// its minimum, maximum and mean in ADC counts, and its first sample (the value at that instant),
// kept for MAX_POINTS (5 minutes); what is shown
// and written is in each source's unit (the scaling, kept on the board in scaling.json).
//
// The plots, one to four with the channels put in them as chosen (voltages in one, currents
// in another), or one per channel, follow the newest data over the chosen window
// (Live), or stay on a range (Pause, or drag across a plot to zoom in, down to single 1 ms
// points) while the collection goes on. A plot shows each channel as its mean with the min..max
// band, or as the instantaneous value alone (the sample at each millisecond, and at the start
// of each pixel column when a column holds several). A plot gets as many points as it has pixel
// columns: min of mins, max of maxes, mean of means. Record 1 ms writes every point to a CSV file until
// stopped, straight to disk where the browser can (Chrome, Edge), else into memory; Record raw
// saves the next seconds of samples as they come. Settings go to the board over /ws as on the
// scope page, and a change starts the collection again. The plots, their units and their
// layout are plots.js's, shared with the scope page.
'use strict';

const STREAM_PORT = 5004;
const RECORD_BYTES = CHANNELS * 2;
const STREAM_MAX_BYTES = 62.5e6;    // what the board carries: 3.9 MS/s
// stream rates, from the fastest the board carries down to one sample a millisecond
const DIVIDERS = [31, 49, 124, 249, 624, 1249, 2499, 6249, 12499, 24999, 62499, 124999];
const WINDOWS = [1, 5, 10, 30, 60, 300];    // seconds
const MAX_POINTS = 1000 * Math.max(...WINDOWS);
const MEMORY_RECORDING_MAX = 200e6;         // bytes of CSV kept in memory without a file

const $ = id => document.getElementById(id);
let control = null;
let config = null;
let sourceNames = [];
let clockHz = 125e6;
let stream = null;
let recording = null;           // raw samples
let pointRecording = null;      // 1 ms points to CSV
let built = false;
let boardStatus = {};           // the board's last status message
let scopeStopped = false;       // the scope stopped once when this page opened
let area = null;                // the plots

// the 1 ms points, a ring of MAX_POINTS: point k is millisecond k of the collection
const points = {
  count: 0,
  min: Array.from({length: CHANNELS}, () => new Float32Array(MAX_POINTS)),
  max: Array.from({length: CHANNELS}, () => new Float32Array(MAX_POINTS)),
  mean: Array.from({length: CHANNELS}, () => new Float32Array(MAX_POINTS)),
  sample: Array.from({length: CHANNELS}, () => new Float32Array(MAX_POINTS)),    // the first of the ms
};

// what is shown: the newest window, or a fixed range of seconds
const view = {live: true, from: 0, to: 0, dirty: true};
let shown = null;               // the data the plots have, from reduce()

function formatRate(hz) {
  if (hz >= 1e6) return `${+(hz / 1e6).toFixed(3)} MS/s`;
  if (hz >= 1e3) return `${+(hz / 1e3).toFixed(3)} kS/s`;
  return `${+hz.toFixed(3)} S/s`;
}

function formatClock(seconds) {
  const m = Math.floor(seconds / 60);
  const s = seconds - 60 * m;
  return m > 0 ? `${m}:${s.toFixed(1).padStart(4, '0')}` : `${+s.toFixed(3)} s`;
}

function sampleRate() {
  return clockHz / ((config ? config.divider : 124) + 1);
}

// ------------------------------------------------------------------ units
function sourceOf(c) {
  return config ? sourceNames[config.sources[c]] || '' : '';
}

function unitOf(c) {
  return units.of(sourceOf(c)).unit;
}

function format(c, v) {
  return units.format(sourceOf(c), v);
}

function showScaling() {
  units.showTable($('scaling').querySelector('tbody'), Array.from({length: CHANNELS}, (_, c) => sourceOf(c)), () => {
    $('scaling_status').textContent = 'changed, not saved';
    rebuildPlots();
  });
}

// ------------------------------------------------------------------ controls
function buildControls() {
  $('rate').innerHTML = DIVIDERS.map(d => {
    const hz = clockHz / (d + 1);
    return `<option value="${d}">${formatRate(hz)} · ${+(hz / 1000).toFixed(2)} samples/ms · ` +
           `${(RECORD_BYTES * hz / 1e6).toFixed(1)} MB/s</option>`;
  }).join('');
  $('window').innerHTML = WINDOWS.map(w => `<option value="${w}">${w < 60 ? w + ' s' : w / 60 + ' min'}</option>`).join('');
  $('window').value = 10;

  const channels = $('channels');
  channels.innerHTML = '';
  for (let i = 0; i < CHANNELS; i++) {
    const row = document.createElement('div');
    row.className = 'channel with_plot';
    row.innerHTML = `<input type="checkbox" id="show${i}" ${area.visible[i] ? 'checked' : ''} title="show CH${i + 1}">
      <span class="swatch" style="background:${COLORS[i]}" title="CH${i + 1}"></span>
      <select id="source${i}" title="CH${i + 1}">${Signals.optionsHtml(sourceNames)}</select>
      ${PlotArea.plotSelect(i)}`;
    channels.appendChild(row);
    row.querySelector(`#show${i}`).addEventListener('change', e => {
      area.visible[i] = e.target.checked;
      rebuildPlots();
    });
    row.querySelector(`#source${i}`).addEventListener('change', settingsChanged);
  }
  $('rate').addEventListener('change', settingsChanged);
  $('window').addEventListener('change', () => { view.dirty = true; });
  area.bindControls();
  // the views (signals.js): their channels and plots; the rate is this page's own
  Signals.viewBar($('views'), {
    apply: v => {
      const list = v.channels.filter(([name]) => sourceNames.includes(name)).slice(0, CHANNELS);
      if (!list.length) return;
      list.forEach(([name, plot], c) => {
        $(`source${c}`).value = sourceNames.indexOf(name);
        area.layout.plotOf[c] = Math.min(Math.max(1, plot || 1), 4);
      });
      for (let c = 0; c < CHANNELS; c++) {
        area.visible[c] = c < list.length;
        $(`show${c}`).checked = area.visible[c];
        area.layout.hidden[c] = false;
      }
      if (area.layout.count !== 'each') area.layout.count = String(Math.max(...list.map(([, plot]) => plot || 1)));
      area.save();
      area.showControls();
      settingsChanged();
    },
    current: () => ({channels: Array.from({length: CHANNELS}, (_, c) => c)
      .filter(c => area.visible[c]).map(c => [sourceNames[+$(`source${c}`).value], area.layout.plotOf[c] || 1])}),
  });
  $('start').onclick = start;
  $('stop').onclick = stop;
  $('export').onclick = exportCsv;
  $('record').onclick = startRecording;
  $('record_points').onclick = () => (pointRecording ? finishPointRecording() : startPointRecording());
  $('save_scaling').onclick = async () => { $('scaling_status').textContent = await units.save(); };
  $('live').onclick = () => setLive(true);
  $('pause').onclick = () => setLive(false);
  $('zoom_out').onclick = () => zoomTo(Math.max(0, points.count - MAX_POINTS) / 1000, points.count / 1000);
}

function showConfig(c) {
  const fastest = Math.min(...DIVIDERS);
  $('rate').value = DIVIDERS.includes(c.divider) ? c.divider : (c.divider < fastest ? fastest : 124);
  c.sources.forEach((s, i) => { $(`source${i}`).value = s; });
  showScaling();
}

// the board takes rate and sources when the stream connects
function sendConfig() {
  config = {...config, divider: +$('rate').value,
            sources: Array.from({length: CHANNELS}, (_, i) => +$(`source${i}`).value)};
  if (control && control.readyState === WebSocket.OPEN) {
    // only what this page sets: the others (the ADCs, the gate driver supplies) are the scope page's
    control.send(JSON.stringify({type: 'config', config: {divider: config.divider, sources: config.sources}}));
  }
  showScaling();
}

function settingsChanged() {
  const running = !!stream;
  stop();
  sendConfig();
  rebuildPlots();
  if (running) setTimeout(start, 300);
}

function setLive(live) {
  if (!live && view.live) {
    // the view as it is now, held
    view.to = points.count / 1000;
    view.from = Math.max(0, points.count - 1000 * +$('window').value) / 1000;
  }
  view.live = live;
  view.dirty = true;
  $('live').classList.toggle('active', live);
  $('pause').classList.toggle('active', !live);
  if (!stream) render();
}

function zoomTo(from, to) {
  if (to - from < 0.002) return;
  view.live = false;
  view.from = from;
  view.to = to;
  setLive(false);
}

// ------------------------------------------------------------------ the stream
function start() {
  if (stream) return;
  sendConfig();
  const rate = sampleRate();
  const s = {
    socket: new WebSocket(`ws://${location.hostname}:${STREAM_PORT}/`),
    rate, perMs: rate / 1000, bytes: 0, started: 0, samples: 0, stopping: false,
    next: Math.floor(rate / 1000), // the first sample of the next point
    min: new Float64Array(CHANNELS).fill(Infinity),
    max: new Float64Array(CHANNELS).fill(-Infinity),
    sum: new Float64Array(CHANNELS), n: 0,
    first: new Float64Array(CHANNELS),
  };
  points.count = 0;
  s.socket.binaryType = 'arraybuffer';
  s.socket.onopen = () => { s.started = performance.now(); };
  s.socket.onmessage = event => {
    s.bytes += event.data.byteLength;
    take(s, new Int16Array(event.data));
    if (recording) record(event.data);
  };
  s.socket.onclose = () => {
    if (stream !== s) return;
    stream = null;
    stopped();
    if (!s.stopping) {
      // the board's status tells an overrun from a stream that another page took over
      $('status').textContent = 'the stream ended…';
      setTimeout(() => {
        $('status').textContent = boardStatus.lost_words ?
          `stopped by the board: the PC fell more than the board's 32 MB ring (about half a second) behind at ${formatRate(rate)} ` +
          `(overrun); the link carries up to about ${STREAM_MAX_BYTES / 1e6} MB/s` :
          'the stream was closed: another page or client may have taken it (the board serves one at a time)';
        $('status').classList.add('error');
      }, 600);
    }
  };
  stream = s;
  setLive(true);
  $('start').classList.add('active');
  $('record').disabled = false;
  $('record_points').disabled = false;
  $('status').classList.remove('error');
  requestAnimationFrame(draw);
}

function stopped() {
  finishRecording();
  finishPointRecording();
  $('start').classList.remove('active');
  $('record').disabled = true;
  $('record_points').disabled = true;
  view.dirty = true;
  render();
}

function stop() {
  if (!stream) return;
  stream.stopping = true;
  stream.socket.close();
  stream = null;
  stopped();
}

// samples into 1 ms points: point k is samples floor(k*perMs) .. floor((k+1)*perMs)-1
function take(s, data) {
  const n = data.length / CHANNELS;
  let at = 0;
  while (at < n) {
    const end = Math.min(n, at + (s.next - s.samples));    // where this point ends, or the data
    if (s.n === 0 && end > at) {
      for (let c = 0; c < CHANNELS; c++) s.first[c] = data[at * CHANNELS + c];
    }
    for (let c = 0; c < CHANNELS; c++) {
      let min = s.min[c], max = s.max[c], sum = s.sum[c];
      for (let i = at * CHANNELS + c, last = end * CHANNELS; i < last; i += CHANNELS) {
        const v = data[i];
        if (v < min) min = v;
        if (v > max) max = v;
        sum += v;
      }
      s.min[c] = min; s.max[c] = max; s.sum[c] = sum;
    }
    s.n += end - at;
    s.samples += end - at;
    at = end;
    if (s.samples === s.next) {
      const slot = points.count % MAX_POINTS;
      for (let c = 0; c < CHANNELS; c++) {
        points.min[c][slot] = s.min[c];
        points.max[c][slot] = s.max[c];
        points.mean[c][slot] = s.sum[c] / s.n;
        points.sample[c][slot] = s.first[c];
        s.min[c] = Infinity; s.max[c] = -Infinity; s.sum[c] = 0;
      }
      s.n = 0;
      points.count++;
      s.next = Math.floor((points.count + 1) * s.perMs);
    }
  }
}

// ------------------------------------------------------------------ what is shown
// points first..end-1 (of those still kept), reduced to columns, in units, and over all of them
function reduce(first, end, columns) {
  first = Math.max(first, points.count - MAX_POINTS, 0);
  end = Math.min(end, points.count);
  const span = Math.max(0, end - first);
  const per = Math.max(1, Math.ceil(span / columns));
  const buckets = Math.ceil(span / per);
  const out = {time: new Float64Array(buckets), first, end, channels: []};
  for (let b = 0; b < buckets; b++) out.time[b] = (first + b * per) / 1000;
  for (let c = 0; c < CHANNELS; c++) {
    const ch = {min: new Float64Array(buckets), max: new Float64Array(buckets), mean: new Float64Array(buckets),
                sample: new Float64Array(buckets),
                lo: Infinity, hi: -Infinity, average: NaN, last: NaN};
    out.channels.push(ch);
    if (!area.visible[c] || span === 0) continue;
    const unit = units.convert(sourceOf(c));
    const pmin = points.min[c], pmax = points.max[c], pmean = points.mean[c], psample = points.sample[c];
    let total = 0;
    for (let b = 0; b < buckets; b++) {
      const from = first + b * per, to = Math.min(end, from + per);
      let lo = Infinity, hi = -Infinity, sum = 0;
      for (let k = from; k < to; k++) {
        const slot = k % MAX_POINTS;
        if (pmin[slot] < lo) lo = pmin[slot];
        if (pmax[slot] > hi) hi = pmax[slot];
        sum += pmean[slot];
      }
      // a negative scale swaps the ends
      const a = unit(lo), z = unit(hi);
      ch.min[b] = Math.min(a, z);
      ch.max[b] = Math.max(a, z);
      ch.mean[b] = unit(sum / (to - from));
      ch.sample[b] = unit(psample[from % MAX_POINTS]);
      if (ch.min[b] < ch.lo) ch.lo = ch.min[b];
      if (ch.max[b] > ch.hi) ch.hi = ch.max[b];
      total += ch.mean[b] * (to - from);
    }
    ch.average = total / span;
    ch.last = unit(pmean[(end - 1) % MAX_POINTS]);
  }
  return out;
}

function rebuildPlots() {
  area.rebuild();
  view.dirty = true;
  if (!stream) render();
}

function render() {
  let first, end;
  if (view.live) {
    end = points.count;
    first = end - 1000 * +$('window').value;
  } else {
    first = Math.floor(view.from * 1000);
    end = Math.ceil(view.to * 1000);
  }
  shown = reduce(first, end, Math.max(100, Math.min(4000, area.width())));
  area.setData(shown);
  updateTable();
  view.dirty = false;
}

// ten times a second while collecting; a paused view only when it changed
function draw(now) {
  const s = stream;
  if (!s) return;
  requestAnimationFrame(draw);
  if (now - (s.lastDraw || 0) < 100) return;
  s.lastDraw = now;
  const seconds = s.started ? (now - s.started) / 1000 : 0;
  $('status').textContent = `${formatRate(s.rate)} · ${seconds > 0 ? (s.bytes / seconds / 1e6).toFixed(1) : 0} MB/s · ` +
    `${formatClock(points.count / 1000)} collected` + (view.live ? '' : ' · view paused') +
    (recording ? ` · recording raw ${(recording.bytes / 1e6).toFixed(0)} MB` : '') +
    (pointRecording ? ` · recording 1 ms points ${(pointRecording.bytes / 1e6).toFixed(1)} MB` +
                      (pointRecording.lost ? `, ${pointRecording.lost} lost` : '') : '');
  if (view.live || view.dirty) render();
}

// a plot's channels at the cursor, or at the newest point shown when the mouse is elsewhere
function plotValues(p, idx, hovering) {
  if (!hovering) idx = shown.time.length - 1;
  let html = `<div class="when">${formatClock(shown.time[idx])} · ${hovering ? 'cursor' : view.live ? 'newest' : 'end of view'}</div>`;
  for (const c of p.channels) {
    const ch = shown.channels[c];
    html += area.valueBlock(c, p.mode === 'sample' ? ch.sample[idx] : ch.mean[idx],
                            p.mode === 'sample' ? 'instantaneous' : `${format(c, ch.min[idx])} … ${format(c, ch.max[idx])}`);
  }
  return html;
}

function updateTable() {
  const body = document.querySelector('#measurements tbody');
  if (!shown || shown.end <= shown.first) { body.innerHTML = ''; return; }
  const rows = [];
  for (let c = 0; c < CHANNELS; c++) {
    if (!area.visible[c]) continue;
    const ch = shown.channels[c];
    rows.push(`<tr><td style="color:${COLORS[c]}">CH${c + 1}</td><td>${sourceOf(c)}</td><td>${unitOf(c)}</td>
      <td>${format(c, ch.last)}</td><td>${format(c, ch.lo)}</td><td>${format(c, ch.hi)}</td>
      <td>${format(c, ch.average)}</td><td>${format(c, ch.hi - ch.lo)}</td></tr>`);
  }
  body.innerHTML = rows.join('');
}

// ------------------------------------------------------------------ files
function download(parts, name, type) {
  const link = document.createElement('a');
  link.href = URL.createObjectURL(new Blob(parts, {type}));
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 10000);
}

function csvHeader(channels) {
  const header = ['time_s'];
  for (const c of channels) {
    const name = `CH${c + 1} ${sourceOf(c)} [${unitOf(c)}]`;
    header.push(`${name} min`, `${name} max`, `${name} mean`, `${name} sample`);
  }
  return header.join(',') + '\n';
}

// points first..end-1 as CSV lines, in units
function csvLines(channels, first, end) {
  const toUnits = channels.map(c => units.convert(sourceOf(c)));
  const lines = [];
  for (let k = first; k < end; k++) {
    const slot = k % MAX_POINTS;
    let line = (k / 1000).toFixed(3);
    channels.forEach((c, i) => {
      const a = toUnits[i](points.min[c][slot]), z = toUnits[i](points.max[c][slot]);
      line += `,${format(c, Math.min(a, z))},${format(c, Math.max(a, z))},${format(c, toUnits[i](points.mean[c][slot]))}` +
              `,${format(c, toUnits[i](points.sample[c][slot]))}`;
    });
    lines.push(line);
  }
  return lines.length ? lines.join('\n') + '\n' : '';
}

// the 1 ms points of the view, every one
function exportCsv() {
  if (!shown || shown.end <= shown.first) return;
  const channels = Array.from({length: CHANNELS}, (_, c) => c).filter(c => area.visible[c]);
  download([csvHeader(channels), csvLines(channels, shown.first, shown.end)],
           `collection_1ms_${shown.end - shown.first}points.csv`, 'text/csv');
}

// every 1 ms point from now on, until stopped
async function startPointRecording() {
  if (!stream || pointRecording) return;
  const channels = Array.from({length: CHANNELS}, (_, c) => c).filter(c => area.visible[c]);
  const name = `collection_1ms_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.csv`;
  const r = {channels, name, next: 0, bytes: 0, parts: [], writable: null, lost: 0, writing: Promise.resolve()};
  if (window.showSaveFilePicker) {
    try {
      const handle = await window.showSaveFilePicker({suggestedName: name,
        types: [{description: 'CSV', accept: {'text/csv': ['.csv']}}]});
      r.writable = await handle.createWritable();
    } catch {
      return;   // no file chosen
    }
  }
  r.next = points.count;
  put(r, csvHeader(channels));
  r.timer = setInterval(() => writePoints(r), 1000);
  pointRecording = r;
  $('record_points').classList.add('active');
  $('record_points').textContent = 'Stop recording';
}

// writes in order, one after the other
function put(r, text) {
  if (!text) return;
  r.bytes += text.length;
  if (r.writable) {
    r.writing = r.writing.then(() => r.writable.write(text));
  } else {
    r.parts.push(text);
  }
}

function writePoints(r) {
  const end = points.count;
  if (end - r.next > MAX_POINTS) {     // fell behind the ring: a gap
    r.lost += end - MAX_POINTS - r.next;
    r.next = end - MAX_POINTS;
  }
  put(r, csvLines(r.channels, r.next, end));
  r.next = end;
  if (!r.writable && r.bytes > MEMORY_RECORDING_MAX && pointRecording === r) {
    finishPointRecording();
    $('status').textContent = `recording stopped at ${(r.bytes / 1e6).toFixed(0)} MB: ` +
      'this browser cannot write to a file as it goes (Chrome and Edge can)';
  }
}

async function finishPointRecording() {
  const r = pointRecording;
  if (!r) return;
  pointRecording = null;
  clearInterval(r.timer);
  writePoints(r);
  $('record_points').classList.remove('active');
  $('record_points').textContent = 'Record 1 ms';
  if (r.writable) {
    await r.writing;
    await r.writable.close();
  } else {
    download(r.parts, r.name, 'text/csv');
  }
}

function startRecording() {
  if (!stream || recording) return;
  const seconds = Math.max(1, Math.min(60, +$('record_seconds').value || 5));
  recording = {buffers: [], bytes: 0, until: performance.now() + 1000 * seconds, rate: stream.rate};
  $('record').classList.add('active');
}

function record(buffer) {
  recording.buffers.push(buffer);
  recording.bytes += buffer.byteLength;
  if (performance.now() >= recording.until) finishRecording();
}

// int16 little endian, eight channels interleaved, the sample rate in the name
function finishRecording() {
  if (!recording) return;
  const r = recording;
  recording = null;
  $('record').classList.remove('active');
  if (r.bytes > 0) {
    download(r.buffers, `stream_${r.rate}Sps_8ch_int16le_${r.bytes / RECORD_BYTES}samples.bin`,
             'application/octet-stream');
  }
}

// ------------------------------------------------------------------ the board's settings
function connect() {
  control = new WebSocket(`ws://${location.host}/ws`);
  control.onmessage = async event => {
    if (typeof event.data !== 'string') return;    // the scope page's captures
    const message = JSON.parse(event.data);
    if (message.type === 'status') {
      boardStatus = message;
      // a scope left running when this page opened stops: its capture stays for the scope page
      if (message.running && !scopeStopped) {
        scopeStopped = true;
        control.send(JSON.stringify({type: 'stop'}));
      }
    } else if (message.type === 'hello') {
      sourceNames = message.source_names;
      clockHz = message.clock_hz;
      config = message.config;
      if (!built) {
        await units.load();
        area = new PlotArea({container: $('plots'), storageKey: 'collect.layout', sourceOf, timeFormat: formatClock,
                             bands: true, values: plotValues, onSelect: zoomTo});
        buildControls();
        built = true;
      }
      showConfig(config);
      rebuildPlots();
      if (!stream) $('status').textContent = 'ready';
    }
  };
  control.onclose = () => {
    if (!stream) {
      $('status').textContent = 'disconnected from the board, retrying…';
      $('status').classList.add('error');
    }
    setTimeout(connect, 1000);
  };
}

window.addEventListener('resize', () => { if (area) area.resize(); });
connect();
