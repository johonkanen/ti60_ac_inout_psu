// Ti60 oscilloscope web client: controls the capture through scope_server.py and plots
// the captures with uPlot.
'use strict';

const CHANNELS = 8;
const COLORS = ['#f5c542', '#4aa3ff', '#ff6b9a', '#5ad67d', '#c08bff', '#ff9f43', '#3dd6d0', '#e0e0e0'];
const DIVIDERS = [0, 1, 4, 9, 24, 49, 124, 249, 624, 1249, 2499, 6249, 12499, 24999, 62499, 124999];
const LENGTHS = {
  bram: [256, 512, 1024, 2048, 4096],
  deep: [4096, 16384, 65536, 262144, 1048576, 2097152, 4194304],
};
const DEEP_MAX_RATE = 20.8e6;   // eight channels stream into the HyperRAM without overflows up to this

let socket = null;
let config = null;
let probes = [];             // per source {name, format} from the FPGA's probe table
const SIGNED = 0, UNSIGNED = 1, DIGITAL = 2;
let logicPlot = null;
let logicLanes = [];         // [{channel, bit}] of the logic plot series
let clockHz = 125e6;
let plot = null;
let capture = null;          // last capture view: {header, time, channels: [Float64Array]}
const visible = new Array(CHANNELS).fill(true);
visible.fill(false, 4);

const $ = id => document.getElementById(id);

function formatRate(hz) {
  if (hz >= 1e6) return `${+(hz / 1e6).toFixed(3)} MS/s`;
  if (hz >= 1e3) return `${+(hz / 1e3).toFixed(3)} kS/s`;
  return `${+hz.toFixed(3)} S/s`;
}

function formatCount(n) {
  if (n >= 1048576 && n % 1048576 === 0) return `${n / 1048576}M`;
  if (n >= 1024 && n % 1024 === 0) return `${n / 1024}k`;
  return `${n}`;
}

function formatTime(seconds) {
  const a = Math.abs(seconds);
  if (a === 0) return '0';
  if (a < 1e-6) return `${+(seconds * 1e9).toFixed(1)} ns`;
  if (a < 1e-3) return `${+(seconds * 1e6).toFixed(2)} µs`;
  if (a < 1) return `${+(seconds * 1e3).toFixed(2)} ms`;
  return `${+seconds.toFixed(3)} s`;
}

// ------------------------------------------------------------------ controls
function buildControls() {
  fillMemoryOptions('bram');
  $('trigger_channel').innerHTML = Array.from({length: CHANNELS}, (_, i) => `<option value="${i}">CH${i + 1}</option>`).join('');

  const channels = $('channels');
  channels.innerHTML = '';
  for (let i = 0; i < CHANNELS; i++) {
    const row = document.createElement('div');
    row.className = 'channel';
    row.innerHTML = `<input type="checkbox" id="show${i}" ${visible[i] ? 'checked' : ''} title="show CH${i + 1}">
      <span class="swatch" style="background:${COLORS[i]}" title="CH${i + 1}"></span>
      <select id="source${i}">${probes.map((p, s) => `<option value="${s}">CH${i + 1}: ${p.name}${p.format === DIGITAL ? ' (digital)' : ''}</option>`).join('')}</select>`;
    channels.appendChild(row);
    row.querySelector(`#show${i}`).addEventListener('change', e => {
      visible[i] = e.target.checked;
      if (capture) showCapture(capture.buffer); else updateMeasurements();
    });
    row.querySelector(`#source${i}`).addEventListener('change', () => sendConfig());
  }

  $('memory').addEventListener('change', () => {
    const memory = $('memory').value;
    fillMemoryOptions(memory);
    $('length').value = memory === 'deep' ? 1048576 : 4096;
    sendConfig();
  });
  for (const id of ['rate', 'length', 'mode', 'trigger_channel', 'trigger_edge', 'trigger_level', 'generator_hz',
                    'trigger_mask', 'trigger_pattern']) {
    $(id).addEventListener('change', () => sendConfig());
  }
  $('pre_percent').addEventListener('input', () => { $('pre_value').textContent = `${$('pre_percent').value} %`; });
  $('pre_percent').addEventListener('change', () => sendConfig());
  $('trigger_edge').addEventListener('change', showTriggerMode);

  $('run').onclick = () => send({type: 'run'});
  $('single').onclick = () => send({type: 'single'});
  $('stop').onclick = () => send({type: 'stop'});
  $('force').onclick = () => send({type: 'force'});
  $('export').onclick = exportCsv;
  $('full_view').onclick = () => send({type: 'view', reset: true});
}

// rates and lengths depend on the memory: deep captures overflow above DEEP_MAX_RATE
function fillMemoryOptions(memory) {
  const rate = $('rate').value, length = $('length').value;
  $('rate').innerHTML = DIVIDERS.map(d => {
    const hz = clockHz / (d + 1);
    const note = memory === 'deep' && hz > DEEP_MAX_RATE ? ' (overflows)' : '';
    return `<option value="${d}">${formatRate(hz)}${note}</option>`;
  }).join('');
  $('length').innerHTML = LENGTHS[memory].map(l => `<option value="${l}">${formatCount(l)} samples</option>`).join('');
  if (rate !== '') $('rate').value = rate;
  if (LENGTHS[memory].includes(+length)) $('length').value = length;
}

function showConfig(c) {
  $('memory').value = c.deep ? 'deep' : 'bram';
  fillMemoryOptions($('memory').value);
  $('rate').value = c.divider;
  $('length').value = c.length;
  $('mode').value = c.mode === 'single' ? 'auto' : c.mode;
  $('pre_percent').value = c.pre_percent;
  $('pre_value').textContent = `${c.pre_percent} %`;
  $('trigger_channel').value = c.trigger_channel;
  $('trigger_edge').value = c.trigger_edge;
  $('trigger_mask').value = (c.trigger_mask ?? 0xffff).toString(16);
  $('trigger_pattern').value = (c.trigger_pattern ?? 1).toString(16);
  showTriggerMode();
  $('trigger_level').value = c.trigger_level;
  $('generator_hz').value = c.generator_hz;
  c.sources.forEach((s, i) => { $(`source${i}`).value = s; });
}

function sendConfig() {
  config = {
    deep: $('memory').value === 'deep',
    divider: +$('rate').value,
    length: +$('length').value,
    mode: $('mode').value,
    pre_percent: +$('pre_percent').value,
    trigger_channel: +$('trigger_channel').value,
    trigger_edge: +$('trigger_edge').value,
    trigger_level: Math.max(-32768, Math.min(65535, Math.round(+$('trigger_level').value))),
    trigger_mask: parseInt($('trigger_mask').value, 16) & 0xffff,
    trigger_pattern: parseInt($('trigger_pattern').value, 16) & 0xffff,
    generator_hz: +$('generator_hz').value,
    sources: Array.from({length: CHANNELS}, (_, i) => +$(`source${i}`).value),
  };
  send({type: 'config', config});
}

function showTriggerMode() {
  document.querySelector('aside').classList.toggle('pattern-mode', $('trigger_edge').value === '3');
}

function sourceFormat(source) {
  return probes[source] ? probes[source].format : SIGNED;
}

function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

// ------------------------------------------------------------------ plot
function plotSize() {
  const width = $('plot').clientWidth || 800;
  return {width, height: Math.max(300, Math.min(window.innerHeight - 260, width * 0.55))};
}

// trigger level and the trigger instant
function drawMarkers(u) {
  if (!capture) return;
  const ctx = u.ctx;
  const h = capture.header;
  ctx.save();
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  const x = u.valToPos(0, 'x', true);
  ctx.strokeStyle = '#8a94a3';
  ctx.beginPath(); ctx.moveTo(x, u.bbox.top); ctx.lineTo(x, u.bbox.top + u.bbox.height); ctx.stroke();
  if (config && config.mode !== 'free' && config.trigger_edge !== 3) {
    const y = u.valToPos(h.trigger_level, 'y', true);
    ctx.strokeStyle = COLORS[h.trigger_channel];
    ctx.beginPath(); ctx.moveTo(u.bbox.left, y); ctx.lineTo(u.bbox.left + u.bbox.width, y); ctx.stroke();
  }
  ctx.restore();
}

function makePlot(data) {
  if (plot) plot.destroy();
  const axis = {stroke: '#8a94a3', grid: {stroke: '#2a313b', width: 1}, ticks: {stroke: '#2a313b'}};
  plot = new uPlot({
    ...plotSize(),
    scales: {x: {time: false}, y: {range: (u, min, max) => [Math.min(min, -1000), Math.max(max, 1000)]}},
    axes: [
      {...axis, values: (u, ticks) => ticks.map(formatTime)},
      {...axis, size: 60},
    ],
    series: [
      {label: 'time', value: (u, v) => v == null ? '' : formatTime(v)},
      ...Array.from({length: CHANNELS}, (_, i) => ({
        label: `CH${i + 1}`, stroke: COLORS[i], width: 1, show: visible[i] && !isDigital(i), points: {show: false},
      })),
    ],
    // a dragged range is fetched from the server at full detail
    cursor: {drag: {x: true, y: false, setScale: false}, sync: {key: 'scope'}},
    hooks: {draw: [drawMarkers], setSelect: [zoomToSelection]},
  }, data, $('plot'));
}

function zoomToSelection(u) {
  const select = u.select;
  if (select.width < 3) return;
  const start = u.posToVal(select.left, 'x'), end = u.posToVal(select.left + select.width, 'x');
  u.setSelect({left: 0, top: 0, width: 0, height: 0}, false);
  send({type: 'view', start, end});
}

function isDigital(c) {
  return capture ? sourceFormat(capture.header.sources[c]) === DIGITAL : false;
}

// header length, JSON header, float64 times, int32 values point-major
function showCapture(buffer) {
  const view = new DataView(buffer);
  const headerLength = view.getUint32(0, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, headerLength)));
  const n = header.points;
  const timesStart = 4 + headerLength;
  const time = new Float64Array(buffer.slice(timesStart, timesStart + 8 * n));
  const values = new Int32Array(buffer.slice(timesStart + 8 * n, timesStart + 8 * n + 4 * n * CHANNELS));
  const channels = Array.from({length: CHANNELS}, () => new Float64Array(n));
  for (let s = 0; s < n; s++) {
    for (let c = 0; c < CHANNELS; c++) channels[c][s] = values[s * CHANNELS + c];
  }
  capture = {buffer, header, time, channels};
  // the analog plot is rebuilt when the digital channels change, it hides them
  const analogShape = CHANNELS_KEY();
  if (plot && plot.shape === analogShape) plot.setData([time, ...channels], false);
  else { makePlot([time, ...channels]); plot.shape = analogShape; }
  plot.setScale('x', {min: time[0], max: time[n - 1]});
  showLogic();
  showViewInfo(header);
  updateMeasurements();
}

function CHANNELS_KEY() {
  return Array.from({length: CHANNELS}, (_, c) => `${capture.header.points}:${visible[c]}:${isDigital(c)}`).join(',');
}

// digital channels as logic lanes: every lane that is high anywhere in the view, lane 0 at least
function showLogic() {
  const lanes = [];
  for (let c = 0; c < CHANNELS; c++) {
    if (!visible[c] || !isDigital(c)) continue;
    let used = 1;
    for (const v of capture.channels[c]) used |= v;
    for (let bit = 0; bit < 16; bit++) if (used & (1 << bit)) lanes.push({channel: c, bit});
  }
  if (!lanes.length) {
    if (logicPlot) { logicPlot.destroy(); logicPlot = null; }
    logicLanes = [];
    return;
  }
  // lane k is drawn between k*1.5 and k*1.5+1, the top lane first
  const data = [capture.time, ...lanes.map((lane, k) => {
    const base = (lanes.length - 1 - k) * 1.5;
    return Array.from(capture.channels[lane.channel], v => base + ((v >> lane.bit) & 1));
  })];
  const key = lanes.map(l => `${l.channel}.${l.bit}`).join(',') + `:${capture.header.points}`;
  if (logicPlot && logicPlot.key === key) {
    logicPlot.setData(data, false);
  } else {
    if (logicPlot) logicPlot.destroy();
    const axis = {stroke: '#8a94a3', grid: {show: false}, ticks: {show: false}};
    const name = lane => `CH${lane.channel + 1}.${lane.bit}`;
    logicPlot = new uPlot({
      width: plotSize().width,
      height: 24 * lanes.length + 40,
      legend: {show: false},
      scales: {x: {time: false}, y: {range: [-0.25, lanes.length * 1.5 - 0.25]}},
      axes: [
        {...axis, values: (u, ticks) => ticks.map(formatTime)},
        {...axis, size: 60, splits: () => lanes.map((_, k) => (lanes.length - 1 - k) * 1.5 + 0.5),
         values: (u, splits) => splits.map(v => name(lanes[lanes.length - 1 - Math.round((v - 0.5) / 1.5)]))},
      ],
      series: [{}, ...lanes.map(lane => ({
        stroke: COLORS[lane.channel], width: 1, paths: uPlot.paths.stepped({align: 1}), points: {show: false},
      }))],
      cursor: {drag: {x: true, y: false, setScale: false}, sync: {key: 'scope'}},
      hooks: {setSelect: [zoomToSelection]},
    }, data, $('logic'));
    logicPlot.key = key;
  }
  logicPlot.setScale('x', {min: capture.time[0], max: capture.time[capture.time.length - 1]});
  logicLanes = lanes;
}

function showViewInfo(h) {
  const [first, last] = h.view;
  const shown = last - first;
  const parts = [`${h.deep ? 'HyperRAM' : 'block RAM'} capture of ${h.length.toLocaleString()} samples`,
                 `${formatTime(h.length / h.sample_rate)} at ${formatRate(h.sample_rate)}`];
  if (shown < h.length) parts.push(`showing ${shown.toLocaleString()} samples from ${formatTime((first - h.pre) / h.sample_rate)}`);
  if (h.envelope) parts.push('min/max envelope, drag to zoom');
  else if (h.length > 64) parts.push('drag to zoom');
  $('view_info').textContent = parts.join(' · ');
}

function updateMeasurements() {
  const body = document.querySelector('#measurements tbody');
  if (!capture) { body.innerHTML = ''; return; }
  const rows = [];
  // computed by the server over every sample of the viewed range
  const stats = capture.header.stats;
  for (let c = 0; c < CHANNELS; c++) {
    if (!visible[c]) continue;
    if (isDigital(c)) {
      rows.push(`<tr><td style="color:${COLORS[c]}">CH${c + 1}</td><td>${probes[capture.header.sources[c]].name}</td>
        <td colspan="5">digital, lanes in the logic view</td></tr>`);
      continue;
    }
    const min = stats.min[c], max = stats.max[c];
    rows.push(`<tr><td style="color:${COLORS[c]}">CH${c + 1}</td><td>${probes[capture.header.sources[c]] ? probes[capture.header.sources[c]].name : ''}</td>
      <td>${min}</td><td>${max}</td><td>${max - min}</td><td>${stats.mean[c].toFixed(1)}</td><td>${stats.rms[c].toFixed(1)}</td></tr>`);
  }
  body.innerHTML = rows.join('');
}

function exportCsv() {
  if (!capture) return;
  const lines = ['time_s,' + Array.from({length: CHANNELS}, (_, i) => `ch${i + 1}`).join(',')];
  for (let s = 0; s < capture.time.length; s++) {
    lines.push(capture.time[s].toPrecision(9) + ',' + capture.channels.map(ch => ch[s]).join(','));
  }
  const link = document.createElement('a');
  link.href = URL.createObjectURL(new Blob([lines.join('\n')], {type: 'text/csv'}));
  link.download = `capture_${capture.header.capture}${capture.header.envelope ? '_envelope' : ''}.csv`;
  link.click();
  URL.revokeObjectURL(link.href);
}

// ------------------------------------------------------------------ status and connection
function showStatus(s) {
  const status = $('status');
  const parts = [s.running ? `running · ${s.state}` : 'stopped', `capture ${s.captures}`];
  if (s.forced) parts.push('forced');
  if (s.download) parts.push(s.download);
  if (s.overflows) parts.push(`${s.overflows} samples lost, sample rate too high for deep capture`);
  if (s.lost_words) parts.push(`${s.lost_words} words re-requested`);
  if (s.error) parts.push(`error: ${s.error}`);
  status.textContent = parts.join(' · ');
  status.classList.toggle('error', !!s.error || !!s.overflows);
  $('run').classList.toggle('active', s.running);
}

function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);
  socket.binaryType = 'arraybuffer';
  socket.onmessage = event => {
    if (event.data instanceof ArrayBuffer) { showCapture(event.data); return; }
    const message = JSON.parse(event.data);
    if (message.type === 'hello') {
      probes = message.probes.length ? message.probes
        : Array.from({length: 16}, (_, s) => ({name: `source ${s}`, format: SIGNED}));
      clockHz = message.clock_hz;
      config = message.config;
      buildControls();
      showConfig(config);
    } else if (message.type === 'status') {
      showStatus(message);
    }
  };
  socket.onclose = () => {
    $('status').textContent = 'disconnected from scope_server.py, retrying…';
    $('status').classList.add('error');
    setTimeout(connect, 1000);
  };
}

window.addEventListener('resize', () => {
  if (plot) plot.setSize(plotSize());
  if (logicPlot) logicPlot.setSize({width: plotSize().width, height: logicPlot.height});
});
connect();
