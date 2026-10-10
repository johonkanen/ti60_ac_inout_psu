// Ti60 waveform: an editor of the output voltage's waveform for the closed loop, as points
// each holding a voltage for a time, played in order (and repeated if asked).
//
// Times are whole periods of the output bridge (25 us, 40 kHz): the control takes a new
// reference once a period, and the FPGA's point table will hold each point's voltage and its
// periods. The plot spans +- the secondary DC link (the bridge can make no more), with the
// control's bridge voltage limit; the expected output is the points through the control's
// reference prefilter (its two first order stages at control_prefilter_hz, from the board's
// config), which the closed loop follows, and its slope gives the capacitor current the
// feedforward asks for (C dv/dt, C the observer's filter capacitance), against the current
// limit.
//
// The points are kept in the browser (localStorage) between visits. Load writes them into the
// FPGA's table (waveform_player, hello.waveform.capacity points), with the passes; Run loads
// them and runs the output bridge's closed loop on them after the DHB's precharge, as the scope
// page's control run does: the run ends with the waveform, or on Stop when it repeats until
// stopped, and the output brought down to 0 V; the DHB runs on as long as the output does.
// Run needs the gate driver supplies and the DHB's voltage control on (the scope page's
// settings).
'use strict';

const PERIOD_MS = 0.025;              // the output bridge's switching period
const PERIODS_A_MS = 40;
const DEFAULT_CAPACITY = 10000;       // the planned FPGA table (~670 kbit of block RAM free)
const PREVIEW_MAX_PERIODS = 4e6;      // longer waveforms are drawn without the expected output
const TABLE_MAX_ROWS = 1000;
const STORAGE_KEY = 'waveform.points';
const UNDO_DEPTH = 100;
const DHB_RUN_MS = 100;               // the DHB's own run after its precharge; it runs on while
                                      // the output does

const $ = id => document.getElementById(id);

// the waveform: [{v: volts, n: periods held}]
let points = [];
let selected = -1;
const undoStack = [];

// the board's
let socket = null;
let config = null;
let dcLink = 94.0;
let waveformSupport = null;      // hello.waveform: {capacity}
let boardWaveform = [0, 0];      // status.waveform: the points loaded, the player's state
let loadedText = null;           // the points as last loaded, to tell a changed waveform

// the view: the time range shown, ms; null for the whole waveform
let view = null;

// the expected output at each period, from the points through the prefilter
let preview = null;
let previewPeak = {current: 0, voltage: 0};

// ------------------------------------------------------------------ the waveform
function totalPeriods() {
  return points.reduce((sum, p) => sum + p.n, 0);
}

function startPeriod(k) {
  let start = 0;
  for (let i = 0; i < k; i++) start += points[i].n;
  return start;
}

function voltageLimit() {
  const limit = config && config.control_u_max ? config.control_u_max : 0.95 * dcLink;
  return Math.min(limit, dcLink);
}

function clampVoltage(v) {
  const limit = voltageLimit();
  return Math.max(-limit, Math.min(limit, v));
}

function snapVoltage(v) {
  const step = +$('snap_v').value;
  return clampVoltage(step > 0 ? Math.round(v / step) * step : v);
}

// a time in ms to whole periods on the time snap
function snapPeriods(ms) {
  const step = Math.max(1, Math.round(+$('snap_t').value * PERIODS_A_MS));
  return Math.max(0, Math.round(ms * PERIODS_A_MS / step) * step);
}

function defaultHold() {
  return Math.max(1, Math.round(+$('default_hold').value * PERIODS_A_MS));
}

function remember() {
  undoStack.push(JSON.stringify(points));
  if (undoStack.length > UNDO_DEPTH) undoStack.shift();
}

function undo() {
  if (!undoStack.length) return;
  points = JSON.parse(undoStack.pop());
  selected = Math.min(selected, points.length - 1);
  changed();
}

function changed() {
  try { localStorage.setItem(STORAGE_KEY, JSON.stringify(points)); } catch (e) { /* private window */ }
  computePreview();
  buildTable();
  showSummary();
  showBoard();
  draw();
}

// a point at a time (ms) and voltage: it splits the level it falls in, or comes after the end
function insertPoint(ms, v) {
  const at = snapPeriods(ms);
  const total = totalPeriods();
  remember();
  if (!points.length || at >= total) {
    if (points.length && at > total) points[points.length - 1].n += at - total;
    points.push({v, n: defaultHold()});
    selected = points.length - 1;
  } else {
    let start = 0;
    for (let k = 0; k < points.length; k++) {
      const end = start + points[k].n;
      if (at < end) {
        if (at === start) {
          points[k].v = v;
          selected = k;
        } else {
          points.splice(k + 1, 0, {v, n: end - at});
          points[k].n = at - start;
          selected = k + 1;
        }
        break;
      }
      start = end;
    }
  }
  changed();
}

// a point removed: its time goes to the point before it (the first's to the next)
function removePoint(k) {
  if (k < 0 || k >= points.length) return;
  remember();
  const n = points[k].n;
  points.splice(k, 1);
  if (points.length) {
    if (k > 0) points[k - 1].n += n;
    else points[0].n += n;
  }
  selected = Math.min(k, points.length - 1);
  changed();
}

// ------------------------------------------------------------------ the expected output
// the control's prefilter, two first order stages, a period at a time from 0 V (a run starts
// at the output's voltage, 0 after the last run's ramp down)
function computePreview() {
  const total = totalPeriods();
  preview = null;
  previewPeak = {current: 0, voltage: 0};
  if (!total || total > PREVIEW_MAX_PERIODS) return;
  const hz = config && config.control_prefilter_hz !== undefined ? config.control_prefilter_hz : 1000;
  const alpha = hz > 0 ? 1 - Math.exp(-2 * Math.PI * hz * PERIOD_MS * 1e-3) : 1;
  const capacitance = config && config.observer_c ? config.observer_c : 18e-6;
  preview = new Float32Array(total);
  let r1 = 0, r2 = 0, k = 0, peak = 0, peakV = 0;
  for (const p of points) {
    for (let i = 0; i < p.n; i++) {
      r1 += alpha * (p.v - r1);
      const step = alpha * (r1 - r2);
      r2 += step;
      peak = Math.max(peak, Math.abs(step));
      peakV = Math.max(peakV, Math.abs(r2));
      preview[k++] = r2;
    }
  }
  previewPeak = {current: capacitance * peak / (PERIOD_MS * 1e-3), voltage: peakV};
}

// ------------------------------------------------------------------ the plot
const canvas = $('editor');
const context = canvas.getContext('2d');
const margin = {left: 52, right: 14, top: 12, bottom: 30};
let size = {width: 800, height: 400};

function colors() {
  const style = getComputedStyle(document.documentElement);
  return {
    text: style.getPropertyValue('--text').trim() || '#d7dde5',
    muted: style.getPropertyValue('--muted').trim() || '#8a94a3',
    grid: style.getPropertyValue('--grid').trim() || '#2a313b',
    accent: style.getPropertyValue('--accent').trim() || '#4aa3ff',
    preview: '#f2b84b',
    limit: '#ff6b6b',
    selected: '#7ee787',
  };
}

function viewRange() {
  const total = Math.max(totalPeriods(), PERIODS_A_MS) / PERIODS_A_MS;
  if (!view) return {from: 0, to: total * 1.05};
  return view;
}

function voltageRange() {
  const extent = Math.max(dcLink, ...points.map(p => Math.abs(p.v)), 10) * 1.08;
  return {low: -extent, high: extent};
}

function xOf(ms) {
  const r = viewRange();
  return margin.left + (ms - r.from) / (r.to - r.from) * (size.width - margin.left - margin.right);
}

function msOf(x) {
  const r = viewRange();
  return r.from + (x - margin.left) / (size.width - margin.left - margin.right) * (r.to - r.from);
}

function yOf(v) {
  const r = voltageRange();
  return margin.top + (r.high - v) / (r.high - r.low) * (size.height - margin.top - margin.bottom);
}

function voltOf(y) {
  const r = voltageRange();
  return r.high - (y - margin.top) / (size.height - margin.top - margin.bottom) * (r.high - r.low);
}

// a grid step of 1, 2 or 5 times a power of ten for about count lines
function niceStep(span, count) {
  const raw = span / count;
  const power = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 5, 10]) if (m * power >= raw) return m * power;
  return 10 * power;
}

function resize() {
  const ratio = window.devicePixelRatio || 1;
  const parent = getComputedStyle(canvas.parentElement);
  const width = canvas.parentElement.clientWidth - parseFloat(parent.paddingLeft) - parseFloat(parent.paddingRight);
  const height = Math.max(260, Math.min(520, Math.round(window.innerHeight * 0.5)));
  size = {width, height};
  canvas.style.width = `${width}px`;
  canvas.style.height = `${height}px`;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  draw();
}

function draw() {
  const c = colors();
  const {width, height} = size;
  context.clearRect(0, 0, width, height);
  context.font = '11px system-ui, sans-serif';
  const r = viewRange();
  const vr = voltageRange();
  const plotRight = width - margin.right;
  const plotBottom = height - margin.bottom;

  // the grid and its labels
  context.strokeStyle = c.grid;
  context.fillStyle = c.muted;
  context.lineWidth = 1;
  const vStep = niceStep(vr.high - vr.low, 8);
  context.textAlign = 'right';
  context.textBaseline = 'middle';
  for (let v = Math.ceil(vr.low / vStep) * vStep; v <= vr.high; v += vStep) {
    const y = Math.round(yOf(v)) + 0.5;
    context.beginPath(); context.moveTo(margin.left, y); context.lineTo(plotRight, y); context.stroke();
    context.fillText(`${+v.toFixed(3)}`, margin.left - 6, y);
  }
  const tStep = niceStep(r.to - r.from, 10);
  context.textAlign = 'center';
  context.textBaseline = 'top';
  for (let t = Math.ceil(r.from / tStep) * tStep; t <= r.to; t += tStep) {
    const x = Math.round(xOf(t)) + 0.5;
    context.beginPath(); context.moveTo(x, margin.top); context.lineTo(x, plotBottom); context.stroke();
    context.fillText(`${+t.toFixed(3)}`, x, plotBottom + 6);
  }
  context.textAlign = 'right';
  context.fillText('ms', plotRight, plotBottom + 18);
  context.save();
  context.translate(12, margin.top + (plotBottom - margin.top) / 2);
  context.rotate(-Math.PI / 2);
  context.textAlign = 'center';
  context.fillText('V', 0, 0);
  context.restore();

  context.save();
  context.beginPath();
  context.rect(margin.left, margin.top, plotRight - margin.left, plotBottom - margin.top);
  context.clip();

  // +- the DC link, and the control's limit
  context.strokeStyle = c.limit;
  context.setLineDash([6, 4]);
  for (const v of [dcLink, -dcLink]) {
    const y = Math.round(yOf(v)) + 0.5;
    context.beginPath(); context.moveTo(margin.left, y); context.lineTo(plotRight, y); context.stroke();
  }
  context.setLineDash([2, 3]);
  context.strokeStyle = c.muted;
  for (const v of [voltageLimit(), -voltageLimit()]) {
    const y = Math.round(yOf(v)) + 0.5;
    context.beginPath(); context.moveTo(margin.left, y); context.lineTo(plotRight, y); context.stroke();
  }
  context.setLineDash([]);

  // the expected output: per pixel column, its range over the column's periods
  if (preview && $('show_preview').checked) {
    context.strokeStyle = c.preview;
    context.lineWidth = 1.2;
    context.beginPath();
    let started = false;
    for (let x = Math.floor(margin.left); x <= plotRight; x++) {
      const a = Math.max(0, Math.floor(msOf(x) * PERIODS_A_MS));
      const b = Math.min(preview.length, Math.floor(msOf(x + 1) * PERIODS_A_MS) + 1);
      if (a >= preview.length || b <= 0 || b <= a) continue;
      let low = Infinity, high = -Infinity;
      for (let i = a; i < b; i++) { low = Math.min(low, preview[i]); high = Math.max(high, preview[i]); }
      if (!started) { context.moveTo(x, yOf(preview[a])); started = true; }
      context.lineTo(x, yOf(high));
      context.lineTo(x, yOf(low));
    }
    context.stroke();
  }

  // the points: their levels and the steps between them
  context.lineWidth = 2;
  let start = 0;
  for (let k = 0; k < points.length; k++) {
    const p = points[k];
    const x0 = xOf(start / PERIODS_A_MS);
    const x1 = xOf((start + p.n) / PERIODS_A_MS);
    if (x1 >= margin.left - 2 && x0 <= plotRight + 2) {
      context.strokeStyle = k === selected ? c.selected : c.accent;
      context.beginPath();
      if (k > 0) context.moveTo(x0, yOf(points[k - 1].v)); else context.moveTo(x0, yOf(0));
      context.lineTo(x0, yOf(p.v));
      context.lineTo(x1, yOf(p.v));
      context.stroke();
      if (x1 - x0 > 6) {
        context.fillStyle = k === selected ? c.selected : c.accent;
        context.beginPath(); context.arc(x0, yOf(p.v), 3, 0, 2 * Math.PI); context.fill();
      }
    }
    start += p.n;
  }
  context.restore();

  context.strokeStyle = c.grid;
  context.strokeRect(margin.left + 0.5, margin.top + 0.5, plotRight - margin.left, plotBottom - margin.top);
}

// ------------------------------------------------------------------ the mouse
// what is under a point of the plot: a step's edge (between point k-1 and k), a level (k), none
function hit(x, y) {
  let start = 0;
  let level = -1;
  for (let k = 0; k < points.length; k++) {
    const x0 = xOf(start / PERIODS_A_MS);
    const x1 = xOf((start + points[k].n) / PERIODS_A_MS);
    if (k > 0 && Math.abs(x - x0) <= 5) {
      const ya = yOf(points[k - 1].v), yb = yOf(points[k].v);
      if (y >= Math.min(ya, yb) - 6 && y <= Math.max(ya, yb) + 6) return {kind: 'edge', k};
    }
    if (x >= x0 && x <= x1 && Math.abs(y - yOf(points[k].v)) <= 6) level = k;
    start += points[k].n;
  }
  return level >= 0 ? {kind: 'level', k: level} : null;
}

let drag = null;

function position(event) {
  const rect = canvas.getBoundingClientRect();
  return {x: event.clientX - rect.left, y: event.clientY - rect.top};
}

canvas.addEventListener('mousedown', event => {
  if (event.button !== 0) return;
  const {x, y} = position(event);
  const target = hit(x, y);
  drag = {target, x, y, moved: false, remembered: false};
  if (target) {
    selected = target.k;
    buildTable();
    draw();
  }
});

window.addEventListener('mousemove', event => {
  const {x, y} = position(event);
  if (x >= margin.left && x <= size.width - margin.right && y >= margin.top && y <= size.height - margin.bottom) {
    $('cursor').textContent = `${msOf(x).toFixed(3)} ms · ${voltOf(y).toFixed(2)} V`;
  }
  if (!drag) {
    const target = event.target === canvas ? hit(x, y) : null;
    canvas.style.cursor = !target ? 'crosshair' : target.kind === 'edge' ? 'ew-resize' : 'ns-resize';
    return;
  }
  if (Math.abs(x - drag.x) + Math.abs(y - drag.y) > 3) drag.moved = true;
  if (!drag.target || !drag.moved) return;
  if (!drag.remembered) { remember(); drag.remembered = true; }
  const k = drag.target.k;
  if (drag.target.kind === 'level') {
    points[k].v = snapVoltage(voltOf(y));
  } else {
    // the edge between k-1 and k: their periods shared out anew, each keeps one at least
    const before = startPeriod(k - 1);
    const both = points[k - 1].n + points[k].n;
    const at = Math.max(before + 1, Math.min(before + both - 1, snapPeriods(msOf(x))));
    points[k - 1].n = at - before;
    points[k].n = both - (at - before);
  }
  computePreview();
  draw();
});

window.addEventListener('mouseup', event => {
  if (!drag) return;
  const d = drag;
  drag = null;
  if (d.moved && d.target) { changed(); return; }
  if (!d.moved && !d.target && event.target === canvas) {
    const {x, y} = position(event);
    if (x >= margin.left && y >= margin.top && y <= size.height - margin.bottom) {
      insertPoint(Math.max(0, msOf(x)), snapVoltage(voltOf(y)));
    }
  }
});

canvas.addEventListener('contextmenu', event => {
  event.preventDefault();
  const {x, y} = position(event);
  const target = hit(x, y);
  if (target && target.kind === 'level') removePoint(target.k);
});

canvas.addEventListener('wheel', event => {
  event.preventDefault();
  const {x} = position(event);
  const r = viewRange();
  const at = msOf(x);
  // as far as the wheel turned: a notch (~100) 15 %, a touchpad's small steps a little each
  const factor = Math.exp(Math.max(-1, Math.min(1, event.deltaY * 0.0015)));
  const span = Math.max(0.1, (r.to - r.from) * factor);
  const from = Math.max(0, at - (at - r.from) * span / (r.to - r.from));
  view = {from, to: from + span};
  draw();
}, {passive: false});

window.addEventListener('keydown', event => {
  if (event.target.tagName === 'INPUT') return;
  if ((event.ctrlKey || event.metaKey) && event.key === 'z') { event.preventDefault(); undo(); }
  else if ((event.key === 'Delete' || event.key === 'Backspace') && selected >= 0) { event.preventDefault(); removePoint(selected); }
});

// ------------------------------------------------------------------ the table and summary
function formatMs(periods) {
  return `${+(periods / PERIODS_A_MS).toFixed(3)}`;
}

function buildTable() {
  const body = $('points').querySelector('tbody');
  body.textContent = '';
  let start = 0;
  const rows = Math.min(points.length, TABLE_MAX_ROWS);
  for (let k = 0; k < rows; k++) {
    const p = points[k];
    const row = document.createElement('tr');
    if (k === selected) row.className = 'current';
    const voltage = document.createElement('input');
    voltage.type = 'number'; voltage.step = 'any'; voltage.value = +p.v.toFixed(4);
    voltage.onchange = () => {
      remember();
      points[k].v = clampVoltage(+voltage.value || 0);
      selected = k;
      changed();
    };
    const hold = document.createElement('input');
    hold.type = 'number'; hold.step = PERIOD_MS; hold.min = PERIOD_MS; hold.value = formatMs(p.n);
    hold.onchange = () => {
      remember();
      points[k].n = Math.max(1, Math.round(+hold.value * PERIODS_A_MS));
      selected = k;
      changed();
    };
    const remove = document.createElement('button');
    remove.className = 'small'; remove.textContent = '×'; remove.title = 'remove the point';
    remove.onclick = () => removePoint(k);
    const cells = [`${k + 1}`, formatMs(start), voltage, hold, remove];
    for (const content of cells) {
      const cell = document.createElement('td');
      if (typeof content === 'string') cell.textContent = content; else cell.appendChild(content);
      row.appendChild(cell);
    }
    row.addEventListener('click', event => {
      if (event.target.tagName === 'INPUT' || event.target.tagName === 'BUTTON') return;
      selected = k;
      buildTable();
      draw();
    });
    body.appendChild(row);
    start += p.n;
  }
  $('points_note').textContent = points.length > TABLE_MAX_ROWS ?
    `the first ${TABLE_MAX_ROWS} of ${points.length} points` : '';
}

// the passes: once, or the count asked (0: until stopped)
function passes() {
  return $('repeat').checked ? Math.max(0, Math.round(+$('repeat_count').value)) : 1;
}

// the run's length in bridge periods: the waveform's passes; 0 until stopped
function runPeriods() {
  return passes() === 0 ? 0 : totalPeriods() * passes();
}

function ready() {
  if (!waveformSupport) return 'the firmware has no waveform player';
  if (!config || config.gate_power !== 63) return 'the gate driver supplies are off (the scope page)';
  if (!config.dhb_control) return "the DHB's voltage control is off (the scope page)";
  if (!points.length) return 'no points';
  if (points.length > capacity()) return `more points than the board's ${capacity()}`;
  return null;
}

function capacity() {
  return waveformSupport && waveformSupport.capacity ? waveformSupport.capacity : DEFAULT_CAPACITY;
}

function showSummary() {
  const total = totalPeriods();
  const lines = [`${points.length} points of ${capacity()} · ${formatMs(total)} ms`];
  if (preview) {
    const limit = config && config.control_i_max ? config.control_i_max : 5;
    lines.push(`expected peak ${previewPeak.voltage.toFixed(1)} V; capacitor current up to ` +
      `${previewPeak.current.toFixed(2)} A (current limit ${limit} A)`);
  } else if (total > PREVIEW_MAX_PERIODS) {
    lines.push('too long for the expected output');
  }
  if (points.length > capacity()) lines.push(`more points than the board's table holds`);
  if (passes() === 0) lines.push('repeated until Stop');
  $('summary').innerHTML = lines.map(l => `<div>${l}</div>`).join('');
}

function showBoard() {
  const parts = [`DC link ${dcLink.toFixed(1)} V · the control's bridge limit ${voltageLimit().toFixed(1)} V`];
  if (config) {
    parts.push(`prefilter ${config.control_prefilter_hz ?? '—'} Hz, feedforward ${config.control_feedforward ?? '—'}`);
  }
  if (waveformSupport) {
    const state = boardWaveform[1];
    const playing = state & 1 ? `playing point ${(state >>> 16) + 1}` : state & 2 ? 'finished' : 'stopped';
    const changedSince = loadedText !== null && loadedText !== JSON.stringify(points) ? ' (changed since)' : '';
    parts.push(`waveform player: ${waveformSupport.capacity} points; ` +
      `${boardWaveform[0] ? `${boardWaveform[0]} loaded${changedSince}` : 'none loaded'}, ${playing}`);
    const problem = ready();
    if (problem) parts.push(`Run: ${problem}`);
  } else {
    parts.push('the firmware has no waveform player: the points stay here');
  }
  $('board').innerHTML = parts.map(l => `<div>${l}</div>`).join('');
  $('load').disabled = !waveformSupport || !points.length || points.length > capacity();
  $('run').disabled = !!ready();
  $('stop').disabled = !waveformSupport;
}

// ------------------------------------------------------------------ generators and files
// one period of a shape at phase 0..1, -1..1
function shapeAt(shape, phase, k, count) {
  switch (shape) {
    case 'sine': return Math.sin(2 * Math.PI * phase);
    case 'triangle': return phase < 0.25 ? 4 * phase : phase < 0.75 ? 2 - 4 * phase : 4 * phase - 4;
    case 'square': return phase < 0.5 ? 1 : -1;
    case 'sawtooth': return 2 * phase - 1;
    case 'steps': return count > 1 ? -1 + 2 * k / (count - 1) : 0;
  }
  return 0;
}

// the shape as points: each period in equal holds, the voltage at each hold's middle; the
// periods rounded so the holds add up to the frequency's period without drift
function generate() {
  const shape = $('gen_shape').value;
  const amplitude = +$('gen_amplitude').value;
  const offset = +$('gen_offset').value;
  const frequency = Math.max(0.1, +$('gen_frequency').value);
  const count = shape === 'square' ? 2 : Math.max(2, Math.round(+$('gen_points').value));
  const periods = Math.max(1, Math.round(+$('gen_periods').value));
  const bridgePeriodsPerCycle = 1000 / frequency * PERIODS_A_MS;
  const out = [];
  for (let c = 0; c < periods; c++) {
    for (let k = 0; k < count; k++) {
      const a = Math.round((c * count + k) * bridgePeriodsPerCycle / count);
      const b = Math.round((c * count + k + 1) * bridgePeriodsPerCycle / count);
      const v = clampVoltage(offset + amplitude * shapeAt(shape, (k + 0.5) / count, k, count));
      if (b - a < 1) continue;
      if (out.length && out[out.length - 1].v === v) out[out.length - 1].n += b - a;
      else out.push({v, n: b - a});
    }
  }
  return out;
}

function exportCsv() {
  const lines = ['voltage_V,hold_ms', ...points.map(p => `${p.v},${formatMs(p.n)}`)];
  const blob = new Blob([lines.join('\n') + '\n'], {type: 'text/csv'});
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = 'waveform.csv';
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 1000);
}

function importCsv(text) {
  const out = [];
  for (const line of text.split(/\r?\n/)) {
    const fields = line.split(/[,;\t]/).map(f => f.trim());
    if (fields.length < 2) continue;
    const v = parseFloat(fields[0]), ms = parseFloat(fields[1]);
    if (!isFinite(v) || !isFinite(ms)) continue;   // the header, blank lines
    out.push({v: clampVoltage(v), n: Math.max(1, Math.round(ms * PERIODS_A_MS))});
  }
  if (!out.length) {
    $('status').textContent = 'no points in that file: voltage_V,hold_ms a line';
    return;
  }
  remember();
  points = out;
  selected = -1;
  view = null;
  changed();
}

// ------------------------------------------------------------------ the board
function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);
  socket.binaryType = 'arraybuffer';
  socket.onmessage = event => {
    if (event.data instanceof ArrayBuffer) return;   // the scope's last capture
    const message = JSON.parse(event.data);
    if (message.type === 'hello') {
      config = message.config;
      waveformSupport = message.waveform || null;
      $('status').textContent = 'connected';
      $('status').classList.remove('error');
      computePreview();
      showSummary();
      showBoard();
      draw();
    } else if (message.type === 'status') {
      if (message.waveform) {
        const before = JSON.stringify(boardWaveform);
        boardWaveform = message.waveform;
        if (JSON.stringify(boardWaveform) !== before) showBoard();
      }
      if (message.dc_links && message.dc_links[1] > 5) {
        const changedLink = Math.abs(message.dc_links[1] - dcLink) > 0.5;
        dcLink = message.dc_links[1];
        if (changedLink) { showBoard(); draw(); }
      }
    }
  };
  socket.onclose = () => {
    $('status').textContent = 'disconnected from the board, retrying…';
    $('status').classList.add('error');
    setTimeout(connect, 1000);
  };
}

// ------------------------------------------------------------------ start
$('fit').onclick = () => { view = null; draw(); };
$('show_preview').onchange = draw;
$('gen_replace').onclick = () => { remember(); points = generate(); selected = -1; view = null; changed(); };
$('gen_append').onclick = () => { remember(); points = points.concat(generate()); view = null; changed(); };
$('export').onclick = exportCsv;
$('import').onclick = () => $('import_file').click();
$('import_file').onchange = async () => {
  const file = $('import_file').files[0];
  if (file) importCsv(await file.text());
  $('import_file').value = '';
};
$('clear').onclick = () => { if (points.length) { remember(); points = []; selected = -1; view = null; changed(); } };
$('add_row').onclick = () => insertPoint(totalPeriods() / PERIODS_A_MS, points.length ? points[points.length - 1].v : 0);
function load() {
  send({type: 'waveform_load', points: points.map(p => [+p.v.toFixed(4), p.n]), passes: passes()});
  loadedText = JSON.stringify(points);
}

$('load').onclick = () => { load(); showBoard(); };
$('run').onclick = () => {
  const problem = ready();
  if (problem) { $('status').textContent = problem; return; }
  const periods = runPeriods();
  const count = passes();
  const what = count === 0 ? 'repeated until Stop' : count === 1 ? 'once' : `${count} times`;
  const length = count === 0 ? '' : `, ${(periods / PERIODS_A_MS).toFixed(1)} ms`;
  if (!confirm(`Output bridge closed loop on the waveform: ${points.length} points, ${what}${length}, ` +
               `after the DHB's precharge (the secondary inverted, the current limited to ` +
               `${config.control_i_max} A); then the output brought down to 0 V?`)) return;
  load();
  // a few periods more than the waveform, so it ends the run itself; 0: until Stop
  send({type: 'waveform_run', periods: count === 0 ? 0 : Math.round(periods) + 10});
  send({type: 'dhb_test', periods: 75000, pulse: 40, ramp: 124, run: DHB_RUN_MS * 150, invert: 1});
  showBoard();
};
$('stop').onclick = () => send({type: 'waveform_stop'});
for (const id of ['repeat', 'repeat_count']) $(id).addEventListener('change', () => { showSummary(); showBoard(); });
for (const id of ['snap_v', 'snap_t', 'default_hold', 'repeat', 'repeat_count']) {
  $(id).addEventListener('change', () => {
    try { localStorage.setItem(`waveform.${id}`, $(id).type === 'checkbox' ? $(id).checked : $(id).value); } catch (e) { /* */ }
  });
  try {
    const saved = localStorage.getItem(`waveform.${id}`);
    if (saved !== null) { if ($(id).type === 'checkbox') $(id).checked = saved === 'true'; else $(id).value = saved; }
  } catch (e) { /* */ }
}

try {
  const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || 'null');
  if (Array.isArray(saved)) points = saved.filter(p => isFinite(p.v) && p.n >= 1).map(p => ({v: +p.v, n: Math.round(p.n)}));
} catch (e) { /* none kept */ }
if (!points.length) points = [{v: -20, n: 200}, {v: 20, n: 200}, {v: 0, n: 200}];

window.addEventListener('resize', resize);
computePreview();
buildTable();
showSummary();
showBoard();
resize();
connect();
