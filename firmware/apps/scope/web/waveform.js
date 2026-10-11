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
// The generator makes points of a shape, among them a grid's voltage: a sine and its
// harmonics (orders, amplitudes in % of the fundamental, phases), optionally pre-compensated
// for the control's prefilter, which passes higher orders weaker and later; noise (Gaussian,
// seeded) goes on what it makes or on the points as they are. Under the plot the harmonics of
// a period of the points and of the expected output are shown against the set ones, with THD.
//
// The points are kept in the browser (localStorage) between visits. Load writes them into the
// FPGA's table (waveform_player, hello.waveform.capacity points), with the passes; Run loads
// them and runs the output bridge's closed loop on them after the DHB's precharge, as the scope
// page's control run does: the run ends with the waveform, or on Stop when it repeats until
// stopped, and the output brought down to 0 V; the DHB runs on as long as the output does.
// Run needs the gate driver supplies and the DHB's voltage control on (the scope page's
// settings). The page sends a heartbeat every second: the board brings a run down when no page
// or script has said anything for a few seconds.
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
let boardOutput = 0;             // status.output[0]: bit3 the last run stopped by the watchdog
let sourceNames = [];

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
  analyse();
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
    if (boardOutput & 8) parts.push('the last run was stopped by the watchdog: no page was there');
    // the banner's Start runs these points (loading them first)
    const problem = !points.length ? 'no points' :
      points.length > capacity() ? `more points than the board's ${capacity()}` : null;
    parts.push(problem ? `Start: ${problem}` : "the banner's Start loads these points and runs them, its Stop ends the run");
  } else {
    parts.push('the firmware has no waveform player: the points stay here');
  }
  $('board').innerHTML = parts.map(l => `<div>${l}</div>`).join('');
  $('load').disabled = !waveformSupport || !points.length || points.length > capacity();
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

// ------------------------------------------------------------------ the grid's harmonics
// [order, % of the fundamental, phase in degrees]: the phase of sin(n wt + phase), 0 on
// orders 3, 7, 11.. and 180 on 5, 9, 13.. lowering the peak (a flat top, as rectifier loads make)
const HARMONIC_PRESETS = {
  clean: [],
  lv: [[3, 1.5, 0], [5, 4, 180], [7, 3, 0], [11, 1.5, 0], [13, 1, 180]],
  flat: [[3, 3, 0], [5, 2, 180]],
  en50160: [[3, 5, 0], [5, 6, 180], [7, 5, 0], [9, 1.5, 180], [11, 3.5, 0], [13, 3, 180]],
  rectifier: [[5, 4, 180], [7, 2.5, 0], [11, 1.5, 0], [13, 1, 180]],
};
let harmonics = HARMONIC_PRESETS.lv.map(h => h.slice());
let lastGrid = null;      // the grid last generated: {frequency, harmonics}, for the analysis

function prefilterHz() {
  return config && config.control_prefilter_hz !== undefined ? config.control_prefilter_hz : 1000;
}

// the control's prefilter at a frequency: two first order stages a bridge period each,
// H(z) = (a / (1 - (1 - a) z^-1))^2; {gain, phase (rad)}
function prefilterResponse(hz) {
  const f = prefilterHz();
  if (!(f > 0)) return {gain: 1, phase: 0};
  const a = 1 - Math.exp(-2 * Math.PI * f * PERIOD_MS * 1e-3);
  const w = 2 * Math.PI * hz * PERIOD_MS * 1e-3;
  // 1 - (1 - a) e^-jw
  const re = 1 - (1 - a) * Math.cos(w), im = (1 - a) * Math.sin(w);
  const gain1 = a / Math.hypot(re, im);
  const phase1 = -Math.atan2(im, re);
  return {gain: gain1 * gain1, phase: 2 * phase1};
}

// the closed loop after the prefilter, from the points to the output voltage (the observer's),
// measured on the board on 2026-10-10 by measure_loop_response.py with tones at its tuning then
// (voltage loop 1000 Hz, its zero 0.1, b 1, current loop 3000 Hz, prefilter 1000 Hz, feedforward
// 1): [Hz, gain, phase in degrees]; between the tones interpolated, beyond them the nearest's
const LOOP_RESPONSE = [[50, 0.981, 0.3], [150, 1.154, 2.4], [250, 1.171, -8.1],
                       [350, 1.084, -11.4], [450, 1.067, -12.3], [550, 1.065, -9.5],
                       [650, 1.075, -11.5]];

// the table in use: the board's measured one (loop_response.json, from the Bode section's "use
// for the compensation") when it has one, else the built-in above
let loopTable = LOOP_RESPONSE;
let loopTableSource = 'built in';

function loopResponse(hz) {
  const r = loopTable;
  if (hz <= r[0][0]) return {gain: r[0][1], phase: r[0][2] * Math.PI / 180};
  for (let i = 1; i < r.length; i++) {
    if (hz <= r[i][0]) {
      const x = (hz - r[i - 1][0]) / (r[i][0] - r[i - 1][0]);
      return {gain: r[i - 1][1] + x * (r[i][1] - r[i - 1][1]),
              phase: (r[i - 1][2] + x * (r[i][2] - r[i - 1][2])) * Math.PI / 180};
    }
  }
  const last = r[r.length - 1];
  return {gain: last[1], phase: last[2] * Math.PI / 180};
}

function thdOf(list) {
  return Math.sqrt(list.reduce((sum, [, percent]) => sum + percent * percent, 0));
}

function buildHarmonics() {
  const body = $('harmonics').querySelector('tbody');
  body.textContent = '';
  harmonics.forEach((h, i) => {
    const row = document.createElement('tr');
    const fields = [['order', 1, 2, 199], ['percent', 0.1, 0, 100], ['phase', 5, -360, 360]];
    fields.forEach(([, step, min, max], j) => {
      const input = document.createElement('input');
      input.type = 'number'; input.step = step; input.min = min; input.max = max; input.value = h[j];
      input.onchange = () => {
        const value = +input.value;
        harmonics[i][j] = j === 0 ? Math.max(2, Math.round(value)) : value;
        harmonicsChanged();
      };
      const cell = document.createElement('td'); cell.appendChild(input); row.appendChild(cell);
    });
    const remove = document.createElement('button');
    remove.className = 'small'; remove.textContent = '×'; remove.title = 'remove the harmonic';
    remove.onclick = () => { harmonics.splice(i, 1); harmonicsChanged(); };
    const cell = document.createElement('td'); cell.appendChild(remove); row.appendChild(cell);
    body.appendChild(row);
  });
  $('harmonics_thd').textContent = `THD ${thdOf(harmonics).toFixed(2)} %`;
}

function harmonicsChanged() {
  try { localStorage.setItem('waveform.harmonics', JSON.stringify(harmonics)); } catch (e) { /* */ }
  buildHarmonics();
}

// the grid's components: the fundamental and the harmonics as {order, amplitude (of the
// fundamental's 1), phase (rad)}, pre-compensated for the prefilter and the holds if asked
function gridComponents(frequency, holdSeconds) {
  const list = [{order: 1, amplitude: 1, phase: 0}].concat(harmonics.map(([order, percent, degrees]) =>
    ({order: Math.round(order), amplitude: percent / 100, phase: degrees * Math.PI / 180})));
  if (!$('compensate').checked) return list;
  return list.map(c => {
    const hz = c.order * frequency;
    const h = prefilterResponse(hz);
    // the holds: each point a step of holdSeconds, sin(x)/x of the frequency (its delay is
    // already off: each point is the value at its hold's middle)
    const x = Math.PI * hz * holdSeconds;
    const hold = x > 1e-9 ? Math.sin(x) / x : 1;
    const loop = $('compensate_loop').checked ? loopResponse(hz) : {gain: 1, phase: 0};
    return {order: c.order, amplitude: c.amplitude / (h.gain * hold * loop.gain),
            phase: c.phase - h.phase - loop.phase};
  });
}

// noise: a seeded generator (mulberry32) and Gaussian values from it (Box-Muller)
function seededRandom(seed) {
  let a = (seed >>> 0) || 1;
  return () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function gaussianSource(seed) {
  const random = seededRandom(seed);
  return () => {
    const u = Math.max(random(), 1e-12), v = random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  };
}

// the shape as points: each period in equal holds, the voltage at each hold's middle; the
// periods rounded so the holds add up to the frequency's period without drift
let clipped = 0;
function generate() {
  const shape = $('gen_shape').value;
  const amplitude = +$('gen_amplitude').value;
  const offset = +$('gen_offset').value;
  const frequency = Math.max(0.1, +$('gen_frequency').value);
  const count = shape === 'square' ? 2 : Math.max(2, Math.round(+$('gen_points').value));
  const periods = Math.max(1, Math.round(+$('gen_periods').value));
  const bridgePeriodsPerCycle = 1000 / frequency * PERIODS_A_MS;
  const components = shape === 'grid' ? gridComponents(frequency, 1 / (frequency * count)) : null;
  const rms = +$('noise_rms').value;
  const noise = $('noise_generated').checked && rms > 0 ? gaussianSource(+$('noise_seed').value) : null;
  const out = [];
  clipped = 0;
  for (let c = 0; c < periods; c++) {
    for (let k = 0; k < count; k++) {
      const a = Math.round((c * count + k) * bridgePeriodsPerCycle / count);
      const b = Math.round((c * count + k + 1) * bridgePeriodsPerCycle / count);
      if (b - a < 1) continue;
      const phase = (k + 0.5) / count;
      let value = components ?
        components.reduce((sum, h) => sum + h.amplitude * Math.sin(2 * Math.PI * h.order * phase + h.phase), 0) :
        shapeAt(shape, phase, k, count);
      let v = offset + amplitude * value;
      if (noise) v += rms * noise();
      const limited = clampVoltage(v);
      if (limited !== v) clipped++;
      v = +limited.toFixed(4);
      if (out.length && out[out.length - 1].v === v) out[out.length - 1].n += b - a;
      else out.push({v, n: b - a});
    }
  }
  lastGrid = shape === 'grid' ? {frequency, harmonics: harmonics.map(h => h.slice())} : null;
  $('status').textContent = clipped ? `${clipped} points beyond the bridge's limit, clipped` : $('status').textContent;
  return out;
}

// noise on the points as they are
function addNoise() {
  const rms = +$('noise_rms').value;
  if (!(rms > 0) || !points.length) return;
  remember();
  const noise = gaussianSource(+$('noise_seed').value);
  for (const p of points) p.v = +clampVoltage(p.v + rms * noise()).toFixed(4);
  changed();
}

// ------------------------------------------------------------------ the harmonics of the waveform
// a period of the fundamental (the generator's frequency) at the end of the waveform, a bridge
// period a sample: of the points, and of the expected output (the prefilter settled: the
// waveform played twice when it is one period long); each order's amplitude and the THD
const ANALYSIS_ORDERS = 25;

function harmonicsOf(signal, start, length) {
  const out = [];
  for (let n = 1; n <= ANALYSIS_ORDERS; n++) {
    let re = 0, im = 0;
    const w = 2 * Math.PI * n / length;
    for (let k = 0; k < length; k++) {
      re += signal[start + k] * Math.cos(w * k);
      im -= signal[start + k] * Math.sin(w * k);
    }
    out.push(2 * Math.hypot(re, im) / length);
  }
  return out;
}

function analyse() {
  const target = $('spectrum');
  const frequency = Math.max(0.1, +$('gen_frequency').value);
  const length = Math.round(1000 / frequency * PERIODS_A_MS);
  const total = totalPeriods();
  if (!total || total < length || total > PREVIEW_MAX_PERIODS || length < 8) { target.innerHTML = ''; return; }
  const steps = new Float32Array(total);
  let k = 0;
  for (const p of points) for (let i = 0; i < p.n; i++) steps[k++] = p.v;
  // the expected output settled: the points twice through the prefilter, the second time's
  const f = prefilterHz();
  const alpha = f > 0 ? 1 - Math.exp(-2 * Math.PI * f * PERIOD_MS * 1e-3) : 1;
  const settled = new Float32Array(total);
  let r1 = 0, r2 = 0;
  for (let pass = 0; pass < 2; pass++) {
    for (let i = 0; i < total; i++) {
      r1 += alpha * (steps[i] - r1);
      r2 += alpha * (r1 - r2);
      if (pass === 1) settled[i] = r2;
    }
  }
  const start = total - length;
  const a = harmonicsOf(steps, start, length);
  const e = harmonicsOf(settled, start, length);
  // with the closed loop's measured gain too: an estimate of the output's
  const l = e.map((x, i) => x * loopResponse((i + 1) * frequency).gain);
  const set = lastGrid && Math.abs(lastGrid.frequency - frequency) < 1e-9 ? lastGrid.harmonics : null;
  const percent = (list, n) => list[0] > 1e-9 ? 100 * list[n - 1] / list[0] : 0;
  const orders = new Set(set ? set.map(h => Math.round(h[0])).filter(n => n <= ANALYSIS_ORDERS) : []);
  for (let n = 2; n <= ANALYSIS_ORDERS; n++) if (percent(a, n) >= 0.3 || percent(e, n) >= 0.3) orders.add(n);
  const thd = list => list[0] > 1e-9 ?
    100 * Math.sqrt(list.slice(1).reduce((sum, x) => sum + x * x, 0)) / list[0] : 0;
  const setPercent = n => { const h = set && set.find(h => Math.round(h[0]) === n); return h ? h[1] : 0; };
  const rows = [...orders].sort((x, y) => x - y).map(n =>
    `<tr><td>${n}</td><td>${set ? setPercent(n).toFixed(2) : '—'}</td><td>${percent(a, n).toFixed(2)}</td>` +
    `<td>${percent(e, n).toFixed(2)}</td><td>${percent(l, n).toFixed(2)}</td></tr>`).join('');
  target.innerHTML =
    `<table class="compact"><thead><tr><th>Order (of ${+frequency.toFixed(3)} Hz)</th><th>Set %</th>` +
    `<th>Points %</th><th>Expected output %</th><th>With the loop %</th></tr></thead><tbody>` +
    `<tr><td>1 (V peak)</td><td>${set ? (+$('gen_amplitude').value).toFixed(2) : '—'}</td>` +
    `<td>${a[0].toFixed(2)}</td><td>${e[0].toFixed(2)}</td><td>${l[0].toFixed(2)}</td></tr>${rows}` +
    `<tr><td>THD to the ${ANALYSIS_ORDERS}th</td><td>${set ? thdOf(set).toFixed(2) : '—'}</td>` +
    `<td>${thd(a).toFixed(2)}</td><td>${thd(e).toFixed(2)}</td><td>${thd(l).toFixed(2)}</td></tr></tbody></table>` +
    `<p class="note">The last period of ${+frequency.toFixed(3)} Hz; the expected output is the ` +
    `control's prefilter (${f} Hz) settled; with the loop, also the closed loop's gain from the ` +
    `loop table in use (${loopTableSource}; an estimate: other tunings differ).</p>`;
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

// ------------------------------------------------------------------ the frequency response
// a tone a frequency through the waveform player, the closed loop on it; the observer's voltage
// in a scope capture against the points at the same instants
const BODE_CAPTURE = 32768;           // samples a capture
const bode = {running: false, stop: false, results: [], waitCapture: null, plots: []};

function sleep(ms) { return new Promise(resolve => setTimeout(resolve, ms)); }

// log spaced, three significant digits
function bodeFrequencies() {
  const from = Math.max(5, +$('bode_from').value), to = Math.max(from * 1.01, +$('bode_to').value);
  const count = Math.max(2, Math.min(40, Math.round(+$('bode_count').value)));
  return Array.from({length: count}, (_, i) => +(from * Math.pow(to / from, i / (count - 1))).toPrecision(3));
}

// a tone's table: whole cycles in it (20 ms or so, a cycle at least), its points at most 500, each
// held a whole number of bridge periods; the frequency as the table makes it
function toneTable(hz, amplitude) {
  const cycles = Math.max(1, Math.round(hz / 50));
  const periods = Math.round(cycles / hz * 1000 * PERIODS_A_MS);
  const hold = Math.max(1, Math.ceil(periods / 500));
  const count = Math.max(2, Math.round(periods / hold));
  const length = count * hold;    // the table's bridge periods
  const actual = cycles * 1000 * PERIODS_A_MS / length;
  const pts = Array.from({length: count}, (_, k) =>
    [+(amplitude * Math.sin(2 * Math.PI * cycles * (k + 0.5) / count)).toFixed(4), hold]);
  return {points: pts, hold, length, hz: actual, seconds: length / (1000 * PERIODS_A_MS)};
}

function complexDiv(a, b) {
  const d = b.re * b.re + b.im * b.im;
  return {re: (a.re * b.re + a.im * b.im) / d, im: (a.im * b.re - a.re * b.im) / d};
}

function prefilterComplex(hz) {
  const h = prefilterResponse(hz);
  return {re: h.gain * Math.cos(h.phase), im: h.gain * Math.sin(h.phase)};
}

// one tone: loaded, run, captured; its closed loop response (complex)
async function measureTone(hz, amplitude) {
  const tone = toneTable(hz, amplitude);
  // a capture of the first tables' settling, then 4 cycles at least
  const settle = Math.max(0.02, tone.seconds);
  const captureSeconds = Math.max(0.13, settle + 4 / tone.hz + 0.01);
  const divider = Math.max(249, Math.ceil(125e6 * captureSeconds / BODE_CAPTURE) - 1);
  const runSeconds = captureSeconds + 0.05;
  const passes = Math.ceil(runSeconds / tone.seconds);
  send({type: 'waveform_load', points: tone.points, passes});
  const want = ['observer v', 'sec 2 vfilt_out', 'sdm iac_out mean', 'control i_ref', 'control u_ref',
                'observer i', 'dhb test', 'output test'];
  send({type: 'config', config: {divider, length: BODE_CAPTURE, pre_percent: 5, mode: 'normal',
                                 trigger_channel: 6, trigger_edge: 0, trigger_level: 8,
                                 sources: want.map(n => sourceNames.indexOf(n))}});
  await sleep(300);
  send({type: 'single'});
  await sleep(300);
  const captured = new Promise((resolve, reject) => {
    bode.waitCapture = resolve;
    setTimeout(() => reject(new Error('no capture')), 15000);
  });
  send({type: 'waveform_run', periods: passes * tone.length + 10});
  send({type: 'dhb_test', periods: 75000, pulse: 40, ramp: 124, run: DHB_RUN_MS * 150, invert: 1});
  const buffer = await captured;
  await sleep(runSeconds * 1000 + 400);       // the run over, the output brought down
  // the capture: a header's length, the header, then 8 channels of 16 bit samples
  const view = new DataView(buffer);
  const headerLength = view.getUint32(0, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, headerLength)));
  const samples = new Int16Array(buffer.slice(4 + headerLength));
  const n = header.length, fs = header.sample_rate;
  let r0 = -1;
  for (let i = 0; i < n; i++) if (samples[i * 8 + 7] & 1) { r0 = i; break; }
  if (r0 < 0) throw new Error('the output did not run');
  const start = r0 + Math.round(settle * fs);
  const cycles = Math.floor((n - start) / fs * tone.hz);
  if (cycles < 2) throw new Error('too short a capture');
  const span = Math.round(cycles * fs / tone.hz);
  let o = {re: 0, im: 0}, r = {re: 0, im: 0};
  for (let i = 0; i < span; i++) {
    const t = (start - r0 + i) / fs;
    const v = samples[(start + i) * 8] / 64;    // the observer's voltage, 64 counts a volt
    const period = Math.floor(t * 1000 * PERIODS_A_MS) % tone.length;
    const ref = tone.points[Math.floor(period / tone.hold)][0];
    const w = 2 * Math.PI * tone.hz * t;
    o.re += v * Math.cos(w); o.im -= v * Math.sin(w);
    r.re += ref * Math.cos(w); r.im -= ref * Math.sin(w);
  }
  return {hz: tone.hz, h: complexDiv(o, r)};
}

const gainDb = h => 20 * Math.log10(Math.hypot(h.re, h.im));
const phaseDeg = h => Math.atan2(h.im, h.re) * 180 / Math.PI;

async function measureBode() {
  const problem = ready();
  // (the waveform's own points do not matter here)
  if (problem && !/points/.test(problem)) { $('bode_status').textContent = problem; return; }
  const amplitude = Math.max(1, Math.min(60, +$('bode_amplitude').value));
  const freqs = bodeFrequencies();
  const current = freqs.map(f => (config.observer_c || 18e-6) * 2 * Math.PI * f * amplitude);
  if (!confirm(`Measure the closed loop's response: ${freqs.length} tones of ${amplitude} V from ` +
               `${freqs[0]} to ${freqs[freqs.length - 1]} Hz (the capacitor current up to ` +
               `${Math.max(...current).toFixed(1)} A), each a run of the power stage after the DHB's ` +
               `precharge, as Run?`)) return;
  const keep = {};
  for (const k of ['divider', 'length', 'pre_percent', 'mode', 'trigger_channel', 'trigger_edge',
                   'trigger_level', 'sources']) keep[k] = config[k];
  bode.running = true; bode.stop = false; bode.results = [];
  $('bode_measure').disabled = true;
  try {
    for (const [i, f] of freqs.entries()) {
      if (bode.stop) break;
      $('bode_status').textContent = `tone ${i + 1} of ${freqs.length}: ${f} Hz…`;
      const result = await measureTone(f, amplitude);
      const p = prefilterComplex(result.hz);
      bode.results.push({hz: result.hz, h: result.h, loop: complexDiv(result.h, p)});
      showBode();
    }
    $('bode_status').textContent = bode.stop ? 'stopped' : `done: ${bode.results.length} tones`;
  } catch (e) {
    $('bode_status').textContent = `stopped: ${e.message}`;
    send({type: 'waveform_stop'});
  } finally {
    bode.running = false;
    bode.waitCapture = null;
    $('bode_measure').disabled = false;
    send({type: 'config', config: keep});
    loadedText = null;       // the player holds the tones now
  }
}

function showBode() {
  const body = $('bode_table').querySelector('tbody');
  body.textContent = '';
  for (const r of bode.results) {
    const row = document.createElement('tr');
    for (const text of [r.hz.toFixed(1), Math.hypot(r.h.re, r.h.im).toFixed(3), phaseDeg(r.h).toFixed(1),
                        Math.hypot(r.loop.re, r.loop.im).toFixed(3), phaseDeg(r.loop).toFixed(1)]) {
      const cell = document.createElement('td'); cell.textContent = text; row.appendChild(cell);
    }
    body.appendChild(row);
  }
  drawBode();
}

// the two plots, gain (dB) and phase, over a log frequency axis: the measured closed loop and the
// loop after the prefilter, the prefilter's model, the compensation table in use
function drawBode() {
  const container = $('bode_plots');
  for (const u of bode.plots) u.destroy();
  bode.plots = [];
  container.textContent = '';
  if (typeof uPlot === 'undefined') return;
  const measured = bode.results;
  const lo = Math.min(+$('bode_from').value, ...measured.map(r => r.hz), ...loopTable.map(r => r[0])) / 1.5;
  const hi = Math.max(+$('bode_to').value, ...measured.map(r => r.hz), ...loopTable.map(r => r[0])) * 1.5;
  const grid = Array.from({length: 80}, (_, i) => lo * Math.pow(hi / lo, i / 79));
  const xs = [...new Set(grid.concat(measured.map(r => r.hz), loopTable.map(r => r[0])))].sort((a, b) => a - b);
  const at = (list, x, get) => { const e = list.find(item => item.x === x); return e ? get(e) : null; };
  const closed = measured.map(r => ({x: r.hz, h: r.h}));
  const after = measured.map(r => ({x: r.hz, h: r.loop}));
  const table = loopTable.map(r => ({x: r[0], gain: r[1], phase: r[2]}));
  const gridSet = new Set(grid);
  const model = x => gridSet.has(x) ? prefilterComplex(x) : null;
  const width = container.clientWidth || 800;
  const c = colors();
  const series = [{label: 'Hz', value: (u, v) => v == null ? '--' : v.toPrecision(4)}, {label: 'closed loop, measured', stroke: c.accent, width: 2, spanGaps: true, points: {show: true, size: 6}},
                  {label: 'loop after the prefilter, measured', stroke: c.selected, width: 2, spanGaps: true, points: {show: true, size: 6}},
                  {label: 'prefilter, model', stroke: c.preview, width: 1.5, spanGaps: true, points: {show: false}},
                  {label: `compensation table (${loopTableSource})`, stroke: c.muted, width: 1.5, dash: [6, 4], spanGaps: true, points: {show: true, size: 4}}];
  for (const [label, get, unit] of [['gain', h => h ? gainDb(h) : null, 'dB'], ['phase', h => h ? phaseDeg(h) : null, '°']]) {
    const data = [xs,
      xs.map(x => at(closed, x, e => get(e.h))),
      xs.map(x => at(after, x, e => get(e.h))),
      xs.map(x => get(model(x))),
      xs.map(x => at(table, x, e => label === 'gain' ? 20 * Math.log10(e.gain) : e.phase))];
    const div = document.createElement('div');
    container.appendChild(div);
    bode.plots.push(new uPlot({
      width, height: 220,
      scales: {x: {time: false, distr: 3, log: 10}},
      axes: [{stroke: c.muted, grid: {stroke: c.grid}, label: 'Hz'},
             {stroke: c.muted, grid: {stroke: c.grid}, label: `${label} (${unit})`, size: 60}],
      series: series.map(s => Object.assign({}, s)),
      legend: {show: label === 'gain'},
    }, data, div));
  }
}

async function loadLoopTable() {
  try {
    const r = await fetch('loop_response.json', {cache: 'no-store'});
    if (!r.ok) return;
    const saved = await r.json();
    if (Array.isArray(saved.table) && saved.table.length >= 2) {
      loopTable = saved.table;
      loopTableSource = `measured ${saved.date}`;
      analyse();
      drawBode();
    }
  } catch (e) { /* the built-in one */ }
  $('bode_table_status').textContent = `the compensation's loop table: ${loopTableSource}`;
}

async function saveLoopTable() {
  if (!bode.results.length) { $('bode_table_status').textContent = 'measure first'; return; }
  const table = bode.results.map(r => [+r.hz.toFixed(1), +Math.hypot(r.loop.re, r.loop.im).toFixed(4),
                                       +phaseDeg(r.loop).toFixed(2)]).sort((a, b) => a[0] - b[0]);
  const tuning = ['control_voltage_hz', 'control_voltage_zero', 'control_b', 'control_current_hz',
                  'control_prefilter_hz', 'control_feedforward'].map(k => `${k} ${config[k]}`).join(', ');
  const now = new Date();
  const pad = n => String(n).padStart(2, '0');
  const date = `${now.getFullYear()}-${pad(now.getMonth() + 1)}-${pad(now.getDate())} ${pad(now.getHours())}:${pad(now.getMinutes())}`;
  const saved = {date, tuning,
                 amplitude: +$('bode_amplitude').value, table};
  const body = new Blob([JSON.stringify(saved)]).stream().pipeThrough(new CompressionStream('gzip'));
  try {
    const r = await fetch('/files/loop_response.json.gz', {method: 'POST', body: await new Response(body).arrayBuffer(),
                                                           headers: {'Content-Type': 'application/octet-stream'}});
    const result = await r.json();
    if (!result.ok) throw new Error(result.message || r.status);
    loopTable = table;
    loopTableSource = `measured ${saved.date}`;
    $('bode_table_status').textContent = `the compensation's loop table: ${loopTableSource}, on the board`;
    analyse();
    drawBode();
  } catch (e) {
    $('bode_table_status').textContent = `not saved: ${e.message}`;
  }
}

async function builtinLoopTable() {
  try { await fetch('/files/loop_response.json.gz', {method: 'DELETE'}); } catch (e) { /* none there */ }
  loopTable = LOOP_RESPONSE;
  loopTableSource = 'built in';
  $('bode_table_status').textContent = `the compensation's loop table: ${loopTableSource}`;
  analyse();
  drawBode();
}

// ------------------------------------------------------------------ the board
function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);
  socket.binaryType = 'arraybuffer';
  socket.onmessage = event => {
    if (event.data instanceof ArrayBuffer) {
      // a capture: the Bode measurement's when it waits for one (else the scope's last)
      if (bode.waitCapture) { bode.waitCapture(event.data); bode.waitCapture = null; }
      return;
    }
    const message = JSON.parse(event.data);
    if (message.type === 'hello') {
      config = message.config;
      sourceNames = message.source_names || [];
      waveformSupport = message.waveform || null;
      $('status').textContent = 'connected';
      boardOutput = 0;
      $('status').classList.remove('error');
      computePreview();
      analyse();
      showSummary();
      showBoard();
      draw();
    } else if (message.type === 'status') {
      Banner.status(message);
      if (message.waveform) {
        const before = JSON.stringify([boardWaveform, boardOutput]);
        boardWaveform = message.waveform;
        boardOutput = message.output ? message.output[0] : 0;
        if (JSON.stringify([boardWaveform, boardOutput]) !== before) showBoard();
      }
      if (message.dc_links && message.dc_links[1] > 5) {
        const changedLink = Math.abs(message.dc_links[1] - dcLink) > 0.5;
        dcLink = message.dc_links[1];
        if (changedLink) { showBoard(); draw(); }
      }
    }
  };
  socket.onclose = () => {
    Banner.disconnected();
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
$('noise_add').onclick = addNoise;
$('bode_measure').onclick = measureBode;
$('bode_stop').onclick = () => { bode.stop = true; send({type: 'waveform_stop'}); };
$('bode_use').onclick = saveLoopTable;
$('bode_builtin').onclick = builtinLoopTable;
$('bode_section').addEventListener('toggle', () => { if ($('bode_section').open) drawBode(); });
$('gen_shape').onchange = () => {
  const grid = $('gen_shape').value === 'grid';
  $('grid_editor').hidden = !grid;
  // a grid's period wants many points for its harmonics: 200 a period leaves 15 to the 13th
  if (grid && +$('gen_points').value < 100) { $('gen_points').value = 200; $('gen_periods').value = 1; }
};
$('gen_frequency').addEventListener('change', analyse);
$('harmonic_preset').onchange = () => {
  const preset = HARMONIC_PRESETS[$('harmonic_preset').value];
  if (preset) { harmonics = preset.map(h => h.slice()); harmonicsChanged(); }
  $('harmonic_preset').value = '';
};
$('add_harmonic').onclick = () => {
  const next = harmonics.length ? Math.max(...harmonics.map(h => h[0])) + 2 : 3;
  harmonics.push([next, 1, 0]);
  harmonicsChanged();
};
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
for (const id of ['repeat', 'repeat_count']) $(id).addEventListener('change', () => { showSummary(); showBoard(); });
for (const id of ['snap_v', 'snap_t', 'default_hold', 'repeat', 'repeat_count', 'noise_rms', 'noise_seed',
                  'noise_generated', 'compensate', 'compensate_loop']) {
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
try {
  const saved = JSON.parse(localStorage.getItem('waveform.harmonics') || 'null');
  if (Array.isArray(saved)) harmonics = saved.filter(h => Array.isArray(h) && h.length === 3).map(h => h.map(Number));
} catch (e) { /* none kept */ }
buildHarmonics();

// the board's watchdog: a run goes on only while a page is there; this one says so every second
setInterval(() => send({type: 'heartbeat'}), 1000);
Banner.useSocket(send);
// the banner's Run: this page's waveform, loaded first
Banner.runSource({
  check: () => {
    if (!waveformSupport) return 'the firmware has no waveform player';
    if (!points.length) return 'no points';
    if (points.length > capacity()) return `more points than the board's ${capacity()}`;
    return null;
  },
  describe: () => {
    const count = passes();
    const what = count === 0 ? 'repeated' : count === 1 ? 'once' : `${count} times`;
    return `this page's waveform (${points.length} points, ${what})`;
  },
  load: () => { load(); showBoard(); },
});

window.addEventListener('resize', () => { resize(); if ($('bode_section').open) drawBode(); });
computePreview();
analyse();
loadLoopTable();
buildTable();
showSummary();
showBoard();
resize();
connect();
