// Ti60 oscilloscope web client: controls the capture over the /ws WebSocket and plots the
// captures with uPlot. From fpga/scripts/scope, served here by the Zephyr scope app.
//
// Stream: on the PSU board the app also streams the FPGA's continuous capture on a WebSocket
// of its own (port 5004), every sample at the selected rate and sources, 16 bytes a sample.
// Each message is one lap of the board's ring, 65520 samples of eight int16 channels. The
// latest lap is drawn as a min/max envelope per pixel column; Record saves the next seconds
// raw. The link takes about 62 MB/s, 3.9 MS/s: above that the board ends the stream.
//
// The plots, their units and their layout are plots.js's, as on the data collection page: what
// is shown, measured and exported is in each source's unit (scaling.json on the board).
'use strict';

const DIVIDERS = [0, 1, 4, 9, 24, 31, 49, 124, 249, 624, 1249, 2499, 6249, 12499, 24999, 62499, 124999];
// up to what the board says (hello.max_length); longer than 32768 a capture is downloaded from
// the board's stream port (downloadCapture) and drawn as min and max per pixel column
const LENGTHS = [256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144, 524288, 1000000];
let maxLength = 32768;
const STREAM_PORT = 5004;
const STREAM_RECORD_BYTES = CHANNELS * 2;
const STREAM_MAX_BYTES = 62.5e6;    // what the board carries, measured: 3.9 MS/s

// rates the stream cannot carry are not offered while it runs
function markStreamRates() {
  for (const option of $('rate').options) {
    const bytes = STREAM_RECORD_BYTES * clockHz / (+option.value + 1);
    option.disabled = !!stream && bytes > STREAM_MAX_BYTES;
  }
}

let socket = null;
let config = null;
let sourceNames = [];
let clockHz = 125e6;
let area = null;             // the plots
let built = false;
let boardStatus = {};        // the board's last status message
let pendingCapture = null;   // a capture that came before the page was ready
let capture = null;          // last capture: {header, time, channels: [Float64Array] in counts}

const $ = id => document.getElementById(id);

function formatRate(hz) {
  if (hz >= 1e6) return `${+(hz / 1e6).toFixed(3)} MS/s`;
  if (hz >= 1e3) return `${+(hz / 1e3).toFixed(3)} kS/s`;
  return `${+hz.toFixed(3)} S/s`;
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
  $('rate').innerHTML = DIVIDERS.map(d => `<option value="${d}">${formatRate(clockHz / (d + 1))}</option>`).join('');
  $('length').innerHTML = LENGTHS.filter(l => l <= maxLength)
    .map(l => `<option value="${l}">${l.toLocaleString('en').replace(/,/g, ' ')} samples</option>`).join('');
  $('trigger_channel').innerHTML = Array.from({length: CHANNELS}, (_, i) => `<option value="${i}">CH${i + 1}</option>`).join('');

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
      replot();
    });
    row.querySelector(`#source${i}`).addEventListener('change', () => { sendConfig(); replot(); });
  }
  area.bindControls();
  Signals.viewBar($('views'), {apply: applyView, current: currentView,
    extra: [{label: 'ADC channels', title: "the ADC section's checked signals, a mains view", onclick: () => adcPreset()}]});

  for (const id of ['rate', 'length', 'mode', 'trigger_channel', 'trigger_edge', 'trigger_level', 'generator_hz']) {
    $(id).addEventListener('change', () => sendConfig());
  }
  $('pre_percent').addEventListener('input', () => { $('pre_value').textContent = `${$('pre_percent').value} %`; });
  $('pre_percent').addEventListener('change', () => sendConfig());

  $('run').onclick = () => send({type: 'run'});
  $('single').onclick = () => send({type: 'single'});
  $('stop').onclick = () => send({type: 'stop'});
  $('force').onclick = () => send({type: 'force'});
  $('export').onclick = exportCsv;
  $('stream').onclick = () => (stream ? stopStream() : startStream());
  $('record').onclick = startRecording;
  $('save_scaling').onclick = async () => { $('scaling_status').textContent = await units.save(); };
  buildAdcControls();
}

// ------------------------------------------------------------------ the ADCs
// the control board's two ADCs, their mux positions as sources 'sec 0 vdc_out' or 'pri mux 2'
const ADCS = ['sec', 'pri'];
// the preset's signals, in channel order, and their plot groups: inputs, outputs, currents;
// up to eight of them are checked, this viewer's choice kept in the browser
const ADC_PRESET = [
  {name: 'pri 0 vac_in', group: 0}, {name: 'pri 1 vdc_in', group: 0}, {name: 'pri 3 vfilt_in', group: 0},
  {name: 'sec 0 vdc_out', group: 1}, {name: 'sec 1 vac_out', group: 1}, {name: 'sec 2 vfilt_out', group: 1},
  {name: 'sdm iac_in', group: 2}, {name: 'sdm iac_out', group: 2}, {name: 'sdm i_dab', group: 2},
];
const ADC_PRESET_DEFAULT = ADC_PRESET.slice(0, 8).map(s => s.name);
const ADC_PRESET_KEY = 'scope.adc_preset';
const ADC_PRESET_DIVIDER = 1249;    // 100 kS/s: two mains periods in 4096 samples

function presetSignals() {
  // the signals this board has, checked or not
  return ADC_PRESET.filter(s => sourceNames.includes(s.name));
}

function buildPresetSignals() {
  let chosen = ADC_PRESET_DEFAULT;
  try {
    const saved = JSON.parse(localStorage.getItem(ADC_PRESET_KEY));
    if (Array.isArray(saved)) chosen = saved;
  } catch { /* none kept, or no storage */ }
  const signals = presetSignals();
  $('adc_preset_signals').innerHTML = signals.map((s, i) =>
    `<label><input type="checkbox" id="preset_${i}" ${chosen.includes(s.name) ? 'checked' : ''}> ${s.name}</label>`).join('');
  const limit = () => {
    const boxes = signals.map((_, i) => $(`preset_${i}`));
    const checked = boxes.filter(b => b.checked).length;
    for (const b of boxes) b.disabled = !b.checked && checked >= CHANNELS;
  };
  signals.forEach((s, i) => $(`preset_${i}`).addEventListener('change', () => {
    limit();
    try {
      localStorage.setItem(ADC_PRESET_KEY, JSON.stringify(signals.filter((_, k) => $(`preset_${k}`).checked).map(x => x.name)));
    } catch { /* not kept */ }
  }));
  // more than eight kept from before: the first eight
  let count = 0;
  signals.forEach((_, i) => { const b = $(`preset_${i}`); if (b.checked && ++count > CHANNELS) b.checked = false; });
  limit();
}

function positionName(adc, p) {
  return sourceNames.find(n => n.startsWith(`${adc} ${p} `)) || sourceNames.find(n => n === `${adc} mux ${p}`);
}

function buildAdcControls() {
  const present = ADCS.filter(a => positionName(a, 0));
  const sdm = sourceNames.some(n => n.startsWith('sdm '));
  $('adc_section').hidden = present.length === 0 && !sdm;
  $('adc_masks').innerHTML = present.map(adc => `<div class="adc">
    <span class="adc_name">${adc}</span><span class="adc_rate" id="adc_rate_${adc}"></span>
    <label class="settle">settle <input id="adc_settle_${adc}" type="number" min="0" max="100000" step="5"> ns</label>
    <div class="positions">${Array.from({length: 8}, (_, p) => {
      const name = positionName(adc, p) || `${adc} mux ${p}`;
      const named = !name.startsWith(`${adc} mux`);
      return `<label class="${named ? '' : 'unnamed'}"><input type="checkbox" id="adc_${adc}_${p}">
        ${p} ${named ? name.slice(adc.length + 3) : ''}</label>`;
    }).join('')}</div></div>`).join('') + (sdm ? `<div class="adc">
    <span class="adc_name">sigma-delta</span><span class="adc_rate" id="sdm_rate"></span>
    <label class="settle">decimation <select id="sdm_log2">${[5, 6, 7, 8].map(l =>
      `<option value="${l}">${2 ** l} · ${formatRate(20e6 / 2 ** l)}</option>`).join('')}</select></label>
    <label class="settle">read phase <input id="sdm_phase" type="number" min="0" max="9" step="1"></label>
    </div>` : '');
  if (sdm) {
    $('sdm_log2').addEventListener('change', () => sendConfig());
    $('sdm_phase').addEventListener('change', () => sendConfig());
  }
  // the gate driver supplies, on the boards with the modulators
  $('gate_power_section').hidden = !sdm;
  $('gate_power').innerHTML = Array.from({length: 6}, (_, n) =>
    `<label><input type="checkbox" id="gate_power_${n}"> supply ${n + 1}</label>`).join('');
  for (let n = 0; n < 6; n++) $(`gate_power_${n}`).addEventListener('change', () => sendConfig());
  const setAll = on => {
    for (let n = 0; n < 6; n++) $(`gate_power_${n}`).checked = on;
    sendConfig();
  };
  $('gate_power_all').onclick = () => setAll(true);
  // the DHB precharge test, on the boards with it (its scope probe)
  $('dhb_section').hidden = !sourceNames.includes('dhb test');
  $('dhb_test').onclick = () => {
    if (confirm('Switch the DHB primary leg for 20 ms (200 ns pulses at 150 kHz)?\n' +
                'The gate driver supplies on, the DC link at the test voltage.')) {
      send({type: 'dhb_test', periods: 3000, pulse: 40});
    }
  };
  $('dhb_ramp').onclick = () => {
    if (confirm('Switch the DHB primary leg for 50 ms, its pulses widening from 200 ns to 500 ns ' +
                '(the ramp that reaches nominal in 500 ms)?\nThe gate driver supplies on, the DC link at the test voltage.')) {
      send({type: 'dhb_test', periods: 7500, pulse: 40, ramp: 124});
    }
  };
  $('dhb_precharge').onclick = () => {
    if (confirm('Precharge the secondary DC link: the primary leg on the 500 ms ramp from 200 ns ' +
                'pulses, until the secondary is within 5 V of the primary?\nThe gate driver supplies on.')) {
      send({type: 'dhb_test', periods: 75000, pulse: 40, ramp: 124});
    }
  };
  $('dhb_stop').onclick = () => send({type: 'dhb_stop'});
  // the DHB's voltage control, on the boards with it
  $('dhb_control_section').hidden = !sourceNames.includes('dhb test');
  for (const id of ['dhb_kp', 'dhb_ki', 'dhb_limit_high', 'dhb_limit_low', 'dhb_control',
                    'dhb_overcurrent', 'dhb_overvoltage', 'dhb_overvoltage_readings', 'dhb_filter',
                    'boost_v_ref', 'boost_d_max', 'boost_limit']) {
    $(id).addEventListener('change', () => sendConfig());
  }
  $('dhb_clear_trip').onclick = () => send({type: 'dhb_clear_trip'});
  // the output voltage's control, on the boards with it
  $('control_section').hidden = !sourceNames.includes('control u_ref');
  for (const id of CONTROL_FIELDS) {
    $(id).addEventListener('change', () => sendConfig());
  }
  $('control_run').onclick = () => {
    const ms = Math.max(2, Math.min(1000, +$('control_run_ms').value || 30));
    const amplitude = +$('control_amplitude').value;
    const half = Math.max(1, Math.round(+$('control_cycle_ms').value * PERIODS_A_MS / 2));
    if (confirm(`Output bridge closed loop: +-${amplitude} V square wave, ${$('control_cycle_ms').value} ms a cycle, ` +
                `${ms} ms, after the DHB's precharge (the current limited to ${$('control_i_max').value} A)?`)) {
      send({type: 'output_test', mode: 'control', amplitude, half, periods: Math.round(ms * PERIODS_A_MS)});
      send({type: 'dhb_test', periods: 75000, pulse: 40, ramp: 124, run: Math.min(75000, Math.round((ms + 100) * 150)),
            invert: $('dhb_invert').checked ? 1 : 0});
    }
  };
  $('control_stop').onclick = () => send({type: 'output_stop'});
  // the output observer, on the boards with it
  $('observer_section').hidden = !sourceNames.includes('observer v');
  for (const id of OBSERVER_FIELDS.map(f => f[0])) {
    $(id).addEventListener('change', () => sendConfig());
  }
  // the output bridge's step test, on the boards with it
  $('output_section').hidden = !sourceNames.includes('output test');
  $('output_mode').onchange = () => {
    const voltage = $('output_mode').value === 'voltage';
    $('output_before_label').textContent = $('output_after_label').textContent = voltage ? 'Voltage (V)' : 'Duty';
    for (const [id, value] of [['output_before', voltage ? 0 : 0.5], ['output_after', voltage ? 20 : 0.55]]) {
      Object.assign($(id), voltage ? {step: 1, min: -450, max: 450} : {step: 0.01, min: 0, max: 1});
      $(id).value = value;
    }
  };
  $('output_run').onclick = () => {
    const ms = Math.max(2, Math.min(1000, +$('output_ms').value || 100));
    const stepMs = Math.max(1, Math.min(ms - 1, +$('output_step_ms').value || 50));
    const before = +$('output_before').value, after = +$('output_after').value;
    const mode = $('output_mode').value, unit = mode === 'voltage' ? ' V' : '';
    if (confirm(`Output bridge: ${mode} ${before}${unit} stepping to ${after}${unit} at ${stepMs} ms, ${ms} ms in all, ` +
                `after the DHB's precharge (the secondary${$('dhb_invert').checked ? ' inverted' : ''}, ` +
                `the voltage control ${$('dhb_control').checked ? 'on' : 'OFF'})?`)) {
      send({type: 'output_test', mode, before, after, step: Math.round(stepMs * PERIODS_A_MS),
            periods: Math.round(ms * PERIODS_A_MS)});
      send({type: 'dhb_test', periods: 75000, pulse: 40, ramp: 124, run: Math.min(75000, Math.round((ms + 100) * 150)),
            invert: $('dhb_invert').checked ? 1 : 0});
    }
  };
  $('output_stop').onclick = () => send({type: 'output_stop'});
  $('dhb_run').onclick = () => {
    const ms = Math.max(1, Math.min(500, +$('dhb_run_ms').value || 20));
    if (confirm(`Precharge the secondary DC link, then run the bridge for ${ms} ms, both legs on the ` +
                `voltage control's phase${$('dhb_invert').checked ? ', the secondary inverted' : ''}?\n` +
                'The gate driver supplies on, the trips set.')) {
      send({type: 'dhb_test', periods: 75000, pulse: 40, ramp: 124, run: Math.round(ms * 150),
            invert: $('dhb_invert').checked ? 1 : 0});
    }
  };
  $('gate_power_none').onclick = () => setAll(false);
  for (const input of $('adc_masks').querySelectorAll('input')) {
    input.addEventListener('change', () => {
      // at least one position each
      const adc = input.id.split('_')[1];
      if (adcMask(adc) === 0) input.checked = true;
      sendConfig();
    });
  }
  for (const adc of present) $(`adc_settle_${adc}`).addEventListener('change', () => sendConfig());
  $('adc_preset').onclick = adcPreset;
  buildPresetSignals();
}

function adcMask(adc) {
  let mask = 0;
  for (let p = 0; p < 8; p++) {
    const box = $(`adc_${adc}_${p}`);
    if (box && box.checked) mask |= 1 << p;
  }
  return mask;
}

function showAdcMasks(masks, settle) {
  ADCS.forEach((adc, a) => {
    const input = $(`adc_settle_${adc}`);
    if (input && settle) input.value = settle[a];
  });
  if (!masks) return;
  ADCS.forEach((adc, a) => {
    for (let p = 0; p < 8; p++) {
      const box = $(`adc_${adc}_${p}`);
      if (box) box.checked = (masks[a] >> p & 1) === 1;
    }
  });
}

function showAdcRates(rates) {
  if (!rates || !config || !config.adc_masks) return;
  ADCS.forEach((adc, a) => {
    const element = $(`adc_rate_${adc}`);
    if (!element) return;
    let positions = 0;
    for (let p = 0; p < 8; p++) positions += config.adc_masks[a] >> p & 1;
    element.textContent = rates[a] ? `${formatRate(rates[a])}, ${formatRate(rates[a] / Math.max(1, positions))} each` : '';
  });
}

// the checked signals on CH1.. in order, inputs, outputs and currents each in a plot of their
// own; the ADCs' mux positions those of the checked voltages alone; a mains view
function adcPreset() {
  const chosen = presetSignals().filter((_, i) => $(`preset_${i}`).checked).slice(0, CHANNELS);
  if (chosen.length === 0) return;
  // the groups used, numbered P1.. in order
  const groups = [...new Set(chosen.map(s => s.group))];
  chosen.forEach((s, c) => {
    $(`source${c}`).value = sourceNames.indexOf(s.name);
    area.layout.plotOf[c] = groups.indexOf(s.group) + 1;
  });
  for (let c = 0; c < CHANNELS; c++) {
    area.visible[c] = c < chosen.length;
    $(`show${c}`).checked = area.visible[c];
    area.layout.hidden[c] = false;
  }
  if (area.layout.count !== 'each') area.layout.count = String(groups.length);
  area.save();
  area.showControls();
  // an ADC with none of its voltages chosen keeps its positions
  ADCS.forEach(adc => {
    const used = chosen.filter(s => s.name.startsWith(`${adc} `));
    if (used.length === 0) return;
    for (let p = 0; p < 8; p++) {
      const box = $(`adc_${adc}_${p}`);
      if (box) box.checked = used.some(s => s.name.startsWith(`${adc} ${p} `));
    }
  });
  // the trigger on vac_in at its 0 V, else on CH1 at its zero
  const trigger = Math.max(0, chosen.findIndex(s => s.name === 'pri 0 vac_in'));
  $('rate').value = ADC_PRESET_DIVIDER;
  $('length').value = 4096;
  $('mode').value = 'auto';
  $('trigger_channel').value = trigger;
  $('trigger_edge').value = 0;
  $('trigger_level').value = 0;   // in the channel's unit, so 0 V
  sendConfig();
  replot();
}

// a view (signals.js): its signals on CH1.. in order with their plots, the rest hidden; its
// rate, length, mode and trigger when it has them
function applyView(view) {
  const list = view.channels.filter(([name]) => sourceNames.includes(name)).slice(0, CHANNELS);
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
  if (view.divider != null && DIVIDERS.includes(view.divider)) $('rate').value = view.divider;
  if (view.length && LENGTHS.includes(view.length)) $('length').value = view.length;
  if (view.mode) $('mode').value = view.mode;
  if (view.trigger) {
    const t = list.findIndex(([name]) => name === view.trigger.source);
    if (t >= 0) {
      $('trigger_channel').value = t;
      $('trigger_edge').value = view.trigger.edge || 0;
      $('trigger_level').value = view.trigger.level || 0;
    }
  }
  sendConfig();
  replot();
}

// the page's channels as a view: the shown ones in order, the rate, length, mode and trigger
function currentView() {
  const shown = [];
  for (let c = 0; c < CHANNELS; c++) {
    if (area.visible[c]) shown.push([sourceNames[+$(`source${c}`).value], area.layout.plotOf[c] || 1]);
  }
  const t = +$('trigger_channel').value;
  return {channels: shown, divider: +$('rate').value, length: +$('length').value, mode: $('mode').value,
          trigger: {source: sourceNames[+$(`source${t}`).value], edge: +$('trigger_edge').value,
                    level: +$('trigger_level').value}};
}

function sourceOf(c) {
  return config ? sourceNames[config.sources[c]] || '' : '';
}

// The trigger level is set in the trigger channel's unit, the board compares counts. A source
// whose counts fall as its value rises (a negative scale) has its edges swapped.
function triggerScale() {
  // the source as selected, which may be newer than config
  const select = $(`source${+$('trigger_channel').value}`);
  return units.of(select ? sourceNames[+select.value] || '' : '');
}

function triggerCounts() {
  const s = triggerScale();
  return Math.max(-32768, Math.min(32767, Math.round(s.zero + (+$('trigger_level').value || 0) * s.perUnit)));
}

function swapEdge(edge) {
  return triggerScale().perUnit < 0 && edge < 2 ? 1 - edge : edge;
}

function showTriggerUnit() {
  const s = triggerScale();
  $('trigger_level_unit').textContent = `(${s.unit})`;
  $('trigger_unit').textContent = s.unit === 'counts' ? '' : `= ${triggerCounts()} counts`;
}

function showScaling() {
  units.showTable($('scaling').querySelector('tbody'), Array.from({length: CHANNELS}, (_, c) => sourceOf(c)), () => {
    $('scaling_status').textContent = 'changed, not saved';
    replot();
  });
  showTriggerUnit();
}

// the plots again, for other channels, sources or units
function replot() {
  showScaling();
  area.rebuild();
  plotCapture();
}

function showConfig(c) {
  $('rate').value = c.divider;
  $('length').value = c.length;
  $('mode').value = c.mode === 'single' ? 'auto' : c.mode;
  $('pre_percent').value = c.pre_percent;
  $('pre_value').textContent = `${c.pre_percent} %`;
  $('trigger_channel').value = c.trigger_channel;
  c.sources.forEach((s, i) => { $(`source${i}`).value = s; });
  $('trigger_edge').value = swapEdge(c.trigger_edge);
  $('trigger_level').value = +units.format(sourceNames[c.sources[c.trigger_channel]] || '',
                                           (c.trigger_level - triggerScale().zero) / triggerScale().perUnit);
  $('generator_hz').value = c.generator_hz;
  c.sources.forEach((s, i) => { $(`source${i}`).value = s; });
  showAdcMasks(c.adc_masks, c.adc_settle_ns);
  if ($('sdm_log2') && c.sdm_log2) $('sdm_log2').value = c.sdm_log2;
  if ($('sdm_phase') && c.sdm_phase != null) $('sdm_phase').value = c.sdm_phase;
  for (const id of ['dhb_kp', 'dhb_ki', 'dhb_limit_high', 'dhb_limit_low', 'dhb_overcurrent', 'dhb_overvoltage',
                    'dhb_overvoltage_readings', 'dhb_filter', 'boost_v_ref', 'boost_d_max', 'boost_limit']) {
    if (c[id] != null) $(id).value = +c[id];
  }
  if (c.dhb_control != null) $('dhb_control').checked = !!c.dhb_control;
  for (const id of CONTROL_FIELDS) {
    if (c[id] == null) continue;
    if (id === 'control') $(id).checked = !!c[id]; else $(id).value = +c[id];
  }
  for (const [id, scale] of OBSERVER_FIELDS) {
    if (c[id] == null) continue;
    if (scale) $(id).value = +(c[id] * scale).toPrecision(6); else $(id).checked = !!c[id];
  }
  if (c.gate_power != null) {
    for (let n = 0; n < 6; n++) {
      const box = $(`gate_power_${n}`);
      if (box) box.checked = (c.gate_power >> n & 1) === 1;
    }
  }
  showTriggerUnit();
}

// the output bridge's switching periods a millisecond (output_bridge_pkg.vhd: 25 us, 40 kHz)
const PERIODS_A_MS = 40;

// the control's settings, the board's values as they are (control: a checkbox)
const CONTROL_FIELDS = ['control', 'control_current_hz', 'control_voltage_hz', 'control_b', 'control_voltage_zero',
                        'control_prefilter_hz', 'control_feedforward', 'control_du_hz',
                        'control_i_max', 'control_vl_max', 'control_u_max'];

// the observer's settings: the page's field and its scale from the board's SI value (none:
// a checkbox)
const OBSERVER_FIELDS = [['observer', 0], ['observer_l', 1e3], ['observer_c', 1e6], ['observer_r', 1],
                         ['observer_current_hz', 1], ['observer_voltage_hz', 1]];

function sendConfig() {
  config = {
    divider: +$('rate').value,
    length: +$('length').value,
    mode: $('mode').value,
    pre_percent: +$('pre_percent').value,
    trigger_channel: +$('trigger_channel').value,
    trigger_edge: swapEdge(+$('trigger_edge').value),
    trigger_level: triggerCounts(),
    generator_hz: +$('generator_hz').value,
    sources: Array.from({length: CHANNELS}, (_, i) => +$(`source${i}`).value),
    adc_masks: ADCS.map((adc, a) => adcMask(adc) || (config && config.adc_masks ? config.adc_masks[a] : 255)),
    adc_settle_ns: ADCS.map((adc, a) => {
      const input = $(`adc_settle_${adc}`);
      return input && input.value !== '' ? Math.max(0, Math.round(+input.value)) :
        config && config.adc_settle_ns ? config.adc_settle_ns[a] : 3000;
    }),
    ...($('sdm_log2') ? {sdm_log2: +$('sdm_log2').value, sdm_phase: Math.max(0, Math.round(+$('sdm_phase').value || 0)),
                         gate_power: Array.from({length: 6}, (_, n) => $(`gate_power_${n}`).checked ? 1 << n : 0)
                           .reduce((a, b) => a | b, 0),
                         dhb_kp: +$('dhb_kp').value, dhb_ki: +$('dhb_ki').value,
                         dhb_limit_high: +$('dhb_limit_high').value, dhb_limit_low: +$('dhb_limit_low').value,
                         dhb_control: $('dhb_control').checked ? 1 : 0,
                         dhb_overcurrent: +$('dhb_overcurrent').value,
                         dhb_overvoltage: +$('dhb_overvoltage').value,
                         dhb_overvoltage_readings: +$('dhb_overvoltage_readings').value,
                         dhb_filter: +$('dhb_filter').value,
                         boost_v_ref: +$('boost_v_ref').value, boost_d_max: +$('boost_d_max').value,
                         boost_limit: +$('boost_limit').value} : {}),
    ...($('observer_l') && !$('observer_section').hidden ? Object.fromEntries(OBSERVER_FIELDS.map(([id, scale]) =>
      [id, scale ? +$(id).value / scale : ($(id).checked ? 1 : 0)])) : {}),
    ...($('control_b') && !$('control_section').hidden ? Object.fromEntries(CONTROL_FIELDS.map(id =>
      [id, id === 'control' ? ($(id).checked ? 1 : 0) : +$(id).value])) : {}),
  };
  send({type: 'config', config});
  showTriggerUnit();
  if (stream) {
    // the board takes the rate and sources when the stream starts
    stopStream();
    setTimeout(startStream, 300);
  }
}

function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

// ------------------------------------------------------------------ plot
// the trigger instant on every plot, the trigger level on the trigger channel's
function drawMarkers(u, entry) {
  if (!capture || capture.stream) return;
  const ctx = u.ctx;
  const h = capture.header;
  ctx.save();
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  const x = u.valToPos(0, 'x', true);
  ctx.strokeStyle = '#8a94a3';
  ctx.beginPath(); ctx.moveTo(x, u.bbox.top); ctx.lineTo(x, u.bbox.top + u.bbox.height); ctx.stroke();
  const c = h.trigger_channel;
  if (config && config.mode !== 'free' && entry.channels.includes(c) && !area.layout.hidden[c]) {
    const y = u.valToPos(units.convert(sourceOf(c))(h.trigger_level), area.unitOf(c), true);
    ctx.strokeStyle = COLORS[c];
    ctx.beginPath(); ctx.moveTo(u.bbox.left, y); ctx.lineTo(u.bbox.left + u.bbox.width, y); ctx.stroke();
  }
  ctx.restore();
}

// a plot's channels at the cursor, else at the trigger (the newest of the stream)
function plotValues(entry, idx, hovering) {
  const time = area.data.time;
  if (!hovering) idx = capture && !capture.stream ? Math.min(capture.header.pre, time.length - 1) : time.length - 1;
  let html = `<div class="when">${formatTime(time[idx])} · ${hovering ? 'cursor' : capture && capture.stream ? 'newest' : 'trigger'}</div>`;
  for (const c of entry.channels) {
    html += area.valueBlock(c, area.data.channels[c].line[idx],
                            capture && capture.stream ? 'min or max of a column' : 'sample');
  }
  return html;
}

// a drag across a plot zooms in, a double-click out; the range shown stays for the captures
// that come after: a long capture is reduced again for it, a short one only scaled
let zoom = null;            // [from, to] of the x axis, s, or null for all of the capture
function zoomTo(range) {
  zoom = range;
  plotCapture();
}

// the zoom for the capture, or null when the capture has nothing in it
function zoomFor(from, to) {
  return zoom && zoom[0] < to && zoom[1] > from ? zoom : null;
}

// a long capture's samples in the range shown (zoom, s) to the plots' columns: every
// sample when there are few, else the minimum and maximum of each column at its two ends
function reduceLong() {
  const {header, samples} = capture;
  const n = header.length, rate = header.sample_rate, pre = header.pre;
  let first = 0, end = n;
  const range = zoomFor(-pre / rate, (n - 1 - pre) / rate);
  if (range) {
    first = Math.max(0, Math.floor(range[0] * rate + pre));
    end = Math.min(n, Math.ceil(range[1] * rate + pre) + 1);
    if (end - first < 2) { first = Math.max(0, first - 1); end = Math.min(n, first + 2); }
  }
  const count = end - first;
  const columns = Math.max(100, Math.min(4000, area.width()));
  if (count <= 2 * columns) {
    const time = new Float64Array(count);
    const channels = Array.from({length: CHANNELS}, () => new Float64Array(count));
    for (let k = 0; k < count; k++) {
      const s = first + k;
      time[k] = (s - pre) / rate;
      for (let c = 0; c < CHANNELS; c++) channels[c][k] = samples[s * CHANNELS + c];
    }
    return {time, channels};
  }
  const per = Math.ceil(count / columns);
  const buckets = Math.ceil(count / per);
  const time = new Float64Array(2 * buckets);
  const channels = Array.from({length: CHANNELS}, () => new Float64Array(2 * buckets));
  for (let b = 0; b < buckets; b++) {
    const from = first + b * per, to = Math.min(end, from + per);
    time[2 * b] = (from - pre) / rate;
    time[2 * b + 1] = (to - 1 - pre) / rate;
    for (let c = 0; c < CHANNELS; c++) {
      if (!area.visible[c]) continue;
      let min = Infinity, max = -Infinity;
      for (let i = from * CHANNELS + c; i < to * CHANNELS; i += CHANNELS) {
        const v = samples[i];
        if (v < min) min = v;
        if (v > max) max = v;
      }
      channels[c][2 * b] = min;
      channels[c][2 * b + 1] = max;
    }
  }
  return {time, channels};
}

// the capture in units to the plots
function plotCapture() {
  if (!capture) return;
  if (capture.long) {
    const reduced = reduceLong();
    capture.time = reduced.time;
    capture.channels = reduced.channels;
  }
  const channels = capture.channels.map((ch, c) => {
    const unit = units.convert(sourceOf(c));
    const line = new Float64Array(ch.length);
    for (let i = 0; i < ch.length; i++) line[i] = unit(ch[i]);
    return {line};
  });
  const time = capture.time;
  area.zoom = capture.long || time.length === 0 ? null : zoomFor(time[0], time[time.length - 1]);
  area.setData({time, channels});
  updateMeasurements();
}

// ------------------------------------------------------------------ stream
let stream = null;          // {socket, bytes, started, laps, lastDraw, stopping}
let recording = null;       // {buffers, bytes, until}

function startStream() {
  // the board takes the rate when the stream connects: one it carries, set first
  const bytes = STREAM_RECORD_BYTES * clockHz / ((config ? config.divider : 0) + 1);
  if (bytes > STREAM_MAX_BYTES) {
    const fastest = [...$('rate').options].find(
      o => STREAM_RECORD_BYTES * clockHz / (+o.value + 1) <= STREAM_MAX_BYTES);
    $('rate').value = fastest.value;
    sendConfig();
    setTimeout(startStream, 300);
    return;
  }
  const s = {socket: new WebSocket(`ws://${location.hostname}:${STREAM_PORT}/`), bytes: 0,
             started: performance.now(), laps: 0, lastDraw: 0, latest: null, stopping: false};
  s.socket.binaryType = 'arraybuffer';
  s.socket.onmessage = event => {
    s.bytes += event.data.byteLength;
    s.laps++;
    s.latest = event.data;
    if (recording) record(event.data);
  };
  s.socket.onclose = () => {
    if (stream !== s) return;
    stream = null;
    finishRecording();
    markStreamRates();
    $('stream').classList.remove('active');
    $('record').disabled = true;
    const rate = clockHz / ((config ? config.divider : 0) + 1);
    $('stream_status').textContent = s.stopping ? '' : 'the stream ended…';
    $('stream_status').classList.toggle('error', !s.stopping);
    if (s.stopping) return;
    // the board's status says why (stream_end)
    setTimeout(() => {
      const why = {
        'overrun': `stream ended by the board: ${formatRate(rate)} is ${(STREAM_RECORD_BYTES * rate / 1e6).toFixed(1)} MB/s, ` +
                   `more than the link kept up with (overrun); the stream carries up to about ${STREAM_MAX_BYTES / 1e6} MB/s`,
        'client gone': 'stream ended by the board: this PC acknowledged nothing for 3 s (a stalled browser or network?)',
        'new client': 'the stream was taken over by another connection (another page, tab or program: the board serves one at a time)',
        'client reset': 'the browser reset the stream connection',
        'client closed': 'the browser closed the stream connection',
      }[boardStatus.stream_end];
      $('stream_status').textContent = why ||
        `the stream ended (${boardStatus.stream_end || 'no reason given by the board'})`;
    }, 600);
  };
  stream = s;
  markStreamRates();
  $('stream').classList.add('active');
  $('record').disabled = false;
  $('stream_status').classList.remove('error');
  requestAnimationFrame(drawStream);
}

function stopStream() {
  if (!stream) return;
  stream.stopping = true;
  stream.socket.close();
}

// the latest lap, at most ten times a second
function drawStream(now) {
  const s = stream;
  if (!s) return;
  requestAnimationFrame(drawStream);
  if (now - s.lastDraw < 100) return;
  s.lastDraw = now;
  const seconds = (now - s.started) / 1000;
  const rate = clockHz / ((config ? config.divider : 0) + 1);
  $('stream_status').textContent = `stream · ${formatRate(rate)} · ${(STREAM_RECORD_BYTES * rate / 1e6).toFixed(1)} MB/s ` +
    `asked, ${(s.bytes / seconds / 1e6).toFixed(1)} MB/s received · ${s.laps} laps` +
    (recording ? ` · recording ${(recording.bytes / 1e6).toFixed(0)} MB` : '');
  if (!s.latest) return;
  const samples = new Int16Array(s.latest);
  const n = samples.length / CHANNELS;
  const columns = Math.max(100, Math.min(4000, area.width()));
  const per = Math.ceil(n / columns);
  const buckets = Math.ceil(n / per);
  const time = new Float64Array(2 * buckets);
  const channels = Array.from({length: CHANNELS}, () => new Float64Array(2 * buckets));
  for (let b = 0; b < buckets; b++) {
    const first = b * per, end = Math.min(n, first + per);
    time[2 * b] = first / rate;
    time[2 * b + 1] = (end - 1) / rate;
    for (let c = 0; c < CHANNELS; c++) {
      if (!area.visible[c]) continue;
      let min = Infinity, max = -Infinity;
      for (let i = first * CHANNELS + c; i < end * CHANNELS; i += CHANNELS) {
        const v = samples[i];
        if (v < min) min = v;
        if (v > max) max = v;
      }
      channels[c][2 * b] = min;
      channels[c][2 * b + 1] = max;
    }
  }
  // the measurements over the whole lap
  const full = Array.from({length: CHANNELS}, (_, c) => {
    const d = new Float64Array(n);
    if (area.visible[c]) for (let i = 0; i < n; i++) d[i] = samples[i * CHANNELS + c];
    return d;
  });
  capture = {header: {sources: config ? config.sources : [], length: n, pre: 0, sample_rate: rate,
                      capture: `stream_${s.laps}`}, time, channels, stream: true, full};
  plotCapture();
}

function startRecording() {
  if (!stream || recording) return;
  const seconds = Math.max(1, Math.min(60, +$('record_seconds').value || 5));
  recording = {buffers: [], bytes: 0, until: performance.now() + 1000 * seconds,
               rate: clockHz / ((config ? config.divider : 0) + 1)};
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
  if (r.bytes === 0) return;
  const link = document.createElement('a');
  link.href = URL.createObjectURL(new Blob(r.buffers, {type: 'application/octet-stream'}));
  link.download = `stream_${r.rate}Sps_8ch_int16le_${r.bytes / STREAM_RECORD_BYTES}samples.bin`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 10000);
}

function showCapture(buffer) {
  if (stream) return;           // the stream has the display
  const view = new DataView(buffer);
  const headerLength = view.getUint32(0, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, headerLength)));
  if (header.download) {
    downloadCapture(header);
    return;
  }
  const samples = new Int16Array(buffer.slice(4 + headerLength));
  const n = header.length;
  const time = new Float64Array(n);
  for (let s = 0; s < n; s++) time[s] = (s - header.pre) / header.sample_rate;
  const channels = Array.from({length: CHANNELS}, () => new Float64Array(n));
  for (let s = 0; s < n; s++) {
    for (let c = 0; c < CHANNELS; c++) channels[c][s] = samples[s * CHANNELS + c];
  }
  capture = {header, time, channels};
  // the capture's own sources: another client (a script, another page) may have changed the
  // board's since this page took its config, and the labels and units must be the data's
  if (header.sources && config && header.sources.some((source, c) => source !== config.sources[c])) {
    config.sources = header.sources.slice();
    config.sources.forEach((source, c) => { $(`source${c}`).value = source; });
    replot();
  } else {
    plotCapture();
  }
  showLost(header);
}

// a long capture from the board's stream port: one message of its records (and the ring's
// next ones, up to a whole segment), taken as the capture
let download = null;
function downloadCapture(header) {
  if (download) download.socket.close();
  const d = {socket: new WebSocket(`ws://${location.hostname}:${STREAM_PORT}/capture?id=${header.capture}`),
             started: performance.now(), done: false};
  download = d;
  d.socket.binaryType = 'arraybuffer';
  $('stream_status').textContent = `capture ${header.capture}: downloading ` +
    `${(header.length * STREAM_RECORD_BYTES / 1e6).toFixed(1)} MB…`;
  $('stream_status').classList.remove('error');
  d.socket.onmessage = event => {
    d.done = true;
    d.socket.close();
    if (download === d) download = null;
    if (stream) return;
    const records = Math.min(header.length, event.data.byteLength / STREAM_RECORD_BYTES);
    const samples = new Int16Array(event.data, 0, records * CHANNELS);
    const seconds = (performance.now() - d.started) / 1000;
    capture = {header: {...header, length: records}, samples, long: true};
    if (header.sources && config && header.sources.some((source, c) => source !== config.sources[c])) {
      config.sources = header.sources.slice();
      config.sources.forEach((source, c) => { $(`source${c}`).value = source; });
      replot();
    } else {
      plotCapture();
    }
    showLost(header);
    if (!header.lost) {
      $('stream_status').textContent = `capture ${header.capture}: ${records.toLocaleString('en')} samples, ` +
        `${(event.data.byteLength / 1e6).toFixed(1)} MB in ${seconds.toFixed(2)} s · drag to zoom in, double-click for all`;
    }
  };
  d.socket.onclose = () => {
    if (download === d) download = null;
    if (!d.done) {
      $('stream_status').textContent = `capture ${header.capture}: the download did not come ` +
        `(${boardStatus.stream_end && boardStatus.stream_end !== 'none' ? boardStatus.stream_end : 'closed'}; ` +
        'the stream port serves one client at a time)';
      $('stream_status').classList.add('error');
    }
  };
}

// a capture the memory did not take all of: the deep capture keeps up to about 12.5 MS/s
function showLost(header) {
  const lost = header.lost || 0;
  $('stream_status').textContent = lost ? `capture ${header.capture} is broken: ${lost} samples lost, ` +
    `${formatRate(header.sample_rate)} is faster than the memory takes (up to about 12.5 MS/s)` : '';
  $('stream_status').classList.toggle('error', lost > 0);
}

function updateMeasurements() {
  const body = document.querySelector('#measurements tbody');
  if (!capture) { body.innerHTML = ''; return; }
  const rows = [];
  for (let c = 0; c < CHANNELS; c++) {
    if (!area.visible[c]) continue;
    const name = sourceOf(c), unit = units.convert(name);
    let min = Infinity, max = -Infinity, sum = 0, squares = 0, count = 0;
    const take = counts => {
      const v = unit(counts);
      if (v < min) min = v;
      if (v > max) max = v;
      sum += v;
      squares += v * v;
      count++;
    };
    if (capture.long) {
      // every sample, not the columns drawn
      for (let i = c; i < capture.header.length * CHANNELS; i += CHANNELS) take(capture.samples[i]);
    } else {
      for (const counts of capture.stream ? capture.full[c] : capture.channels[c]) take(counts);
    }
    const d = {length: count};
    const f = v => units.format(name, v);
    rows.push(`<tr><td style="color:${COLORS[c]}">CH${c + 1}</td><td>${name}</td><td>${units.of(name).unit}</td>
      <td>${f(min)}</td><td>${f(max)}</td><td>${f(max - min)}</td><td>${f(sum / d.length)}</td><td>${f(Math.sqrt(squares / d.length))}</td></tr>`);
  }
  body.innerHTML = rows.join('');
}

// every sample of the capture (of the stream's lap, not its envelope), in units
function exportCsv() {
  if (!capture) return;
  const channels = Array.from({length: CHANNELS}, (_, c) => c);
  const convert = channels.map(c => units.convert(sourceOf(c)));
  const lines = ['time_s,' + channels.map(c => `CH${c + 1} ${sourceOf(c)} [${units.of(sourceOf(c)).unit}]`).join(',')];
  if (capture.long) {
    // every sample of a long capture, not the columns drawn
    const {header, samples} = capture;
    for (let s = 0; s < header.length; s++) {
      const t = (s - header.pre) / header.sample_rate;
      lines.push(t.toPrecision(9) + ',' + channels.map(c => units.format(sourceOf(c), convert[c](samples[s * CHANNELS + c]))).join(','));
    }
  } else {
    const data = capture.stream ? capture.full : capture.channels;
    const n = capture.stream ? capture.header.length : capture.time.length;
    for (let s = 0; s < n; s++) {
      const t = capture.stream ? s / capture.header.sample_rate : capture.time[s];
      lines.push(t.toPrecision(9) + ',' + channels.map(c => units.format(sourceOf(c), convert[c](data[c][s]))).join(','));
    }
  }
  const link = document.createElement('a');
  link.href = URL.createObjectURL(new Blob([lines.join('\n')], {type: 'text/csv'}));
  link.download = `capture_${capture.header.capture}.csv`;
  link.click();
  URL.revokeObjectURL(link.href);
}

// ------------------------------------------------------------------ status and connection
function showStatus(s) {
  boardStatus = s;
  showAdcRates(s.adc_rates);
  if ($('sdm_rate')) $('sdm_rate').textContent = s.sdm_rate ? formatRate(s.sdm_rate) : '';
  if (s.dhb_control && s.dc_links && $('dhb_control_status')) {
    const [command, phase, updates] = s.dhb_control;
    const filtered = s.dhb_filtered ? ` (filtered ${s.dhb_filtered[0].toFixed(1)} / ${s.dhb_filtered[1].toFixed(1)} V)` : '';
    $('dhb_control_status').textContent = `DC links ${s.dc_links[0].toFixed(1)} V / ${s.dc_links[1].toFixed(1)} V${filtered} · ` +
      `command ${command.toFixed(4)} · phase ${phase} counts · ${updates} updates`;
  }
  if (s.dhb && $('dhb_status')) {
    const mode = ['off', 'precharging', 'running', 'failed'][s.dhb[0] & 3];
    $('dhb_status').textContent = `${s.dhb[0] & 256 ? 'burst running · ' : ''}${mode} · last burst ` +
      `${s.dhb[1]} periods precharging${s.dhb[0] & 512 ? ', charged' : ''}, ${s.dhb[2] || 0} running`;
  }
  if (s.control && $('control_status')) {
    const [iRef, vlRef, uRef, vPredicted, flags, steps] = s.control;
    const limits = ['current', 'inductor voltage', 'bridge'].filter((_, n) => flags >> n & 1);
    $('control_status').textContent = `i_ref ${iRef.toFixed(2)} A · u_ref ${uRef.toFixed(1)} V · ` +
      `v ${vPredicted.toFixed(1)} V · ${steps} steps` + (limits.length ? ` · limited: ${limits.join(', ')}` : '');
  }
  if (s.observer && $('observer_status')) {
    const [i, v, du, iload, steps] = s.observer;
    $('observer_status').textContent = `i ${i.toFixed(2)} A · v ${v.toFixed(1)} V · du ${du.toFixed(1)} V · ` +
      `iload ${iload.toFixed(2)} A · ${steps} steps`;
  }
  if (s.output && $('output_status')) {
    const [state, periods] = s.output;
    $('output_status').textContent = (state & 1 ? (state & 2 ? 'running, stepped' : 'running') : state & 4 ? 'armed' : 'off') +
      ` · ${periods} periods`;
  }
  if (s.dhb_trip && $('dhb_trip_status')) {
    const [trips, current, voltage] = s.dhb_trip;
    const what = [trips & 1 ? `overcurrent at ${current.toFixed(2)} A` : '', trips & 2 ? `overvoltage at ${voltage.toFixed(1)} V` : '']
      .filter(Boolean).join(', ');
    $('dhb_trip_status').textContent = what ? `TRIPPED: ${what}` : 'no trip';
    $('dhb_trip_status').classList.toggle('error', !!what);
  }
  if (s.gate_power_on != null && $('gate_power_status')) {
    const on = Array.from({length: 6}, (_, n) => n + 1).filter(n => s.gate_power_on >> (n - 1) & 1);
    $('gate_power_status').textContent = on.length ? `on: ${on.join(', ')}` : 'all off';
  }
  const status = $('status');
  const parts = [s.running ? `running · ${s.state}` : 'stopped', `capture ${s.captures}`];
  if (s.forced) parts.push('forced');
  if (s.lost_words) parts.push('the last stream overran');
  if (s.error) parts.push(`error: ${s.error}`);
  // another page's stream (the data collection) has the capture hardware
  const blocked = s.state === 'streaming' && !stream;
  if (blocked) parts.push('no triggered captures: the data collection stream has the capture hardware, stop it there');
  status.textContent = parts.join(' · ');
  status.classList.toggle('error', !!s.error || blocked);
  $('run').classList.toggle('active', s.running);
}

function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);
  socket.binaryType = 'arraybuffer';
  socket.onmessage = async event => {
    if (event.data instanceof ArrayBuffer) {
      // the board's last capture comes right after the hello, maybe before the page is ready
      if (built) showCapture(event.data); else pendingCapture = event.data;
      return;
    }
    const message = JSON.parse(event.data);
    if (message.type === 'hello') {
      sourceNames = message.source_names;
      clockHz = message.clock_hz;
      config = message.config;
      if (message.max_length) maxLength = message.max_length;
      if (!built) {
        await units.load();
        area = new PlotArea({container: $('plots'), storageKey: 'scope.layout', sourceOf,
                             timeFormat: formatTime, values: plotValues, draw: drawMarkers,
                             onSelect: (from, to) => zoomTo([from, to]), onReset: () => zoomTo(null)});
        area.visible.fill(false, 4);
        buildControls();
        built = true;
      }
      showConfig(config);
      replot();
      if (pendingCapture) {
        showCapture(pendingCapture);
        pendingCapture = null;
      }
    } else if (message.type === 'config') {
      // the board's settings after any client changed them (this page, another, a script)
      const sourcesBefore = config && JSON.stringify(config.sources);
      config = message.config;
      if (built) {
        showConfig(config);
        if (JSON.stringify(config.sources) !== sourcesBefore) replot();
      }
    } else if (message.type === 'status') {
      showStatus(message);
      Banner.status(message);
      // the gate driver supplies' enables as the board has them (the banner's Power changes
      // them too): the boxes follow, so this page's next config keeps them
      if (message.gate_power != null && config) {
        config.gate_power = message.gate_power;
        for (let n = 0; n < 6; n++) {
          const box = $(`gate_power_${n}`);
          if (box) box.checked = (message.gate_power >> n & 1) === 1;
        }
      }
    }
  };
  socket.onclose = () => {
    Banner.disconnected();
    $('status').textContent = 'disconnected from the scope, retrying…';
    $('status').classList.add('error');
    setTimeout(connect, 1000);
  };
}

window.addEventListener('resize', () => { if (area) area.resize(); });

// to the data collection: the scope stops first, keeping the capture shown for coming back
document.querySelector('nav a[href="collect.html"]').addEventListener('click', e => {
  if (!boardStatus.running) return;
  e.preventDefault();
  send({type: 'stop'});
  setTimeout(() => { location.href = e.target.href; }, 200);
});
// the board's watchdog: a run (started here or on the waveform page) goes on only while a page
// is there; this one says so every second
setInterval(() => send({type: 'heartbeat'}), 1000);
Banner.useSocket(send);
connect();
