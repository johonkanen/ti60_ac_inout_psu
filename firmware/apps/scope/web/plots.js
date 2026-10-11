// The plot area shared by the scope page (scope.js) and the data collection page (collect.js):
// the channels' units, and up to four plots (or one per channel) with the channels put in them
// as chosen, each with its title row (channel names that hide or show a channel on a click, the
// y range of each unit, auto or fixed, and on the collection page the band or the instantaneous
// value), a handle under it that sets its height, and a values panel on its right. The layout is
// this viewer's, kept in the browser; the scaling is the board's (scaling.json), for everyone.
// The channel clicked last is drawn over the others: a click on its curve (the one nearest the
// click), or showing it again by its name.
'use strict';

const CHANNELS = 8;
const COLORS = ['#f5c542', '#4aa3ff', '#ff6b9a', '#5ad67d', '#c08bff', '#ff9f43', '#3dd6d0', '#e0e0e0'];
const MAX_PLOTS = 4;
const VALUES_WIDTH = 180;     // the values panel: fixed, so changing values move nothing

// ------------------------------------------------------------------ units
// the calibration of the power supply design (ac_in_ac_out_lab_power_supply, test_hw.py and
// meas_pkg): value = (counts - zero) / perUnit
const DEFAULT_SCALING = {
  'pri 0 vac_in': {unit: 'V', zero: 8137.909, perUnit: 16.763636},
  'pri 1 vdc_in': {unit: 'V', zero: -15.197, perUnit: 16.857480},
  // the secondary DC link: the same divider and scaling as the primary's
  'sec 0 vdc_out': {unit: 'V', zero: -15.197, perUnit: 16.857480},
  'pri 3 vfilt_in': {unit: 'V', zero: 8153.273, perUnit: -16.327273},
  // the output side's AC voltages: the same dividers and scaling as the input side's. vac_out is
  // at the output contact, behind the output relay: with the relay open it does not follow vfilt_out
  'sec 1 vac_out': {unit: 'V', zero: 8137.909, perUnit: 16.763636},
  'sec 2 vfilt_out': {unit: 'V', zero: 8153.273, perUnit: -16.327273},
  // the currents: ADuM7702 isolated sigma-delta modulators (+-64 mV full scale, +-32768 out
  // of sdm_adc) across 0.5 mOhm shunts (the schematic says 2 mOhm, but the output bridge's
  // ripple and LC ringing measured 4x the current that gives): 32768 * 0.0005 / 0.064 counts
  // per A, +-128 A at full scale. The zeros are the offsets measured on the board with no current,
  // 2026-10-09
  'sdm iac_in': {unit: 'A', zero: -160.2, perUnit: -256},   // sign turned like i_dab's
  'sdm iac_out': {unit: 'A', zero: -331.6, perUnit: -256},  // positive: into the output filter
  'sdm i_dab': {unit: 'A', zero: -197.4, perUnit: -256},   // positive: charging the secondary
  // the same over exactly one switching period (sdm_adc's window, 25 us)
  'sdm iac_in mean': {unit: 'A', zero: -160.2, perUnit: -256},
  'sdm iac_out mean': {unit: 'A', zero: -331.6, perUnit: -256},
  'sdm i_dab mean': {unit: 'A', zero: -197.4, perUnit: -256},
  // that of iac_out taken at the output bridge's period start and middle
  'iac_out sampled': {unit: 'A', zero: -331.6, perUnit: -256},
  // the same at the middles of its rise and fall (sdm_adc's adaptive sampler)
  'iac_out midpoint': {unit: 'A', zero: -331.6, perUnit: -256},
  // the output observer's estimates (output_observer.vhd): 256 counts an ampere, 64 a volt
  'observer i': {unit: 'A', zero: 0, perUnit: 256},
  'observer v': {unit: 'V', zero: 0, perUnit: 64},
  'observer du': {unit: 'V', zero: 0, perUnit: 64},
  'observer iload': {unit: 'A', zero: 0, perUnit: 256},
  // the output voltage's control (output_voltage_control.vhd)
  'control i_ref': {unit: 'A', zero: 0, perUnit: 256},
  'control u_ref': {unit: 'V', zero: 0, perUnit: 64},
};
const COUNTS = {unit: 'counts', zero: 0, perUnit: 1};

// a source's plain name (signals.js, where the page has it), else the board's
function plainName(name) {
  return typeof Signals !== 'undefined' && name ? Signals.info(name).label : name;
}

const units = {
  scaling: {},    // by source name, as saved on the board

  async load() {
    try {
      const r = await fetch('scaling.json', {cache: 'no-store'});
      this.scaling = r.ok ? await r.json() : {};
    } catch {
      this.scaling = {};
    }
  },

  async save() {
    const body = new Blob([JSON.stringify(this.scaling, null, 1)]).stream().pipeThrough(new CompressionStream('gzip'));
    try {
      const r = await fetch('/files/scaling.json.gz', {method: 'POST', body: await new Response(body).arrayBuffer(),
                                                       headers: {'Content-Type': 'application/octet-stream'}});
      const result = await r.json();
      return result.ok ? 'saved on the board' : `not saved: ${result.message || r.status}`;
    } catch (e) {
      return `not saved: ${e.message}`;
    }
  },

  of(name) {
    return this.scaling[name] || DEFAULT_SCALING[name] || COUNTS;
  },

  // counts to the source's unit
  convert(name) {
    const s = this.of(name);
    return v => (v - s.zero) / s.perUnit;
  },

  // about a count's worth of decimals
  digits(name) {
    const s = this.of(name);
    return s.unit === 'counts' ? 1 : Math.max(0, Math.min(6, Math.ceil(Math.log10(Math.abs(s.perUnit))) + 1));
  },

  format(name, v) {
    return Number.isFinite(v) ? v.toFixed(this.digits(name)) : '';
  },

  // the scaling table for these sources; changed() after an edit
  showTable(body, names, changed) {
    names = [...new Set(names)].filter(n => n && n !== '-');
    body.innerHTML = names.map(n => {
      const s = this.of(n);
      return `<tr data-name="${n}"><td>${n}</td>
        <td><input class="unit" value="${s.unit}" size="5"></td>
        <td><input class="zero" type="number" step="any" value="${s.zero}"></td>
        <td><input class="per" type="number" step="any" value="${s.perUnit}"></td></tr>`;
    }).join('');
    for (const row of body.querySelectorAll('tr')) {
      for (const input of row.querySelectorAll('input')) {
        input.addEventListener('change', () => {
          const per = +row.querySelector('.per').value;
          this.scaling[row.dataset.name] = {
            unit: row.querySelector('.unit').value.trim() || 'counts',
            zero: +row.querySelector('.zero').value || 0,
            perUnit: per !== 0 && Number.isFinite(per) ? per : 1,
          };
          changed();
        });
      }
    }
  },
};

// ------------------------------------------------------------------ plots
class PlotArea {
  // options:
  //   container     the element the plots go in
  //   storageKey    where the layout is kept in the browser
  //   sourceOf(c)   the source name of channel c
  //   timeFormat(v) the x axis values
  //   bands         offer the band (min..max with the mean) or the instantaneous value per plot
  //   values(entry, idx, hovering) the values panel's HTML, at data index idx
  //   onSelect(from, to)  a drag across a plot; without it the plots zoom, all of them
  //   draw(u, entry)  drawn over a plot after it (markers)
  constructor(options) {
    this.o = options;
    this.layout = {count: '2', plotOf: null, ranges: {}, heights: {}, modes: {}, hidden: [], order: []};
    this.visible = new Array(CHANNELS).fill(true);
    this.plots = [];
    this.data = null;     // {time, channels: [{line | mean, min, max, sample}]}, in units
    this.zoom = null;     // [from, to] of the x axis when zoomed, else the data's
    try {
      const saved = JSON.parse(localStorage.getItem(options.storageKey));
      if (saved && Array.isArray(saved.plotOf) && saved.plotOf.length === CHANNELS) {
        this.layout = {...this.layout, ...saved};
      }
    } catch { /* none kept, or no storage */ }
    if (!Array.isArray(this.layout.hidden) || this.layout.hidden.length !== CHANNELS) {
      this.layout.hidden = new Array(CHANNELS).fill(false);
    }
    // the drawing order, the last on top: every channel once
    const order = Array.isArray(this.layout.order) ? this.layout.order.filter(c => c >= 0 && c < CHANNELS) : [];
    this.layout.order = [...new Set([...order, ...Array.from({length: CHANNELS}, (_, c) => c)])];
  }

  // channel c drawn over the others
  toFront(c) {
    if (this.layout.order[CHANNELS - 1] === c) return;
    this.layout.order = [...this.layout.order.filter(x => x !== c), c];
    this.save();
    this.rebuild();
  }

  save() {
    try { localStorage.setItem(this.o.storageKey, JSON.stringify(this.layout)); } catch { /* not kept */ }
  }

  unitOf(c) {
    return units.of(this.o.sourceOf(c)).unit;
  }

  // channels of the same unit together, when nothing was chosen yet
  defaultPlots() {
    const seen = [];
    return Array.from({length: CHANNELS}, (_, c) => {
      const unit = this.unitOf(c);
      if (!seen.includes(unit)) seen.push(unit);
      return Math.min(seen.indexOf(unit), MAX_PLOTS - 1) + 1;
    });
  }

  // the plot count select and the per channel P selects, ids plot_count and plot_of<c>
  bindControls() {
    if (!this.layout.plotOf) this.layout.plotOf = this.defaultPlots();
    const count = document.getElementById('plot_count');
    count.addEventListener('change', e => {
      this.layout.count = e.target.value;
      this.save();
      this.showControls();
      this.rebuild();
    });
    for (let c = 0; c < CHANNELS; c++) {
      document.getElementById(`plot_of${c}`).addEventListener('change', e => {
        this.layout.plotOf[c] = +e.target.value;
        this.save();
        this.rebuild();
      });
    }
    this.showControls();
  }

  showControls() {
    document.getElementById('plot_count').value = this.layout.count;
    for (let c = 0; c < CHANNELS; c++) {
      const select = document.getElementById(`plot_of${c}`);
      select.value = Math.min(this.layout.plotOf[c], this.layout.count === 'each' ? MAX_PLOTS : +this.layout.count);
      select.disabled = this.layout.count === 'each';
      for (const option of select.options) {
        option.disabled = this.layout.count !== 'each' && +option.value > +this.layout.count;
      }
    }
  }

  static plotSelect(c) {
    return `<select id="plot_of${c}" class="plot_of" title="the plot CH${c + 1} goes in">` +
      Array.from({length: MAX_PLOTS}, (_, p) => `<option value="${p + 1}">P${p + 1}</option>`).join('') + '</select>';
  }

  width() {
    return (this.o.container.clientWidth || 800) - VALUES_WIDTH - 8;
  }

  rebuild() {
    for (const p of this.plots) p.u.destroy();
    this.plots = [];
    this.o.container.innerHTML = '';
    const channels = Array.from({length: CHANNELS}, (_, c) => c).filter(c => this.visible[c]);
    const room = window.innerHeight - 300;
    if (this.layout.count === 'each') {
      const height = Math.max(110, Math.min(220, room / Math.max(1, channels.length)));
      for (const c of channels) this.plots.push(this.makePlot([c], height, `CH${c + 1}`));
    } else {
      const count = +this.layout.count;
      const groups = Array.from({length: count}, (_, p) => ({
        number: p + 1, channels: channels.filter(c => Math.min(this.layout.plotOf[c], count) === p + 1)}))
        .filter(g => g.channels.length);
      const height = Math.max(150, Math.min(groups.length === 1 ? 600 : 400, room / Math.max(1, groups.length) - 20));
      for (const g of groups) this.plots.push(this.makePlot(g.channels, height, `P${g.number}`));
    }
    if (this.data) this.setData(this.data);
  }

  makePlot(channels, height, key) {
    const layout = this.layout;
    height = layout.heights[key] || height;
    const mode = this.o.bands ? layout.modes[key] || 'band' : 'line';
    const axis = {stroke: '#8a94a3', grid: {stroke: '#2a313b', width: 1}, ticks: {stroke: '#2a313b'}};
    const plotUnits = [...new Set(channels.map(c => this.unitOf(c)))];
    const series = [{label: 'x'}];
    const bands = [];
    // the series in the drawing order, the channel clicked last on top
    const drawn = [...channels].sort((a, b) => layout.order.indexOf(a) - layout.order.indexOf(b));
    for (const c of drawn) {
      const scale = this.unitOf(c);
      const base = series.length;
      const show = !layout.hidden[c];
      const band = mode === 'band' && show;
      series.push({label: `CH${c + 1} min`, stroke: 'transparent', width: 0, scale, show: band, points: {show: false}});
      series.push({label: `CH${c + 1} max`, stroke: 'transparent', width: 0, scale, show: band, points: {show: false}});
      series.push({label: `CH${c + 1}`, stroke: COLORS[c], width: 1, scale, show, points: {show: false}});
      if (mode === 'band') bands.push({series: [base + 1, base], fill: COLORS[c] + '40'});
    }
    const axes = [{...axis, values: (u, ticks) => ticks.map(this.o.timeFormat)}];
    plotUnits.slice(0, 2).forEach((unit, i) => axes.push({
      ...axis, scale: unit, side: i === 0 ? 3 : 1, size: 70,
      label: channels.length === 1 ? `CH${channels[0] + 1} ${this.o.sourceOf(channels[0])} [${unit}]` : unit,
      labelSize: 18, grid: i === 0 ? axis.grid : {show: false},
    }));
    const scales = {x: {time: false}};
    for (const unit of plotUnits) {
      // a fixed range as set, else the data's with a little room
      scales[unit] = {range: (u, min, max) => {
        const r = layout.ranges[`${key} ${unit}`];
        if (r && r.fixed && r.max > r.min) return [r.min, r.max];
        if (!(max > min)) return [min - 1, max + 1];
        const pad = (max - min) * 0.05;
        return [min - pad, max + pad];
      }};
    }

    // the title row: names that hide or show, the show mode, the y ranges
    const heading = document.createElement('div');
    heading.className = 'plot_title';
    heading.innerHTML = channels.map(c =>
      `<span data-channel="${c}" class="${layout.hidden[c] ? 'off' : ''}" style="color:${COLORS[c]}" ` +
      `title="${this.o.sourceOf(c)}: click to hide or show">CH${c + 1} ${plainName(this.o.sourceOf(c))} [${this.unitOf(c)}]</span>`).join('') +
      (this.o.bands ? `<label class="range">show <select class="show_mode">
        <option value="band">avg + min/max</option>
        <option value="sample"${mode === 'sample' ? ' selected' : ''}>instantaneous</option></select></label>` : '') +
      plotUnits.slice(0, 2).map(unit => {
        const r = layout.ranges[`${key} ${unit}`] || {};
        return `<label class="range" data-unit="${unit}">${unit}
          <select class="mode"><option value="auto">auto</option><option value="fixed"${r.fixed ? ' selected' : ''}>fixed</option></select>
          <input class="min" type="number" step="any" value="${r.min ?? ''}" ${r.fixed ? '' : 'disabled'} title="bottom">
          <input class="max" type="number" step="any" value="${r.max ?? ''}" ${r.fixed ? '' : 'disabled'} title="top"></label>`;
      }).join('');
    this.o.container.appendChild(heading);
    const row = document.createElement('div');
    row.className = 'plot_row';
    const div = document.createElement('div');
    const side = document.createElement('div');
    side.className = 'values';
    side.style.width = `${VALUES_WIDTH}px`;
    row.appendChild(div);
    row.appendChild(side);
    this.o.container.appendChild(row);

    const entry = {u: null, channels, drawn, side, hover: null, mode, key};
    const area = this;
    const u = new uPlot({
      width: this.width(), height, scales, axes, series, bands,
      legend: {show: false},
      cursor: {drag: {x: true, y: false, setScale: false}, sync: {key: this.o.storageKey}},
      hooks: {
        // a drag: the page's own, or every plot zooms in
        setSelect: [u => {
          if (u.select.width > 4) {
            const from = u.posToVal(u.select.left, 'x'), to = u.posToVal(u.select.left + u.select.width, 'x');
            if (area.o.onSelect) area.o.onSelect(from, to); else area.setZoom([from, to]);
          }
          u.setSelect({left: 0, top: 0, width: 0, height: 0}, false);
        }],
        setCursor: [u => {
          entry.hover = u.cursor.idx ?? null;
          area.showValues(entry);
        }],
        draw: this.o.draw ? [u => this.o.draw(u, entry)] : [],
      },
    }, null, div);
    entry.u = u;
    // a double-click zooms out, when the plots zoom themselves
    div.addEventListener('dblclick', () => { if (!this.o.onSelect) this.setZoom(null); });
    // a click (not a drag) on a curve: that channel on top
    let down = null;
    div.addEventListener('mousedown', e => { down = [e.clientX, e.clientY]; });
    div.addEventListener('click', e => {
      if (!down || Math.abs(e.clientX - down[0]) > 3 || Math.abs(e.clientY - down[1]) > 3) return;
      const c = this.nearestChannel(entry, u.cursor.left, u.cursor.top);
      if (c != null) this.toFront(c);
    });

    // the y range of each unit: auto, or fixed at min..max (from what is shown when switched)
    for (const label of heading.querySelectorAll('label.range[data-unit]')) {
      const unit = label.dataset.unit;
      const modeSelect = label.querySelector('.mode'), min = label.querySelector('.min'), max = label.querySelector('.max');
      const apply = () => {
        const r = {fixed: modeSelect.value === 'fixed', min: parseFloat(min.value), max: parseFloat(max.value)};
        if (r.fixed && !(r.max > r.min)) {
          r.min = u.scales[unit].min;
          r.max = u.scales[unit].max;
          min.value = +r.min.toPrecision(6);
          max.value = +r.max.toPrecision(6);
        }
        min.disabled = max.disabled = !r.fixed;
        layout.ranges[`${key} ${unit}`] = r;
        this.save();
        u.redraw(true, true);
      };
      modeSelect.addEventListener('change', () => {
        if (modeSelect.value === 'fixed') {
          min.value = +u.scales[unit].min.toPrecision(6);
          max.value = +u.scales[unit].max.toPrecision(6);
        }
        apply();
      });
      min.addEventListener('change', apply);
      max.addEventListener('change', apply);
    }
    const show = heading.querySelector('.show_mode');
    if (show) {
      show.addEventListener('change', e => {
        layout.modes[key] = e.target.value;
        this.save();
        this.rebuild();
      });
    }
    for (const span of heading.querySelectorAll('span[data-channel]')) {
      span.addEventListener('click', () => {
        const c = +span.dataset.channel;
        const base = 1 + 3 * drawn.indexOf(c);
        layout.hidden[c] = !layout.hidden[c];
        this.save();
        span.classList.toggle('off', layout.hidden[c]);
        if (!layout.hidden[c] && layout.order[CHANNELS - 1] !== c) {
          this.toFront(c);      // shown again: on top (the plots are made again)
          return;
        }
        for (let k = 0; k < 3; k++) u.setSeries(base + k, {show: !layout.hidden[c] && (k === 2 || mode === 'band')});
        this.showValues(entry);
      });
    }
    this.addResizer(u, key);
    return entry;
  }

  // a handle under the plot: drag it to set the plot's height, double-click for the automatic one
  addResizer(u, key) {
    const handle = document.createElement('div');
    handle.className = 'resize';
    handle.title = 'drag to resize, double-click for the automatic height';
    this.o.container.appendChild(handle);
    handle.addEventListener('pointerdown', down => {
      const start = u.height, y0 = down.clientY;
      handle.setPointerCapture(down.pointerId);
      const move = e => u.setSize({width: u.width, height: Math.max(80, Math.min(2000, start + e.clientY - y0))});
      const up = () => {
        handle.removeEventListener('pointermove', move);
        handle.removeEventListener('pointerup', up);
        this.layout.heights[key] = u.height;
        this.save();
      };
      handle.addEventListener('pointermove', move);
      handle.addEventListener('pointerup', up);
    });
    handle.addEventListener('dblclick', () => {
      delete this.layout.heights[key];
      this.save();
      this.rebuild();
    });
  }

  // data in units: {time, channels[c]: {line} or {mean, min, max, sample}}
  setData(data) {
    this.data = data;
    for (const p of this.plots) {
      const plotData = [data.time];
      for (const c of p.drawn) {
        const ch = data.channels[c];
        const line = ch.line || (p.mode === 'sample' ? ch.sample : ch.mean);
        plotData.push(ch.min || line, ch.max || line, line);
      }
      p.u.setData(plotData, !this.zoom);
      if (this.zoom) p.u.setScale('x', {min: this.zoom[0], max: this.zoom[1]});
      if (p.hover == null) this.showValues(p);
    }
  }

  // the shown channel of a plot whose line is nearest (within 20 pixels) to a point in it
  nearestChannel(entry, left, top) {
    const u = entry.u;
    if (left == null || left < 0 || u.cursor.idx == null) return null;
    let best = null, bestDistance = 20;
    entry.drawn.forEach((c, k) => {
      if (this.layout.hidden[c]) return;
      const v = u.data[1 + 3 * k + 2][u.cursor.idx];
      if (v == null || !Number.isFinite(v)) return;
      const d = Math.abs(u.valToPos(v, u.series[1 + 3 * k + 2].scale) - top);
      if (d < bestDistance) { bestDistance = d; best = c; }
    });
    return best;
  }

  // zoom every plot to [from, to] of the x axis, or back to the data's with null
  setZoom(range) {
    this.zoom = range;
    if (this.data) this.setData(this.data);
  }

  showValues(entry) {
    if (!entry.u || !this.data || this.data.time.length === 0) { entry.side.innerHTML = ''; return; }
    const hovering = entry.hover != null && entry.hover < this.data.time.length;
    entry.side.innerHTML = this.o.values(entry, hovering ? entry.hover : null, hovering);
  }

  // a channel's values block of a values panel
  valueBlock(c, value, detail) {
    return `<div class="channel_value${this.layout.hidden[c] ? ' off' : ''}" style="color:${COLORS[c]}">
      <div class="name">CH${c + 1} ${this.o.sourceOf(c)}</div>
      <div class="v">${units.format(this.o.sourceOf(c), value)} ${this.unitOf(c)}</div>
      <div class="range">${detail}</div></div>`;
  }

  resize() {
    for (const p of this.plots) p.u.setSize({width: this.width(), height: p.u.height});
  }
}
