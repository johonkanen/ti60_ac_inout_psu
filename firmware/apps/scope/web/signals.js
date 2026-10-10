// The signals the scope's channels take: each source's group, a plain name and its unit, for
// grouped channel lists; and views, a click setting every channel (with its plot) and, on the
// scope page, the rate and trigger. Shared by the scope and data collection pages; plots.js's
// units give the units.
//
// A view: {name, channels: [[source name, plot], ...] (CH1.. in order, the rest hidden),
//          divider, length, mode, trigger: {source, edge, level}} (all but channels optional).
// The built-in views below; this browser's own saved in localStorage (signals.views).
const Signals = (() => {
  const GROUPS = ['Input', 'DHB', 'Output', 'Control', 'Digital and status', 'Test signals', 'Other'];

  // in each group's order
  const CATALOGUE = [
    ['pri 0 vac_in', 'Input', 'Input voltage'],
    ['pri 3 vfilt_in', 'Input', 'Input filter voltage'],
    ['sdm iac_in', 'Input', 'Input current'],
    ['sdm iac_in mean', 'Input', 'Input current, period mean'],
    ['input test', 'Input', 'Input boost lanes'],
    ['pri 1 vdc_in', 'DHB', 'Primary DC link'],
    ['sec 0 vdc_out', 'DHB', 'Secondary DC link'],
    ['sdm i_dab', 'DHB', 'DHB current'],
    ['sdm i_dab mean', 'DHB', 'DHB current, period mean'],
    ['dhb test', 'DHB', 'DHB lanes'],
    ['sec 2 vfilt_out', 'Output', 'Output voltage (filter capacitor)'],
    ['sec 1 vac_out', 'Output', 'Output contact voltage'],
    ['sdm iac_out', 'Output', 'Output current'],
    ['sdm iac_out mean', 'Output', 'Output current, period mean'],
    ['iac_out sampled', 'Output', 'Output current, at the pulses'],
    ['iac_out midpoint', 'Output', 'Output current, pulse midpoints'],
    ['output test', 'Output', 'Output bridge lanes'],
    ['observer v', 'Control', 'Observer: output voltage'],
    ['observer i', 'Control', 'Observer: inductor current'],
    ['observer iload', 'Control', 'Observer: load current'],
    ['observer du', 'Control', 'Observer: bridge voltage error'],
    ['control i_ref', 'Control', 'Control: current reference'],
    ['control u_ref', 'Control', 'Control: bridge voltage reference'],
    ['status', 'Digital and status', 'Board status lanes'],
    ['counter 15:0', 'Test signals', 'Clock counter, low half'],
    ['counter 31:16', 'Test signals', 'Clock counter, high half'],
    ['sine', 'Test signals', 'Test: sine'],
    ['cosine', 'Test signals', 'Test: cosine'],
    ['triangle', 'Test signals', 'Test: triangle'],
    ['square', 'Test signals', 'Test: square'],
    ['sawtooth', 'Test signals', 'Test: sawtooth'],
    ['noise', 'Test signals', 'Test: noise'],
    ['sine + noise', 'Test signals', 'Test: sine and noise'],
    ['counter', 'Test signals', 'Test: counter'],
  ];
  const INFO = Object.fromEntries(CATALOGUE.map(([name, group, label], order) => [name, {group, label, order}]));

  function info(name) {
    return INFO[name] || {group: 'Other', label: name, order: 1000};
  }

  // the name shown: the plain one, its unit, the board's own name after it
  function label(name) {
    const i = info(name);
    const unit = typeof units !== 'undefined' ? units.of(name).unit : '';
    const shown = unit && unit !== 'counts' ? `${i.label} (${unit})` : i.label;
    return shown === name ? name : `${shown} · ${name}`;
  }

  // a channel's list: the board's sources by group, in the catalogue's order (values the
  // source numbers)
  function optionsHtml(sourceNames) {
    const escape = s => s.replace(/&/g, '&amp;').replace(/</g, '&lt;');
    const byGroup = new Map(GROUPS.map(g => [g, []]));
    sourceNames.forEach((name, s) => { if (name && name !== '-') byGroup.get(info(name).group).push(s); });
    let html = '';
    for (const [group, list] of byGroup) {
      if (!list.length) continue;
      list.sort((a, b) => info(sourceNames[a]).order - info(sourceNames[b]).order);
      html += `<optgroup label="${group}">` +
        list.map(s => `<option value="${s}">${escape(label(sourceNames[s]))}</option>`).join('') + '</optgroup>';
    }
    return html;
  }

  const BUILT_IN = [
    {name: 'Output control', builtIn: true,
     channels: [['sec 2 vfilt_out', 1], ['observer v', 1], ['sdm iac_out mean', 2], ['observer i', 2],
                ['control i_ref', 2], ['control u_ref', 3], ['output test', 4], ['dhb test', 4]],
     divider: 1249, length: 4096, mode: 'auto', trigger: {source: 'output test', edge: 0, level: 1}},
    {name: 'DHB', builtIn: true,
     channels: [['pri 1 vdc_in', 1], ['sec 0 vdc_out', 1], ['sdm i_dab', 2], ['sdm i_dab mean', 2],
                ['dhb test', 3]],
     divider: 124, length: 4096, mode: 'auto', trigger: {source: 'dhb test', edge: 0, level: 1}},
    {name: 'Input / boost', builtIn: true,
     channels: [['pri 0 vac_in', 1], ['pri 3 vfilt_in', 1], ['pri 1 vdc_in', 1], ['sdm iac_in', 2],
                ['sdm iac_in mean', 2], ['input test', 3]],
     divider: 1249, length: 4096, mode: 'auto', trigger: {source: 'input test', edge: 0, level: 1}},
  ];
  const KEY = 'signals.views';

  function saved() {
    try {
      const list = JSON.parse(localStorage.getItem(KEY) || '[]');
      return Array.isArray(list) ? list.filter(v => v && v.name && Array.isArray(v.channels)) : [];
    } catch {
      return [];
    }
  }

  function store(list) {
    try { localStorage.setItem(KEY, JSON.stringify(list)); } catch { /* not kept */ }
  }

  function views() {
    return [...BUILT_IN, ...saved()];
  }

  // a bar of view buttons in container: apply(view) sets the page; current() gives the page's
  // channels as a view to save; extra: more buttons first ([{label, title, onclick}])
  function viewBar(container, {apply, current, extra = []}) {
    function build() {
      container.innerHTML = '';
      const make = (text, title, onclick, cls = 'small') => {
        const b = document.createElement('button');
        b.className = cls;
        b.textContent = text;
        b.title = title;
        b.onclick = onclick;
        container.appendChild(b);
        return b;
      };
      for (const e of extra) make(e.label, e.title || '', e.onclick);
      for (const view of views()) {
        const names = view.channels.map(([n]) => info(n).label).join(', ');
        make(view.name, `${view.builtIn ? '' : 'saved here: '}${names}`, () => apply(view));
        if (!view.builtIn) {
          make('×', `forget the view ${view.name}`, () => {
            if (!confirm(`Forget the view "${view.name}"?`)) return;
            store(saved().filter(v => v.name !== view.name));
            build();
          }, 'small forget');
        }
      }
      make('Save as view…', 'keep the channels as they are now (and the rate and trigger) as a view, in this browser', () => {
        const name = (prompt('A name for the view:') || '').trim();
        if (!name) return;
        if (BUILT_IN.some(v => v.name === name)) { alert('A built-in view has that name.'); return; }
        const list = saved().filter(v => v.name !== name);
        list.push({...current(), name});
        store(list);
        build();
      });
    }
    build();
    return {rebuild: build};
  }

  return {info, label, optionsHtml, views, viewBar};
})();
