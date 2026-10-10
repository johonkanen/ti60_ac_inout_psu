// The supply's banner, on top of every page: its state (running, idle: the gate driver supplies
// on, off, tripped), Power on / off, Run, Stop, and Clear trip when tripped.
//
// The board does the sequences (scope.c's supply_*): Power on brings the gate driver supplies up
// (one at a time, in the FPGA); Run starts the DHB and the output bridge in closed loop on the
// loaded waveform until Stop (none loaded: 0 V held); Stop stops everything switching (the
// output brought down to 0 V first), the supplies left on; Power off stops, then turns them off.
//
// A page with its own /ws connection hands it over (Banner.useSocket(send), and each status to
// Banner.status), so the banner takes no client of the board's few; a page without one gets the
// banner's own, with the heartbeat the board's watchdog wants.
const Banner = (() => {
  const STATES = {
    running: 'Running',
    idle: 'Idle',
    off: 'Off',
    tripped: 'Tripped',
  };
  let send = null;
  let own = null;
  let last = null;
  let element = null;

  function build() {
    element = document.createElement('div');
    element.id = 'supply_banner';
    element.innerHTML = `
      <span class="supply_state" id="supply_state">…</span>
      <button id="supply_power"></button>
      <button id="supply_run" class="supply_run">Run</button>
      <button id="supply_stop" class="supply_stop">Stop</button>
      <button id="supply_clear" class="supply_clear" hidden>Clear trip</button>
      <span class="supply_detail" id="supply_detail"></span>`;
    document.body.prepend(element);
    element.querySelector('#supply_power').onclick = () => {
      if (!last) return;
      const on = last.supply === 'off';
      if (!on && last.supply === 'running' &&
          !confirm('Power off: stop the running output and the DHB, then turn the gate driver supplies off?')) return;
      send && send({type: 'supply_power', on: on ? 1 : 0});
    };
    element.querySelector('#supply_run').onclick = () => {
      if (!last) return;
      const loaded = last.waveform ? last.waveform[0] : 0;
      const what = loaded ? `the loaded waveform (${loaded} point${loaded === 1 ? '' : 's'})` : '0 V (no waveform loaded)';
      if (!confirm(`Run: the DHB's precharge, then the output in closed loop on ${what} until Stop?`)) return;
      send && send({type: 'supply_run'});
    };
    element.querySelector('#supply_stop').onclick = () => send && send({type: 'supply_stop'});
    element.querySelector('#supply_clear').onclick = () => send && send({type: 'dhb_clear_trip'});
    show();
  }

  function show() {
    if (!element) return;
    const state = element.querySelector('#supply_state');
    const power = element.querySelector('#supply_power');
    const run = element.querySelector('#supply_run');
    const stop = element.querySelector('#supply_stop');
    const clear = element.querySelector('#supply_clear');
    const detail = element.querySelector('#supply_detail');
    if (!last || !send) {
      state.textContent = 'No connection';
      state.className = 'supply_state none';
      power.textContent = 'Power on';
      for (const b of [power, run, stop]) b.disabled = true;
      clear.hidden = true;
      detail.textContent = '';
      return;
    }
    const supply = last.supply || 'off';
    state.textContent = STATES[supply] || supply;
    state.className = `supply_state ${supply}`;
    const supplies = last.gate_power_on || 0;
    const count = [0, 1, 2, 3, 4, 5].filter(n => supplies >> n & 1).length;
    power.textContent = supply === 'off' ? 'Power on' : 'Power off';
    power.disabled = false;
    run.disabled = supply !== 'idle' || count !== 6;
    run.title = supply === 'idle' && count !== 6 ? 'the gate driver supplies are still coming on' : '';
    stop.disabled = false;
    clear.hidden = supply !== 'tripped';
    const parts = [`gate drivers ${count}/6`];
    if (last.dc_links) parts.push(`DC links ${last.dc_links[0].toFixed(1)} V / ${last.dc_links[1].toFixed(1)} V`);
    if (last.waveform) parts.push(last.waveform[0] ? `waveform ${last.waveform[0]} points` : 'no waveform loaded');
    detail.textContent = parts.join(' · ');
  }

  // the banner's own connection, for a page without one
  function connect() {
    own = new WebSocket(`ws://${location.host}/ws`);
    own.onopen = () => {
      send = message => { if (own.readyState === WebSocket.OPEN) own.send(JSON.stringify(message)); };
      show();
    };
    own.onmessage = event => {
      if (typeof event.data !== 'string') return;
      const message = JSON.parse(event.data);
      if (message.type === 'status') status(message);
    };
    own.onclose = () => { last = null; show(); setTimeout(connect, 1000); };
  }

  function status(message) {
    last = message;
    show();
  }

  document.addEventListener('DOMContentLoaded', () => {
    build();
    if (!send) {
      connect();
      setInterval(() => send && send({type: 'heartbeat'}), 1000);
    }
  });

  return {
    useSocket: sender => { send = sender; },
    status,
    disconnected: () => { last = null; show(); },
  };
})();
