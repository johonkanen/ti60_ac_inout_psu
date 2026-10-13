// The supply's banner, on top of every page: its state (starting, idle: the boost and the DHB
// up, the output off; running: the output on; stopping, stopped: the gate driver supplies on,
// nothing switching; off, tripped), Power on / off, Start, Output on / off, Stop, and Clear trip
// when tripped.
//
// The board does the sequences (scope.c's supply_*, the FPGA's supply_sequencer): Start, from
// off or stopped, brings the gate driver supplies up (one at a time), then the input boost to its
// reference (the scope page's boost settings), then the DHB: idle. Output on, from idle, runs the
// output bridge in closed loop on the loaded waveform (none loaded: 0 V held) until Output off
// (back to idle), Stop or its end; Stop stops everything switching (the output brought down to
// 0 V first), the supplies left on; Power on only brings the supplies up; Power off stops, then
// turns them off.
//
// A page with a waveform of its own (the waveform page) gives it to Output on (Banner.runSource):
// Output on then loads that waveform first and runs it.
//
// A page with its own /ws connection hands it over (Banner.useSocket(send), and each status to
// Banner.status), so the banner takes no client of the board's few; a page without one gets the
// banner's own, with the heartbeat the board's watchdog wants.
const Banner = (() => {
  const STATES = {
    running: 'Running',
    starting: 'Starting',
    idle: 'Idle',
    stopping: 'Stopping',
    stopped: 'Stopped',
    off: 'Off',
    tripped: 'Tripped',
  };
  let send = null;
  let own = null;
  let last = null;
  let element = null;
  let runSource = null;   // {check: () => problem or null, describe: () => text, load: () => {}}
  let notice = null;      // {text, until}: a message shown in the status line for a while

  function note(text) {
    notice = {text, until: Date.now() + 5000};
    show();
  }

  function build() {
    element = document.createElement('div');
    element.id = 'supply_banner';
    element.innerHTML = `
      <span class="supply_state" id="supply_state">…</span>
      <button id="supply_power"></button>
      <button id="supply_run" class="supply_run">Start</button>
      <button id="supply_output" class="supply_run">Output on</button>
      <button id="supply_stop" class="supply_stop">Stop</button>
      <button id="supply_clear" class="supply_clear" hidden>Clear trip</button>
      <span class="supply_detail" id="supply_detail"></span>`;
    document.body.prepend(element);
    element.querySelector('#supply_power').onclick = () => {
      if (!last) return;
      const on = last.supply === 'off';
      if (!on && (last.supply === 'running' || last.supply === 'idle') &&
          !confirm('Power off: stop the output, the DHB and the boost, then turn the gate driver supplies off?')) return;
      send && send({type: 'supply_power', on: on ? 1 : 0});
    };
    // Start at once, no question asked: the supplies, the boost, the DHB, to idle
    element.querySelector('#supply_run').onclick = () => {
      if (!last || !send) return;
      send({type: 'supply_run'});
    };
    // the output on from idle (a page's own waveform loaded first), off when running; what it
    // cannot run shown in the status line
    element.querySelector('#supply_output').onclick = () => {
      if (!last || !send) return;
      if (last.supply === 'running') {
        send({type: 'output_off'});
        return;
      }
      if (runSource) {
        const problem = runSource.check();
        if (problem) { note(`Output on: ${problem}`); return; }
        runSource.load();
      }
      send({type: 'output_on'});
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
    const output = element.querySelector('#supply_output');
    const stop = element.querySelector('#supply_stop');
    const clear = element.querySelector('#supply_clear');
    const detail = element.querySelector('#supply_detail');
    if (!last || !send) {
      state.textContent = 'No connection';
      state.className = 'supply_state none';
      power.textContent = 'Power on';
      for (const b of [power, run, output, stop]) b.disabled = true;
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
    run.disabled = !(supply === 'off' || (supply === 'stopped' && count === 6));
    run.title = supply === 'stopped' && count !== 6 ? 'the gate driver supplies are still coming on' : '';
    output.textContent = supply === 'running' ? 'Output off' : 'Output on';
    output.disabled = !(supply === 'idle' || supply === 'running');
    output.title = supply === 'idle' || supply === 'running' ? '' : 'Start first: the output switches on from idle';
    stop.disabled = false;
    clear.hidden = supply !== 'tripped';
    const parts = [`gate drivers ${count}/6`];
    if (last.dc_links) parts.push(`DC links ${last.dc_links[0].toFixed(1)} V / ${last.dc_links[1].toFixed(1)} V`);
    if (last.waveform) parts.push(last.waveform[0] ? `waveform ${last.waveform[0]} points` : 'no waveform loaded');
    // why the last start ended, when not by a stop or its waveform's end
    const ended = last.sequencer != null ? (last.sequencer >> 12) & 15 : 0;
    const reasons = {1: 'a trip', 2: 'the gate driver supplies not on in time', 3: 'the DHB not running in time',
                     4: 'the output not running in time', 6: 'the boost not at its reference in time',
                     7: 'the boost stopping', 8: 'the DHB stopping'};
    if (reasons[ended] && supply !== 'running' && supply !== 'starting' && supply !== 'idle') {
      parts.push(`the last start ended by ${reasons[ended]}`);
    }
    if (notice && Date.now() < notice.until) parts.unshift(notice.text);
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
    runSource: source => { runSource = source; },
    status,
    disconnected: () => { last = null; show(); },
  };
})();
