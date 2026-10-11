// The data collection's stream, off the page's thread: the WebSocket to the board's stream
// (port 5004) and every sample made into 1 ms points (each channel's minimum, maximum, mean
// and first sample) here, so the page's drawing, its garbage collection or a busy tab cannot
// hold up the reading: the board's ring is short (168 ms at 3.9 MS/s), the browser stops
// reading the socket while messages wait for a busy thread.
//
// From the page: {type: 'start', url, rate}, {type: 'stop'}, {type: 'raw', on} (the samples
// as they come too, for a raw recording).
// To the page: {type: 'open'}; {type: 'points', count, values, bytes} after each message:
// count points, values CHANNELS * 4 floats a point (min, max, mean, first for each channel,
// channel-major within the point), bytes received so far; {type: 'raw', buffer} (transferred);
// {type: 'closed'} when the stream ends, by stop or by the board.
'use strict';

const CHANNELS = 8;
let socket = null;
let raw = false;
let s = null;

function take(data) {
  const n = data.length / CHANNELS;
  const out = [];
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
      for (let c = 0; c < CHANNELS; c++) {
        out.push(s.min[c], s.max[c], s.sum[c] / s.n, s.first[c]);
        s.min[c] = Infinity; s.max[c] = -Infinity; s.sum[c] = 0;
      }
      s.n = 0;
      s.count++;
      s.next = Math.floor((s.count + 1) * s.perMs);
    }
  }
  return new Float32Array(out);
}

onmessage = event => {
  const m = event.data;
  if (m.type === 'start' && !socket) {
    s = {perMs: m.rate / 1000, samples: 0, next: Math.floor(m.rate / 1000), count: 0, n: 0, bytes: 0,
         min: new Float64Array(CHANNELS).fill(Infinity), max: new Float64Array(CHANNELS).fill(-Infinity),
         sum: new Float64Array(CHANNELS), first: new Float64Array(CHANNELS)};
    socket = new WebSocket(m.url);
    socket.binaryType = 'arraybuffer';
    socket.onopen = () => postMessage({type: 'open'});
    socket.onmessage = e => {
      s.bytes += e.data.byteLength;
      const values = take(new Int16Array(e.data));
      postMessage({type: 'points', count: values.length / (CHANNELS * 4), values, bytes: s.bytes}, [values.buffer]);
      if (raw) postMessage({type: 'raw', buffer: e.data}, [e.data]);
    };
    socket.onclose = () => { socket = null; postMessage({type: 'closed'}); };
  } else if (m.type === 'stop' && socket) {
    socket.close();
  } else if (m.type === 'raw') {
    raw = m.on;
  }
};
