// Ti60 update page: firmware and the web pages over the network, so the board needs no JTAG
// (which cannot be used safely while the power supply runs).
//   GET /update            the slots, JSON
//   POST /update           zephyr.bin or loader.bin; an image makes the board reboot into it
//   GET /files/            the web pages in the board's flash
//   POST /files/<name>.gz  a page, gzipped; DELETE removes one
'use strict';

const $ = id => document.getElementById(id);
const LOADER_MAGIC = 0x444c325a;    // "Z2LD", the loader's second word (sapphire_loader.h)
let image = null;                   // {name, data, crc, kind}

const CRC_TABLE = (() => {
  const table = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    table[n] = c >>> 0;
  }
  return table;
})();

// the CRC-32 the board shows for a slot (zlib's)
function crc32(bytes) {
  let c = 0xffffffff;
  for (let i = 0; i < bytes.length; i++) c = CRC_TABLE[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
  return ((c ^ 0xffffffff) >>> 0).toString(16).padStart(8, '0');
}

function setStatus(id, text, error = false) {
  $(id).textContent = text;
  $(id).classList.toggle('error', error);
}

// ------------------------------------------------------------------ firmware
function describeFpga(f) {
  if (!f || f.role === 'none') return '';
  const i = f.image;
  const image = i ? `application image ${i.sequence} (${i.size} bytes, CRC-32 ${i.crc32}, ` +
    `${i.confirmed ? 'working' : i.failed ? 'failed to load' : i.tried ? 'tried, not yet working' : 'not tried'})`
    : 'no application image';
  if (f.role === 'application') return `FPGA: running the ${image}`;
  return `FPGA: running the golden image` +
    (f.fell_back ? ' after an application image failed to load (a new one loads after a power cycle)' : '') +
    `; ${image}`;
}

// after an FPGA upload the FPGA reloads, the board comes back with the new image or the golden
async function waitForFpga(result) {
  setStatus('update_status', 'written and checked; the FPGA is reloading from it…');
  const started = performance.now();
  await sleep(2000);
  let reply = null;
  while (performance.now() - started < 120000) {   // a bad image: six tries, up to 41 s
    try {
      const controller = new AbortController();
      setTimeout(() => controller.abort(), 2000);
      const r = await fetch('/update', {cache: 'no-store', signal: controller.signal});
      if (r.ok) { reply = await r.json(); break; }
    } catch { /* not back yet */ }
    await sleep(1000);
  }
  if (!reply) throw new Error('the board did not answer within 2 minutes');
  await showSlots();
  const f = reply.fpga || {};
  const seconds = ((performance.now() - started) / 1000).toFixed(0);
  if (f.role === 'application' && f.image && f.image.crc32 === result.crc32) {
    setStatus('update_status', `done: the FPGA runs the new application image, back after ${seconds} s; ` +
              'it counts as working after 10 s');
    setTimeout(showSlots, 11000);
  } else {
    setStatus('update_status', `the FPGA is back on the golden image: the new image did not load ` +
              `(${f.fell_back ? 'it failed its check' : 'see the state below'})`, true);
  }
}

async function showSlots() {
  const reply = await (await fetch('/update', {cache: 'no-store'})).json();
  $('fpga').textContent = describeFpga(reply.fpga);
  $('slots').querySelector('tbody').innerHTML = reply.slots.map((s, i) => {
    const state = i === reply.running ? 'running' : !s.valid ? 'empty' : s.intact ? 'standby' : 'damaged';
    return `<tr${i === reply.running ? ' class="current"' : ''}><td>${i}</td><td>${state}</td>
      <td>${s.valid ? s.sequence : ''}</td><td>${s.valid ? s.size : ''}</td><td>${s.valid ? s.crc32 : ''}</td></tr>`;
  }).join('');
  return reply;
}

// what the board will make of it, the same checks it does (fw_update.c)
function kindOf(data) {
  if (data.length >= 64 && new TextDecoder().decode(data.subarray(0, 8)) === 'TI60FPGA') {
    return new DataView(data.buffer).getUint32(12, true) === 1 ? 'an FPGA application image' : null;
  }
  if (new TextDecoder().decode(data.subarray(0, 9)) === 'Version: ') {
    return null;    // a bare bitstream: application_update.bin is the one
  }
  if (data.length >= 4 && data[0] === 0x7f && data[1] === 0x45 && data[2] === 0x4c && data[3] === 0x46) {
    return null;    // an ELF file: zephyr.bin is the one
  }
  const view = new DataView(data.buffer);
  if (data.length >= 8 && (data[0] & 0x7f) === 0x6f && view.getUint32(4, true) === LOADER_MAGIC) {
    return 'the loader';
  }
  // Zephyr's RISC-V reset vector starts with the auipc of a call to __initialize
  if (data.length >= 8 && (data[0] & 0x7f) === 0x17) return 'a Zephyr image';
  return null;
}

$('image').addEventListener('change', async () => {
  const file = $('image').files[0];
  image = null;
  $('install').disabled = true;
  setStatus('update_status', '');
  if (!file) { $('image_info').textContent = ''; return; }
  const data = new Uint8Array(await file.arrayBuffer());
  const kind = kindOf(data);
  image = {name: file.name, data, crc: crc32(data), kind};
  if (kind === 'an FPGA application image') {
    // what the board reports is the CRC-32 of the image, after our header
    image.crc = new DataView(data.buffer).getUint32(24, true).toString(16).padStart(8, '0');
  }
  $('image_info').textContent = `${file.name}: ${data.length} bytes, CRC-32 ${image.crc}, ` +
    (kind ? `${kind}` : 'not an image the board takes (an FPGA image is application_update.bin)');
  $('install').disabled = !kind;
});

function upload(url, body, progress) {
  return new Promise((resolve, reject) => {
    const request = new XMLHttpRequest();
    request.open('POST', url);
    request.setRequestHeader('Content-Type', 'application/octet-stream');
    request.upload.onprogress = e => { if (e.lengthComputable) progress(e.loaded / e.total); };
    request.onload = () => resolve({status: request.status, text: request.responseText});
    request.onerror = () => reject(new Error('the connection failed'));
    request.ontimeout = () => reject(new Error('no answer'));
    request.timeout = 120000;
    request.send(body);
  });
}

const sleep = ms => new Promise(r => setTimeout(r, ms));

$('install').onclick = async () => {
  if (!image) return;
  const loader = image.kind === 'the loader';
  if (loader && !confirm('Replace the loader? For a few tens of milliseconds while it is ' +
                         'copied, a power cut would leave the board unbootable.')) return;
  $('install').disabled = true;
  $('image').disabled = true;
  $('progress').hidden = false;
  $('progress').value = 0;
  try {
    setStatus('update_status', `sending ${image.name}…`);
    const reply = await upload('/update', image.data, f => {
      $('progress').value = f;
      if (f >= 1) setStatus('update_status', 'writing and checking it in the flash…');
    });
    let result;
    try { result = JSON.parse(reply.text); } catch { result = {ok: false, message: reply.text}; }
    if (!result.ok) throw new Error(result.message || `HTTP ${reply.status}`);
    if (result.crc32 !== image.crc) throw new Error(`the board wrote CRC-32 ${result.crc32}, the file is ${image.crc}`);
    if (result.message === 'reconfiguring') {
      await waitForFpga(result);
      return;
    }
    if (result.message === 'power cycle') {
      setStatus('update_status', 'installed and checked. The FPGA fell back to the golden image ' +
                'before and loads no other image until it is powered off: power-cycle the board to load it.');
      await showSlots();
      return;
    }
    if (result.message !== 'rebooting') {
      setStatus('update_status', `installed ${image.kind}: ${result.message}`);
      await showSlots();
      return;
    }

    // the board reboots into it; its web server answers again some seconds later
    setStatus('update_status', `written to slot ${result.slot}; the board is rebooting into it…`);
    const started = performance.now();
    await sleep(2000);
    let slots = null;
    while (performance.now() - started < 90000) {
      try {
        const controller = new AbortController();
        setTimeout(() => controller.abort(), 2000);
        const r = await fetch('/update', {cache: 'no-store', signal: controller.signal});
        if (r.ok) { slots = await r.json(); break; }
      } catch { /* not back yet */ }
      await sleep(1000);
    }
    if (!slots) throw new Error('the board did not answer within 90 s after the reboot');
    await showSlots();
    const running = slots.slots[slots.running];
    const seconds = ((performance.now() - started) / 1000).toFixed(0);
    if (running && running.crc32 === image.crc) {
      setStatus('update_status', `done: the board runs ${image.name} (slot ${slots.running}), back after ${seconds} s`);
    } else {
      setStatus('update_status', `the board is back but runs ${running ? running.crc32 : 'no slot'}, ` +
                `not ${image.crc}: the loader kept the old image`, true);
    }
  } catch (e) {
    setStatus('update_status', `failed: ${e.message}. The board still boots what it booted before.`, true);
  } finally {
    $('progress').hidden = true;
    $('image').disabled = false;
    $('install').disabled = !image || !image.kind;
  }
};

// ------------------------------------------------------------------ web pages
async function showFiles() {
  const reply = await (await fetch('/files/', {cache: 'no-store'})).json();
  $('files').querySelector('tbody').innerHTML = reply.files.map(f =>
    `<tr><td>${f.name.replace(/\.gz$/, '')}</td><td>${f.size}</td>
     <td><button data-name="${f.name}" class="small">Delete</button></td></tr>`).join('');
  for (const button of $('files').querySelectorAll('button[data-name]')) {
    button.onclick = async () => {
      const name = button.dataset.name;
      if (!confirm(`Delete ${name} from the board?`)) return;
      await fetch(`/files/${encodeURIComponent(name)}`, {method: 'DELETE'});
      await showFiles();
    };
  }
}

$('web_files').addEventListener('change', () => {
  $('upload').disabled = $('web_files').files.length === 0;
});

async function gzip(file) {
  const stream = file.stream().pipeThrough(new CompressionStream('gzip'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

$('upload').onclick = async () => {
  const files = [...$('web_files').files].filter(f => !f.name.startsWith('LICENSE'));
  $('upload').disabled = true;
  try {
    for (const [i, file] of files.entries()) {
      setStatus('files_status', `${i + 1}/${files.length}: ${file.name}…`);
      const reply = await upload(`/files/${encodeURIComponent(file.name)}.gz`, await gzip(file), () => {});
      const result = JSON.parse(reply.text);
      if (!result.ok) throw new Error(`${file.name}: ${result.message || reply.status}`);
    }
    setStatus('files_status', `uploaded ${files.length} files; reload a page to use them`);
  } catch (e) {
    setStatus('files_status', `failed: ${e.message}`, true);
  } finally {
    $('upload').disabled = false;
    await showFiles();
  }
};

showSlots().catch(e => setStatus('update_status', `cannot read the slots: ${e.message}`, true));
showFiles().catch(e => setStatus('files_status', `cannot read the files: ${e.message}`, true));
