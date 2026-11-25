'use strict';

// ============================================================================
//  app.js — VW CDC BLE Web UI Logic (Single-Flight Queue + Live Learn Mode)
// ============================================================================

const HID_PRESETS = [
  ['0x0000', 'Disabled (0x0000)'],
  ['0x00CD', 'Media: Play / Pause (0x00CD)'],
  ['0x00B5', 'Media: Next Track (0x00B5)'],
  ['0x00B6', 'Media: Prev Track (0x00B6)'],
  ['0x00B7', 'Media: Stop (0x00B7)'],
  ['0x00B3', 'Media: Fast Forward (0x00B3)'],
  ['0x00B4', 'Media: Rewind (0x00B4)'],
  ['0x00E9', 'Media: Volume Up (0x00E9)'],
  ['0x00EA', 'Media: Volume Down (0x00EA)'],
  ['0x00E2', 'Media: Mute (0x00E2)'],
  ['0xF001', 'Key: F1 (0xF001)'],
  ['0xF002', 'Key: F2 (0xF002)'],
  ['0xF003', 'Key: F3 (0xF003)'],
  ['0xF004', 'Key: F4 (0xF004)'],
  ['0xF005', 'Key: F5 (0xF005)'],
  ['0xF006', 'Key: F6 (0xF006)'],
  ['0xF007', 'Key: F7 (0xF007)'],
  ['0xF008', 'Key: F8 (0xF008)'],
  ['0xF009', 'Key: F9 (0xF009)'],
  ['0xF00A', 'Key: F10 (0xF00A)'],
  ['0xF00B', 'Key: F11 (0xF00B)'],
  ['0xF00C', 'Key: F12 (0xF00C)'],
  ['0xF00D', 'Key: F13 (0xF00D)'],
  ['0xF00E', 'Key: F14 (0xF00E)'],
  ['0xF00F', 'Key: F15 (0xF00F)'],
  ['0xF010', 'Key: F16 (0xF010)'],
  ['0xF011', 'Key: F17 (0xF011)'],
  ['0xF012', 'Key: F18 (0xF012)'],
  ['0xF013', 'Key: F19 (0xF013)'],
  ['0xF014', 'Key: F20 (0xF014)'],
  ['0xF015', 'Key: F21 (0xF015)'],
  ['0xF016', 'Key: F22 (0xF016)'],
  ['0xF017', 'Key: F23 (0xF017)'],
  ['0xF018', 'Key: F24 (0xF018)'],
];

const RADIO_ACTIONS = [
  [0, 'None (BLE only)'],
  [1, 'Track +1 (Next)'],
  [2, 'Track -1 (Prev)'],
  [3, 'Sticky Disc (stays on disc)'],
  [4, 'Momentary Disc (flash 250ms & return)'],
  [5, 'Toggle SCAN indicator'],
  [6, 'Toggle MIX indicator'],
];

const BLE_EVENT_NAMES = {
  1: 'CONNECT',
  2: 'DISCONNECT',
  3: 'MAP_READ',
  5: 'SUB_MEDIA',
  6: 'SUB_KEYS',
  7: 'UNSUB_MEDIA',
  8: 'UNSUB_KEYS',
};

const KNOWN_VW_HINTS = {
  '0x0C': 'Likely: DISC 1 (RCD300/310)',
  '0x08': 'Likely: DISC 1 (RNS/MFD)',
  '0x8C': 'Likely: DISC 2',
  '0x4C': 'Likely: DISC 3',
  '0xCC': 'Likely: DISC 4',
  '0x2C': 'Likely: DISC 5',
  '0xAC': 'Likely: DISC 6',
  '0xF8': 'Likely: NEXT TRACK',
  '0x78': 'Likely: PREV TRACK',
  '0xA0': 'Likely: SCAN',
  '0xE0': 'Likely: MIX / RANDOM',
  '0xD8': 'Likely: SEEK FWD',
  '0x58': 'Likely: SEEK REV',
};

// Single-flight request queue so AsyncTCP on ESP32-C3 handles 1 request at a time
let qBusy = false;
const qQueue = [];

function enqueueReq(url, opts, onOk, onErr) {
  qQueue.push({ url, opts: opts || {}, onOk, onErr, retried: false });
  pumpQueue();
}

function pumpQueue() {
  if (qBusy || qQueue.length === 0) return;
  qBusy = true;
  const job = qQueue.shift();

  fetch(job.url, Object.assign({ cache: 'no-store' }, job.opts))
    .then(async (res) => {
      const text = await res.text();
      if (!res.ok) {
        if (job.onErr) job.onErr(text || ('HTTP ' + res.status));
        return;
      }
      if (job.onOk) job.onOk(text);
    })
    .catch((err) => {
      if (!job.retried && (!job.opts.method || job.opts.method === 'GET')) {
        job.retried = true;
        qQueue.unshift(job);
      } else if (job.onErr) {
        job.onErr(err ? String(err) : 'Network error');
      }
    })
    .finally(() => {
      qBusy = false;
      setTimeout(pumpQueue, 30);
    });
}

function postForm(url, params, onOk, onErr) {
  const body = new URLSearchParams(params).toString();
  enqueueReq(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body,
  }, onOk, onErr || ((msg) => showToast(msg, true)));
}

let toastTimer = null;
function showToast(msg, isError) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className = 'show ' + (isError ? 'toast-err' : 'toast-ok');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.className = ''; }, 3200);
}

function normalizeHex16(raw) {
  const clean = String(raw || '').trim().replace(/^0x/i, '');
  if (!/^[0-9a-fA-F]{1,4}$/.test(clean)) return null;
  const val = parseInt(clean, 16);
  return '0x' + val.toString(16).toUpperCase().padStart(4, '0');
}

function normalizeHex8(raw) {
  const clean = String(raw || '').trim().replace(/^0x/i, '');
  if (!/^[0-9a-fA-F]{1,2}$/.test(clean)) return null;
  const val = parseInt(clean, 16);
  return '0x' + val.toString(16).toUpperCase().padStart(2, '0');
}

function buildPresetSelectHtml(selectedHex, customAllowed) {
  const norm = normalizeHex16(selectedHex) || '0x0000';
  let found = false;
  let html = '';
  for (const [val, label] of HID_PRESETS) {
    const sel = (val === norm) ? ' selected' : '';
    if (sel) found = true;
    html += `<option value="${val}"${sel}>${label}</option>`;
  }
  if (customAllowed) {
    html += `<option value="__CUSTOM__"${found ? '' : ' selected'}>Custom Hex</option>`;
  }
  return html;
}

function populateStaticSelect(selectId, defaultHex) {
  const el = document.getElementById(selectId);
  if (!el) return;
  el.innerHTML = buildPresetSelectHtml(defaultHex, true);
}

function syncPresetToInput(selectId, inputId) {
  const sel = document.getElementById(selectId);
  const inp = document.getElementById(inputId);
  if (!sel || !inp) return;
  if (sel.value !== '__CUSTOM__') {
    inp.value = sel.value;
  }
}

function syncInputToPreset(inputId, selectId) {
  const sel = document.getElementById(selectId);
  const inp = document.getElementById(inputId);
  if (!sel || !inp) return;
  const norm = normalizeHex16(inp.value);
  const match = norm && HID_PRESETS.some(([val]) => val === norm);
  sel.value = match ? norm : '__CUSTOM__';
}

// ============================================================================
//  TABLE STATE (preserves unsaved edits while merging newly learned packets)
// ============================================================================
let localRows = [];
let tableDirty = false;
let keepaliveLoaded = false;
let lastHighlightCode = '';

function setDirty(dirty) {
  tableDirty = dirty;
  document.getElementById('dirtyBadge').classList.toggle('hidden', !dirty);
}

function actionBadgeHtml(act) {
  if (act === 3) return '<span class="pill pill-sticky">STICKY</span>';
  if (act === 4) return '<span class="pill pill-momentary">FLASH</span>';
  return '';
}

function renderTable() {
  const tbody = document.getElementById('cmdBody');
  if (!localRows.length) {
    tbody.innerHTML = '<tr><td colspan="5" class="muted">No button mappings captured yet. Press any button on your radio to learn it automatically, or add one manually.</td></tr>';
    return;
  }

  let html = '';
  localRows.forEach((row, idx) => {
    const isActive = (lastHighlightCode && row.code === lastHighlightCode);
    const isDiscAct = (row.act === 3 || row.act === 4);
    const hint = KNOWN_VW_HINTS[row.code] || '';

    let actOpts = '';
    for (const [val, label] of RADIO_ACTIONS) {
      actOpts += `<option value="${val}"${row.act === val ? ' selected' : ''}>${label}</option>`;
    }

    let discOpts = '';
    for (let d = 1; d <= 6; d++) {
      discOpts += `<option value="${d}"${row.param === d ? ' selected' : ''}>CD ${d}</option>`;
    }

    html += `<tr class="${isActive ? 'row-active' : ''}" data-idx="${idx}">
      <td>
        <span class="pkt-badge">${row.code}</span>
        ${hint ? `<div class="pkt-hint">${escapeAttr(hint)}</div>` : ''}
      </td>
      <td><input class="input-name" maxlength="15" value="${escapeAttr(row.name)}" oninput="onRowName(${idx}, this.value)"></td>
      <td>
        <div class="cell-stack">
          <select id="hp_${idx}" onchange="onRowPreset(${idx}, this.value)">
            ${buildPresetSelectHtml(row.hid, true)}
          </select>
          <input id="hi_${idx}" class="input-hex" value="${escapeAttr(row.hid)}" oninput="onRowHex(${idx}, this.value)">
        </div>
      </td>
      <td>
        <div class="cell-stack">
          <select onchange="onRowAction(${idx}, this.value)">${actOpts}</select>
          <select class="select-disc ${isDiscAct ? '' : 'hidden'}" onchange="onRowParam(${idx}, this.value)">${discOpts}</select>
          <span id="ab_${idx}">${actionBadgeHtml(row.act)}</span>
        </div>
      </td>
      <td>
        <div class="cell-stack">
          <button type="button" class="btn-sm" onclick="testRowHid(${idx})">Test</button>
          <button type="button" class="btn-sm btn-danger" onclick="deleteRow('${row.code}')">Delete</button>
        </div>
      </td>
    </tr>`;
  });

  tbody.innerHTML = html;
}

function escapeAttr(str) {
  return String(str || '')
    .replace(/&/g, '&amp;')
    .replace(/"/g, '&quot;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;');
}

function onRowName(idx, val) {
  localRows[idx].name = val.replace(/[\t\r\n]/g, ' ').slice(0, 15);
  setDirty(true);
}

function onRowPreset(idx, val) {
  if (val !== '__CUSTOM__') {
    localRows[idx].hid = val;
    document.getElementById('hi_' + idx).value = val;
    setDirty(true);
  }
}

function onRowHex(idx, val) {
  localRows[idx].hid = val;
  const norm = normalizeHex16(val);
  const sel = document.getElementById('hp_' + idx);
  if (sel) {
    const match = norm && HID_PRESETS.some(([preset]) => preset === norm);
    sel.value = match ? norm : '__CUSTOM__';
  }
  setDirty(true);
}

function onRowAction(idx, val) {
  const act = parseInt(val, 10) || 0;
  localRows[idx].act = act;
  if ((act === 3 || act === 4) && (localRows[idx].param < 1 || localRows[idx].param > 6)) {
    localRows[idx].param = 1;
  }
  setDirty(true);
  renderTable();
}

function onRowParam(idx, val) {
  localRows[idx].param = parseInt(val, 10) || 1;
  setDirty(true);
}

function testRowHid(idx) {
  const hex = normalizeHex16(localRows[idx].hid);
  if (!hex || hex === '0x0000') {
    showToast('Row has no HID code assigned (0x0000)', true);
    return;
  }
  postForm('/api/cmd', { c: hex }, (txt) => showToast(txt, false));
}

function deleteRow(code) {
  postForm('/api/btn/delete', { code }, (txt) => {
    setDirty(false);
    showToast(txt, false);
    fetchStateNow();
  });
}

function saveAllButtons() {
  const lines = [];
  for (const row of localRows) {
    const code = normalizeHex8(row.code);
    const hid  = normalizeHex16(row.hid);
    if (!code) { showToast('Invalid packet code: ' + row.code, true); return; }
    if (!hid)  { showToast('Invalid HID hex in ' + row.name + ': ' + row.hid, true); return; }
    const cleanName = (row.name || ('CMD_' + code)).replace(/[\t\r\n]/g, ' ').trim();
    const param = (row.act === 3 || row.act === 4) ? (row.param || 1) : 0;
    lines.push(`${code}\t${cleanName}\t${hid}\t${row.act}\t${param}`);
  }

  postForm('/api/btns/save', { rows: lines.join('\n') }, (txt) => {
    setDirty(false);
    showToast(txt, false);
    fetchStateNow();
  });
}

function clearAllButtons() {
  if (!confirm('Remove ALL learned and configured buttons?')) return;
  postForm('/api/btn/reset', { mode: 'clear' }, (txt) => {
    setDirty(false);
    showToast(txt, false);
    fetchStateNow();
  });
}

function toggleAddManual() {
  document.getElementById('manualAddBox').classList.toggle('hidden');
}

function addManualEntry() {
  const code = normalizeHex8(document.getElementById('addCode').value);
  const hid  = normalizeHex16(document.getElementById('addHid').value);
  const name = (document.getElementById('addName').value || '').replace(/[\t\r\n]/g, ' ').trim();
  if (!code) { showToast('Enter a valid 1-byte packet hex (e.g. 0x0C)', true); return; }
  if (!hid)  { showToast('Enter a valid 16-bit HID code (e.g. 0x00CD)', true); return; }

  const existing = localRows.find((r) => r.code === code);
  if (existing) {
    existing.name = name || existing.name;
    existing.hid  = hid;
  } else {
    localRows.push({
      code,
      name: (name || ('CMD_' + code)).slice(0, 15),
      hid,
      act: 0,
      param: 0,
    });
  }
  setDirty(true);
  renderTable();
  document.getElementById('addCode').value = '';
  document.getElementById('addName').value = '';
  showToast('Added ' + code + ' — click Save All Changes to persist', false);
}

// ============================================================================
//  CONTROLS, KEEPALIVE & DIAGNOSTICS
// ============================================================================
function saveKeepalive() {
  const en = document.getElementById('kaEnabled').checked ? '1' : '0';
  const ms = parseInt(document.getElementById('kaInterval').value, 10) || 5000;
  const hid = normalizeHex16(document.getElementById('kaCode').value);
  if (!hid) {
    showToast('Invalid keepalive HID hex code', true);
    return;
  }
  postForm('/api/keepalive', { en, ms: String(ms), hid }, (txt) => {
    keepaliveLoaded = false;
    showToast(txt, false);
  });
}

function sendManualTest() {
  const hid = normalizeHex16(document.getElementById('testHex').value);
  if (!hid || hid === '0x0000') {
    showToast('Enter a valid HID code (0x0001..0x03FF or 0xF001..0xF018)', true);
    return;
  }
  postForm('/api/cmd', { c: hid }, (txt) => showToast(txt, false));
}

function toggleAdv() {
  postForm('/api/tadv', {}, (txt) => { showToast(txt, false); fetchStateNow(); });
}

function togglePrefix() {
  postForm('/api/prefix', {}, (txt) => { showToast(txt, false); fetchStateNow(); });
}

function deleteBonds() {
  const devName = document.getElementById('devTitle').textContent || 'RCD310';
  const msg = `Delete all BLE pairings and reboot?\n\nAfter reboot on your phone:\n1. Bluetooth settings -> ${devName} -> Forget device\n2. Re-pair cleanly so Android refreshes the HID descriptor.`;
  if (!confirm(msg)) return;
  postForm('/api/delbonds', {}, (txt) => {
    showToast(txt, false);
    setTimeout(() => location.reload(), 15000);
  });
}

function loadDump() {
  enqueueReq('/api/dump', { method: 'GET' }, (txt) => {
    const el = document.getElementById('dumpBox');
    el.value = txt;
    el.focus();
    el.select();
  });
}

function copyDump() {
  const el = document.getElementById('dumpBox');
  if (!el.value) { loadDump(); return; }
  if (navigator.clipboard) {
    navigator.clipboard.writeText(el.value).then(() => showToast('Diagnostic dump copied', false));
  } else {
    el.select();
    document.execCommand('copy');
    showToast('Diagnostic dump copied', false);
  }
}

// ============================================================================
//  STATE POLLING
// ============================================================================
function applyState(d) {
  if (d.dev) {
    document.getElementById('devTitle').textContent = d.dev + ' CDC Control';
    document.title = d.dev + ' CDC Control';
  }
  if (d.fw) document.getElementById('fwBadge').textContent = d.fw;

  const badge = document.getElementById('bleBadge');
  badge.textContent = d.con ? 'BLE Connected' : 'BLE Disconnected';
  badge.className = 'pill ' + (d.con ? 'pill-ok' : 'pill-err');

  document.getElementById('bleText').textContent = d.con ? 'Connected' : 'Disconnected';
  document.getElementById('advText').textContent = d.adv ? 'Active' : 'Inactive';

  if (d.ka && !keepaliveLoaded) {
    keepaliveLoaded = true;
    document.getElementById('kaEnabled').checked = !!d.ka.en;
    document.getElementById('kaInterval').value = d.ka.ms;
    document.getElementById('kaCode').value = d.ka.hid;
    syncInputToPreset('kaCode', 'kaPreset');
  }

  if (d.lastRx && d.lastRx.valid) {
    document.getElementById('lastRxCode').textContent = d.lastRx.code;
    document.getElementById('lastRxName').textContent = d.lastRx.name;
    const rxHint = KNOWN_VW_HINTS[d.lastRx.code] || '';
    document.getElementById('lastRxHint').textContent = rxHint ? ` (${rxHint})` : '';
    document.getElementById('lastRxAge').textContent = ` (${(d.lastRx.ageMs / 1000).toFixed(1)}s ago)`;
    lastHighlightCode = d.lastRx.code;
  }

  if (Array.isArray(d.btns)) {
    if (!tableDirty) {
      const serverJson = JSON.stringify(d.btns);
      if (serverJson !== JSON.stringify(localRows)) {
        localRows = d.btns;
        renderTable();
      } else {
        // Still update active packet highlight without rebuilding inputs
        document.querySelectorAll('#cmdBody tr').forEach((tr) => {
          const idx = parseInt(tr.getAttribute('data-idx'), 10);
          const row = localRows[idx];
          tr.classList.toggle('row-active', !!(row && lastHighlightCode && row.code === lastHighlightCode));
        });
      }
    } else {
      // Merge any newly learned packet rows without discarding user edits
      let added = false;
      for (const srvRow of d.btns) {
        if (!localRows.some((r) => r.code === srvRow.code)) {
          localRows.push(srvRow);
          added = true;
        }
      }
      if (added) renderTable();
    }
  }

  if (d.dbg) {
    document.getElementById('mSubs').textContent = d.dbg.mSubs;
    document.getElementById('kSubs').textContent = d.dbg.kSubs;
    document.getElementById('diagCounters').textContent =
      `Connections: ${d.dbg.conCnt} | Disconnects: ${d.dbg.discCnt}\n` +
      `Report Map reads: ${d.dbg.mapRds}${d.dbg.mapRds === 0 ? ' (0 = phone may use cached GATT)' : ''}\n` +
      `Media subs: ${d.dbg.mSubs} | Keyboard subs: ${d.dbg.kSubs}\n` +
      `Reports sent: ${d.dbg.sends} | Rejected: ${d.dbg.rej}\n` +
      `Last report: ${d.dbg.lastCode} -> [${d.dbg.lastRpt}] @ ${d.dbg.lastMs} ms\n` +
      `Report ID prefix: ${d.dbg.pfx ? 'ON' : 'OFF'}`;

    const evts = (d.dbg.evts || []).map(([t, ms]) =>
      `${BLE_EVENT_NAMES[t] || ('?' + t)}@${ms}ms`
    ).join(' | ');
    document.getElementById('diagEvents').textContent = 'Events: ' + (evts || 'none');
  }
}

function fetchStateNow() {
  enqueueReq('/api/state', { method: 'GET' }, (txt) => {
    try { applyState(JSON.parse(txt)); } catch (e) {}
  });
}

function pollLoop() {
  fetchStateNow();
  setTimeout(pollLoop, 2000);
}

// Initialize static dropdowns and start live telemetry polling
populateStaticSelect('testPreset', '0x00CD');
populateStaticSelect('addHidPreset', '0x0000');
populateStaticSelect('kaPreset', '0xF008');
pollLoop();
