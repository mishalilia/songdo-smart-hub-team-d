// Author: Mikhail Li (12236295)
// Shared helpers for the control page and the dashboard.
const $ = (sel) => document.querySelector(sel);

async function getJSON(path) {
  const r = await fetch(path, {cache: 'no-store'});
  if (!r.ok) throw new Error(path + ' ' + r.status);
  return r.json();
}

// Not named `command`: inside inline onclick handlers that name resolves to the
// <button>.command property (Invoker Commands API) in current browsers.
async function sendCommand(cmd, args = {}) {
  await fetch('/api/command', {method: 'POST', headers: {'Content-Type': 'application/json'},
                               body: JSON.stringify({cmd, ...args})});
}

function pill(text, cls) {
  return `<span class="pill ${cls || ''}">${text}</span>`;
}

function fmt(v, digits = 1) {
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(digits) : (v ?? '–');
}

function escapeHtml(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;',
                                                     '"': '&quot;'}[c]));
}

// ---------------------------------------------------------------- map drawing
const EDGE_COLOURS = {NORMAL: '#9aa3b5', CONGESTED: '#d98a00', BLOCKED: '#d64545',
                      UNKNOWN: '#4a5163'};
const SLOT_COLOURS = {FREE: '#1f9d55', RESERVED: '#d98a00', OCCUPIED: '#d64545'};
const AGENT_STYLE = {bus: ['#2f6fde', 0.30], car: ['#9aa3b5', 0.30], robot: ['#1f9d55', 0.20],
                     obstacle: ['#d64545', 0.35]};

function drawMap(canvas, map, st, tick) {
  const ctx = canvas.getContext('2d');
  const [xmin, xmax, ymin, ymax] = map.bounds;
  const pad = 0.6;
  const W = canvas.width, H = canvas.height;
  const s = Math.min(W / (xmax - xmin + 2 * pad), H / (ymax - ymin + 2 * pad));
  const X = (x) => (x - xmin + pad) * s;
  const Y = (y) => H - (y - ymin + pad) * s;
  ctx.clearRect(0, 0, W, H);

  const rect = (r, fill, stroke) => {
    ctx.fillStyle = fill; ctx.fillRect(X(r[0]), Y(r[3]), (r[1] - r[0]) * s, (r[3] - r[2]) * s);
    if (stroke) { ctx.strokeStyle = stroke; ctx.strokeRect(X(r[0]), Y(r[3]),
                                                            (r[1] - r[0]) * s, (r[3] - r[2]) * s); }
  };
  ctx.lineWidth = 1;
  rect(map.bounds, 'rgba(255,255,255,0.03)', 'rgba(255,255,255,0.15)');
  rect(map.queue, 'rgba(47,111,222,0.18)', 'rgba(47,111,222,0.6)');
  const c = st.crossing || {};
  const cFill = !c.valid ? 'rgba(138,147,166,0.25)' : c.pedestrian ? 'rgba(214,69,69,0.35)'
    : c.owner ? 'rgba(217,138,0,0.35)' : 'rgba(31,157,85,0.25)';
  rect(map.crossing, cFill, 'rgba(255,255,255,0.35)');

  const edges = st.edges || {};
  const line = (a, b, colour, width, dash) => {
    ctx.strokeStyle = colour; ctx.lineWidth = width; ctx.setLineDash(dash || []);
    ctx.beginPath(); ctx.moveTo(X(map.nodes[a][0]), Y(map.nodes[a][1]));
    ctx.lineTo(X(map.nodes[b][0]), Y(map.nodes[b][1])); ctx.stroke(); ctx.setLineDash([]);
  };
  const id = (a, b) => [a, b].sort().join('-');
  const vehicleIds = new Set([...map.vehicle_edges, ...map.parking_edges].map(([a, b]) => id(a, b)));
  for (const [a, b] of map.robot_edges) {
    if (!vehicleIds.has(id(a, b))) {
      const e = edges[id(a, b)];
      line(a, b, e && e.label === 'BLOCKED' ? '#d64545' : '#4cc3ff', 2, [6, 5]);
    }
  }
  for (const [a, b] of [...map.vehicle_edges, ...map.parking_edges]) {
    const e = edges[id(a, b)];
    line(a, b, EDGE_COLOURS[e ? e.label : 'UNKNOWN'], 5);
  }
  ctx.font = `${Math.max(10, s * 0.22)}px system-ui`;
  for (const [n, [x, y]] of Object.entries(map.nodes)) {
    ctx.fillStyle = '#e6e9ef'; ctx.fillText(n, X(x) + 5, Y(y) - 5);
  }
  const slots = {};
  for (const sl of (st.parking && st.parking.slots) || []) slots[sl.slot] = sl.state;
  for (const n of map.slots) {
    const state = slots[n] || 'UNKNOWN';
    const blink = state === 'UNKNOWN' && tick % 2 === 0;
    ctx.fillStyle = blink ? 'rgba(217,138,0,0.2)' : (SLOT_COLOURS[state] || '#d98a00');
    const [x, y] = map.nodes[n];
    ctx.fillRect(X(x - 0.5), Y(y + 1.0), s, 0.7 * s);
    ctx.fillStyle = '#fff'; ctx.fillText(state, X(x - 0.45), Y(y + 1.15));
  }
  for (const [x, y] of st.pedestrians || []) {
    ctx.fillStyle = '#f5f5f5'; ctx.beginPath(); ctx.arc(X(x), Y(y), 0.15 * s, 0, 2 * Math.PI);
    ctx.fill();
  }
  for (const v of st.vehicles || []) {
    const [colour, r] = AGENT_STYLE[v.kind] || ['#fff', 0.25];
    ctx.globalAlpha = v.valid ? 1 : 0.4;
    ctx.fillStyle = colour; ctx.beginPath(); ctx.arc(X(v.x), Y(v.y), r * s, 0, 2 * Math.PI);
    ctx.fill();
    if (v.kind !== 'obstacle') {
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.beginPath();
      ctx.moveTo(X(v.x), Y(v.y));
      ctx.lineTo(X(v.x + Math.cos(v.theta) * r), Y(v.y + Math.sin(v.theta) * r)); ctx.stroke();
    }
    ctx.globalAlpha = 1;
    if (v.kind === 'bus' || v.kind === 'robot' || v.kind === 'obstacle') {
      ctx.fillStyle = '#e6e9ef'; ctx.fillText(v.id, X(v.x) - r * s, Y(v.y) - r * s - 3);
    }
  }
}
