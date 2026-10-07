/* RM_radar_Cpp_2027 调试台前端逻辑 */
"use strict";

const $ = (id) => document.getElementById(id);

const App = {
  state: null,
  generation: 0,
  connected: false,
  selectedRobot: null,
  debug: null,
  mapImage: null,
  mapImageLoaded: false,
  activePage: "map",
  hoverPos: null,
  lastRender: 0,
  lastStatusRender: 0,
  lastListRender: 0,
};

const FIELD_DEFAULT = { width: 28, height: 15 };

function fieldSize() {
  const scene = App.state?.config?.scene || {};
  return {
    width: Number(scene.field_width || FIELD_DEFAULT.width),
    height: Number(scene.field_height || FIELD_DEFAULT.height),
  };
}

function colorForId(id) {
  return Number(id) >= 100 ? "#38bdf8" : "#ef4444";
}

function shortLabel(id, label) {
  if (label && /^[RB]\d+$/i.test(label)) return label.toUpperCase();
  const n = Number(id);
  if (n >= 100) return "B" + (n - 100);
  if (n > 0 && n <= 7) return "R" + n;
  return String(label || id);
}

function fmtNumber(value, digits = 2) {
  const n = Number(value);
  if (!Number.isFinite(n)) return "—";
  return n.toFixed(digits);
}

function ageText(seconds) {
  if (seconds === undefined || seconds === null || !Number.isFinite(Number(seconds))) return "—";
  const s = Number(seconds);
  if (s < 1) return `${(s * 1000).toFixed(0)} ms`;
  return `${s.toFixed(1)} s`;
}

function timeText(ts) {
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString("zh-CN", { hour12: false });
}

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, (ch) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  }[ch]));
}

function debugValue(key, fallback) {
  const d = App.debug || {};
  return key in d ? d[key] : fallback;
}

/* ----------------------------- SSE / 状态 ----------------------------- */
async function loadStateOnce() {
  try {
    const res = await fetch("/api/state", { cache: "no-store" });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    applyState(await res.json());
  } catch (err) {
    $("summary").textContent = `HTTP 状态请求失败: ${err.message}`;
  }
}

async function loadDebugOnce() {
  try {
    const res = await fetch("/api/debug", { cache: "no-store" });
    if (!res.ok) return;
    const payload = await res.json();
    App.debug = payload.debug || {};
    syncDebugControls();
  } catch (err) {
    console.warn("debug load failed", err);
  }
}

function connectEvents() {
  if (typeof EventSource === "undefined") {
    $("connBadge").className = "badge bad";
    $("connBadge").textContent = "浏览器不支持 SSE";
    return;
  }
  const source = new EventSource(`/api/events?generation=${App.generation}`);
  source.onopen = () => {
    App.connected = true;
    updateHeader();
  };
  source.onmessage = (event) => {
    try {
      applyState(JSON.parse(event.data));
    } catch (err) {
      console.error("SSE parse failed", err);
    }
  };
  source.onerror = () => {
    App.connected = false;
    updateHeader();
  };
  App.eventSource = source;
}

function applyState(state) {
  if (!state || !state.ok) return;
  App.state = state;
  App.generation = Math.max(App.generation, Number(state.generation || 0));
  App.connected = true;

  if (App.selectedRobot === null && state.robots?.length) {
    const newest = [...state.robots].filter((r) => r.current).sort((a, b) => a.last_seen_age - b.last_seen_age)[0];
    if (newest) App.selectedRobot = Number(newest.id);
  }
  updateHeader();
  if (App.activePage === "map") {
    updateRobotList();
    renderRobotDetail();
  }
}

function updateHeader() {
  const badge = $("connBadge");
  if (!App.connected) {
    badge.className = "badge bad";
    badge.textContent = "SSE 断开";
  } else {
    badge.className = "badge ok";
    badge.textContent = "实时推送中";
  }
  $("genBadge").textContent = `gen ${App.generation}`;
  const node = App.state?.node;
  if (node) {
    $("summary").textContent = `${node.name} · 运行 ${fmtNumber(node.uptime, 0)}s · ${node.host}:${node.port}`;
  }
}

/* ----------------------------- 地图渲染 ----------------------------- */
function setupCanvas(canvas) {
  const parent = canvas.parentElement;
  const rect = parent.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(60, Math.floor(rect.width));
  const h = Math.max(60, Math.floor(rect.height));
  const pw = Math.floor(w * dpr), ph = Math.floor(h * dpr);
  if (canvas.width !== pw || canvas.height !== ph) {
    canvas.width = pw;
    canvas.height = ph;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

function mapTransform(w, h) {
  const f = fieldSize();
  // 按场地长宽比等比缩放并居中,避免画布拉伸导致 x/y 比例不一致
  const scale = Math.min(w / f.width, h / f.height);
  const dw = f.width * scale, dh = f.height * scale;
  const ox = (w - dw) / 2, oy = (h - dh) / 2;
  return {
    f, ox, oy, dw, dh,
    px: (x) => ox + (Number(x) / f.width) * dw,
    py: (y) => oy + ((f.height - Number(y)) / f.height) * dh,
  };
}

function drawMap() {
  const canvas = $("mapCanvas");
  if (!canvas || !App.state) return;
  const { ctx, w, h } = setupCanvas(canvas);
  const map = mapTransform(w, h);
  const debug = App.debug || {};
  ctx.fillStyle = "#060b12";
  ctx.fillRect(0, 0, w, h);

  if (debug.map_image_enabled !== false && App.mapImageLoaded && App.mapImage) {
    ctx.drawImage(App.mapImage, map.ox, map.oy, map.dw, map.dh);
  } else {
    const grad = ctx.createLinearGradient(map.ox, map.oy, map.ox + map.dw, map.oy + map.dh);
    grad.addColorStop(0, "#0a1420");
    grad.addColorStop(1, "#0e1a2c");
    ctx.fillStyle = grad;
    ctx.fillRect(map.ox, map.oy, map.dw, map.dh);
    ctx.strokeStyle = "rgba(56,189,248,0.35)";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(map.ox, map.oy, map.dw, map.dh);
    if (debug.map_show_grid !== false) {
      ctx.strokeStyle = "rgba(56,189,248,0.10)";
      ctx.lineWidth = 1;
      for (let x = 0; x <= map.f.width; x += 1) {
        const sx = map.px(x);
        ctx.beginPath(); ctx.moveTo(sx, map.oy); ctx.lineTo(sx, map.oy + map.dh); ctx.stroke();
      }
      for (let y = 0; y <= map.f.height; y += 1) {
        const sy = map.py(y);
        ctx.beginPath(); ctx.moveTo(map.ox, sy); ctx.lineTo(map.ox + map.dw, sy); ctx.stroke();
      }
    }
  }

  if (debug.map_show_labels !== false) {
    ctx.fillStyle = "rgba(219,231,245,0.45)";
    ctx.font = "11px sans-serif";
    ctx.textAlign = "center";
    for (let x = 0; x <= map.f.width; x += 2) {
      ctx.fillText(`${x}`, map.px(x), map.oy + map.dh + 14);
    }
    ctx.textAlign = "left";
    for (let y = 0; y <= map.f.height; y += 2) {
      ctx.fillText(`${y}`, map.ox - 22, map.py(y) + 3);
    }
  }

  const robots = App.state.robots || [];
  const showLastKnown = debug.show_last_known !== false;

  // 轨迹
  if (debug.trail_enabled !== false) {
    for (const robot of robots) {
      const trail = robot.trail || [];
      if (trail.length < 2) continue;
      if (robot.last_known && !showLastKnown) continue;
      const color = colorForId(robot.id);
      for (let i = 1; i < trail.length; i++) {
        const a = trail[i - 1], b = trail[i];
        ctx.strokeStyle = color;
        ctx.globalAlpha = Math.max(0.08, i / trail.length * 0.85);
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(map.px(a.x), map.py(a.y));
        ctx.lineTo(map.px(b.x), map.py(b.y));
        ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }
  }

  // 检测投影 (弱化小点)
  const detections = App.state.detections || [];
  ctx.fillStyle = "rgba(132,204,22,0.55)";
  for (const det of detections) {
    if (!Number.isFinite(Number(det.field_x)) || !Number.isFinite(Number(det.field_y))) continue;
    ctx.beginPath();
    ctx.arc(map.px(det.field_x), map.py(det.field_y), 2.5, 0, Math.PI * 2);
    ctx.fill();
  }

  // 最后已知 (灰色 X)
  if (showLastKnown) {
    for (const robot of robots) {
      if (!robot.last_known || !robot.current) continue;
      const x = map.px(robot.current.x), y = map.py(robot.current.y);
      ctx.strokeStyle = "#94a3b8";
      ctx.lineWidth = 2;
      ctx.globalAlpha = 0.75;
      ctx.beginPath();
      ctx.moveTo(x - 7, y - 7); ctx.lineTo(x + 7, y + 7);
      ctx.moveTo(x + 7, y - 7); ctx.lineTo(x - 7, y + 7);
      ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.fillStyle = "#cbd5e1";
      ctx.font = "11px sans-serif";
      ctx.textAlign = "left";
      ctx.fillText(`last ${shortLabel(robot.id, robot.current.label)}`, x + 10, y - 8);
    }
  }

  // 机器人本体
  const airIds = new Set([6, 106]);
  for (const robot of robots) {
    const c = robot.current;
    if (!c) continue;
    if (robot.last_known && !showLastKnown) continue;
    const x = map.px(c.x), y = map.py(c.y);
    const color = colorForId(robot.id);
    const selected = Number(robot.id) === Number(App.selectedRobot);
    const isAir = airIds.has(Number(robot.id)) || Number(c.z) > 0.8;
    const baseR = isAir ? Number(debug.air_marker_radius_px ?? 13) : Number(debug.robot_marker_radius_px ?? 16);

    if (isAir) {
      const r = baseR + 7;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.globalAlpha = robot.current?.stale ? 0.45 : 1;
      ctx.beginPath();
      ctx.moveTo(x, y - r); ctx.lineTo(x + r, y); ctx.lineTo(x, y + r); ctx.lineTo(x - r, y);
      ctx.closePath(); ctx.stroke();
      ctx.beginPath(); ctx.arc(x, y, 3.5, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
      ctx.globalAlpha = 1;
    } else {
      ctx.beginPath();
      ctx.arc(x, y, baseR, 0, Math.PI * 2);
      ctx.fillStyle = color + (robot.current?.stale ? "55" : "cc");
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = color;
      ctx.stroke();
    }

    if (debug.map_show_labels !== false) {
      const label = shortLabel(robot.id, c.label);
      ctx.fillStyle = color === "#38bdf8" ? "#e0f2fe" : "#fee2e2";
      ctx.font = `bold ${Math.max(11, Math.round(baseR * 0.75))}px sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(label, x, y);
      ctx.textBaseline = "alphabetic";
      if (isAir) {
        ctx.font = "10px sans-serif";
        ctx.fillText(`h=${fmtNumber(c.z, 1)}`, x, y - baseR - 14);
      }
    }

    if (selected) {
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2.5;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.arc(x, y, baseR + 8, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  // 哨兵决策
  for (const s of App.state.sentry || []) {
    const x = map.px(s.x), y = map.py(s.y);
    ctx.fillStyle = "#e879f9";
    ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = "#f0abfc"; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.fillStyle = "#fdf4ff";
    ctx.font = "10px sans-serif"; ctx.textAlign = "left";
    ctx.fillText("SENTRY", x + 9, y + 3);
  }

  // AI-NAV
  for (const n of App.state.ai_nav || []) {
    const x = map.px(n.x), y = map.py(n.y);
    ctx.strokeStyle = "#fb923c";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(x, y - 11); ctx.lineTo(x + 11, y); ctx.lineTo(x, y + 11); ctx.lineTo(x - 11, y);
    ctx.closePath(); ctx.stroke();
    ctx.fillStyle = "#ffedd5"; ctx.font = "10px sans-serif"; ctx.textAlign = "left";
    ctx.fillText("AI-NAV", x + 13, y + 3);
  }

  const hasAny = robots.length || (App.state.sentry || []).length || (App.state.ai_nav || []).length;
  $("mapEmpty").style.display = hasAny ? "none" : "flex";
  $("mapMeta").textContent =
    `坐标: 裁判系统系 ${map.f.width}×${map.f.height} m · 红方补给站原点 · 机器人 ${robots.filter((r) => r.current && !r.last_known).length} 辆`;
}

function nearestRobot(px, py) {
  const canvas = $("mapCanvas");
  const { w, h } = setupCanvas(canvas);
  const map = mapTransform(w, h);
  let best = null, bestDist = 42;
  for (const robot of App.state?.robots || []) {
    const c = robot.current;
    if (!c || robot.last_known) continue;
    const d = Math.hypot(map.px(c.x) - px, map.py(c.y) - py);
    if (d < bestDist) { bestDist = d; best = robot; }
  }
  return best;
}

function setupMapInteraction() {
  const viewport = $("mapViewport");
  const canvas = $("mapCanvas");
  canvas.addEventListener("mousemove", (event) => {
    const rect = canvas.getBoundingClientRect();
    const robot = nearestRobot(event.clientX - rect.left, event.clientY - rect.top);
    const tip = $("mapTooltip");
    if (!robot) { tip.hidden = true; canvas.style.cursor = "default"; return; }
    const c = robot.current;
    tip.hidden = false;
    tip.style.left = `${Math.min(rect.width - 260, event.clientX - rect.left + 14)}px`;
    tip.style.top = `${Math.max(8, event.clientY - rect.top - 10)}px`;
    tip.innerHTML =
      `<b style="color:${colorForId(robot.id)}">${escapeHtml(shortLabel(robot.id, c.label))}</b> ` +
      `(${fmtNumber(c.x)}, ${fmtNumber(c.y)}, z=${fmtNumber(c.z)})<br>` +
      `来源 ${escapeHtml(c.source || "—")} · 年龄 ${ageText(c.age)} · 轨迹 ${robot.trail?.length || 0} 点`;
    canvas.style.cursor = "pointer";
  });
  canvas.addEventListener("mouseleave", () => { $("mapTooltip").hidden = true; });
  canvas.addEventListener("click", (event) => {
    const rect = canvas.getBoundingClientRect();
    const robot = nearestRobot(event.clientX - rect.left, event.clientY - rect.top);
    App.selectedRobot = robot ? Number(robot.id) : null;
    renderRobotDetail();
    updateRobotList();
  });
}

/* ----------------------------- 机器人列表 / 详情 ----------------------------- */
function updateRobotList() {
  const robots = App.state?.robots || [];
  $("robotCount").textContent = `${robots.length} 条轨迹`;
  const box = $("robotList");
  if (!robots.length) {
    box.innerHTML = `<div class="empty">暂无机器人</div>`;
    return;
  }
  const rows = [...robots].sort((a, b) => (a.last_seen_age || 0) - (b.last_seen_age || 0));
  box.innerHTML = rows.map((r) => {
    const c = r.current;
    const color = colorForId(r.id);
    const label = shortLabel(r.id, c?.label);
    const selected = Number(r.id) === Number(App.selectedRobot);
    return `<div class="robot-row ${selected ? "selected" : ""}" data-id="${r.id}">
      <span class="robot-chip ${color === "#38bdf8" ? "blue" : "red"}">${escapeHtml(label)}</span>
      <span>${r.last_known ? "最后已知" : (c?.stale ? "可能丢失" : "跟踪中")}</span>
      <span class="pos">${c ? `${fmtNumber(c.x)}, ${fmtNumber(c.y)}` : "—"}</span>
      <span class="age">${ageText(r.last_seen_age)}</span>
    </div>`;
  }).join("");
  box.querySelectorAll(".robot-row").forEach((row) => {
    row.addEventListener("click", () => {
      App.selectedRobot = Number(row.dataset.id);
      renderRobotDetail();
      updateRobotList();
    });
  });
}

function velocityOf(robot) {
  const trail = robot.trail || [];
  if (trail.length < 2) return null;
  const a = trail[trail.length - 2], b = trail[trail.length - 1];
  const dt = b.t - a.t;
  if (!dt) return null;
  return {
    vx: (b.x - a.x) / dt,
    vy: (b.y - a.y) / dt,
    speed: Math.hypot((b.x - a.x) / dt, (b.y - a.y) / dt),
    dt,
  };
}

function renderRobotDetail() {
  const box = $("robotDetail");
  const robots = App.state?.robots || [];
  let robot = robots.find((r) => Number(r.id) === Number(App.selectedRobot));
  if (!robot) robot = [...robots].filter((r) => r.current && !r.last_known).sort((a, b) => a.last_seen_age - b.last_seen_age)[0];
  if (!robot) {
    box.innerHTML = `<div class="empty">暂无机器人数据。雷达融合节点输出 location 后自动显示。</div>`;
    return;
  }
  const c = robot.current;
  const color = colorForId(robot.id);
  const label = shortLabel(robot.id, c?.label);
  const vel = velocityOf(robot);
  const det = (App.state?.detections || []).filter((d) => d.label && d.label.toLowerCase() === String(c?.label || "").toLowerCase());
  const bestDet = det[0] || null;
  const rows = [
    ["机器人", `<b style="color:${color}">${escapeHtml(label)}</b> (id ${robot.id})`],
    ["阵营", escapeHtml(c?.label || "—")],
    ["数据来源", escapeHtml(c?.source || "—")],
    ["X / Y", `${fmtNumber(c?.x)} / ${fmtNumber(c?.y)} m`],
    ["Z (高度)", `${fmtNumber(c?.z)} m`],
    ["数据年龄", ageText(c?.age)],
    ["状态", robot.last_known ? "最后已知(灰叉)" : (c?.stale ? "已超时(疑似丢失)" : "跟踪中")],
    ["消息计数", robot.message_count],
    ["轨迹点数", robot.trail?.length || 0],
    ["速度", vel ? `${fmtNumber(vel.speed)} m/s (vx ${fmtNumber(vel.vx)}, vy ${fmtNumber(vel.vy)})` : "—"],
  ];
  if (bestDet) {
    rows.push(["视觉置信度", `${fmtNumber(bestDet.confidence, 3)} · track ${bestDet.track_id}`]);
    rows.push(["视觉场坐标", `${fmtNumber(bestDet.field_x)}, ${fmtNumber(bestDet.field_y)}`]);
  } else {
    rows.push(["视觉检测", "未匹配"]);
  }
  box.innerHTML =
    `<div class="detail-title">${escapeHtml(label)} · 详细数据</div>` +
    `<div class="detail-grid">${rows.map(([k, v]) => `<span class="k">${k}</span><span class="v">${v}</span>`).join("")}</div>`;
}

/* ----------------------------- 调试参数 ----------------------------- */
const DEBUG_SPECS = [
  ["map_image_enabled", "地图底图", "bool"],
  ["map_show_grid", "无底图时显示网格", "bool"],
  ["map_show_labels", "显示编号/坐标标注", "bool"],
  ["trail_enabled", "显示运动轨迹", "bool"],
  ["show_last_known", "显示最后已知位置", "bool"],
  ["trail_max_points", "轨迹保留点数", "number", 5, 1000],
  ["robot_marker_radius_px", "地面机器人标记半径(px)", "number", 4, 60],
  ["air_marker_radius_px", "空中机器人标记半径(px)", "number", 4, 60],
  ["stale_timeout_s", "目标超时阈值(s)", "number", 0.2, 30],
  ["wave_max_samples", "解析波图表采样点数", "number", 10, 20000],
];

function buildDebugControls() {
  const box = $("debugParams");
  box.innerHTML = DEBUG_SPECS.map(([key, label, kind, min, max]) => {
    const input = kind === "bool"
      ? `<input type="checkbox" data-debug="${key}" />`
      : `<input type="number" data-debug="${key}" min="${min}" max="${max}" step="${key === "stale_timeout_s" ? "0.1" : "1"}" />`;
    return `<div class="debug-row"><label for="dbg-${key}">${label}</label>${input}</div>`;
  }).join("");
  box.querySelectorAll("input[data-debug]").forEach((input) => {
    input.addEventListener("change", () => {
      const key = input.dataset.debug;
      const value = input.type === "checkbox" ? input.checked : Number(input.value);
      if (!App.debug) App.debug = {};
      App.debug[key] = value;
      pushDebugParams({ [key]: value });
    });
  });
  syncDebugControls();
}

function syncDebugControls() {
  if (!App.debug) return;
  document.querySelectorAll("input[data-debug]").forEach((input) => {
    const key = input.dataset.debug;
    const value = App.debug[key];
    if (input.type === "checkbox") input.checked = !!value;
    else if (value !== undefined) input.value = value;
  });
}

async function pushDebugParams(patch) {
  try {
    const res = await fetch("/api/debug", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(patch),
    });
    const payload = await res.json();
    if (payload.debug) {
      App.debug = { ...App.debug, ...payload.debug };
      syncDebugControls();
    }
  } catch (err) {
    console.warn("debug push failed", err);
  }
}

/* ----------------------------- 解析波页面 ----------------------------- */
function resizeCanvas(canvas) {
  const parent = canvas.parentElement;
  const rect = parent.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(80, rect.width), h = Math.max(80, rect.height);
  const pw = Math.floor(w * dpr), ph = Math.floor(h * dpr);
  if (canvas.width !== pw || canvas.height !== ph) {
    canvas.width = pw; canvas.height = ph;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

function drawLineChart(canvas, points, opts = {}) {
  const { ctx, w, h } = resizeCanvas(canvas);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#080f1a";
  ctx.fillRect(0, 0, w, h);
  if (!points || points.length < 2) {
    ctx.fillStyle = "#7f93ad";
    ctx.font = "12px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("数据不足", w / 2, h / 2);
    return;
  }
  const margin = { left: 52, right: 14, top: 14, bottom: 28 };
  const iw = w - margin.left - margin.right, ih = h - margin.top - margin.bottom;
  const xs = points.map((p) => Number(p.x)), ys = points.map((p) => Number(p.y));
  let xmin = opts.xMin ?? Math.min(...xs), xmax = opts.xMax ?? Math.max(...xs);
  let ymin = opts.yMin ?? Math.min(...ys), ymax = opts.yMax ?? Math.max(...ys);
  if (xmax - xmin < 1e-9) { xmin -= 0.5; xmax += 0.5; }
  if (ymax - ymin < 1e-9) { ymin -= 0.5; ymax += 0.5; }
  const px = (x) => margin.left + (Number(x) - xmin) / (xmax - xmin) * iw;
  const py = (y) => margin.top + ih - (Number(y) - ymin) / (ymax - ymin) * ih;

  ctx.strokeStyle = "rgba(56,189,248,0.16)";
  ctx.fillStyle = "rgba(219,231,245,0.55)";
  ctx.font = "10px sans-serif";
  ctx.lineWidth = 1;
  const xTicks = 6, yTicks = 5;
  for (let i = 0; i <= xTicks; i++) {
    const val = xmin + (xmax - xmin) * i / xTicks;
    const x = px(val);
    ctx.beginPath(); ctx.moveTo(x, margin.top); ctx.lineTo(x, margin.top + ih); ctx.stroke();
    ctx.textAlign = "center";
    ctx.fillText(opts.xLabel ? `${fmtNumber(val, opts.xDigits ?? 1)}` : fmtNumber(val, opts.xDigits ?? 1), x, h - 8);
  }
  for (let i = 0; i <= yTicks; i++) {
    const val = ymin + (ymax - ymin) * i / yTicks;
    const y = py(val);
    ctx.beginPath(); ctx.moveTo(margin.left, y); ctx.lineTo(margin.left + iw, y); ctx.stroke();
    ctx.textAlign = "right";
    ctx.fillText(fmtNumber(val, opts.yDigits ?? 2), margin.left - 6, y + 3);
  }
  if (opts.xLabel) {
    ctx.textAlign = "center";
    ctx.fillStyle = "rgba(219,231,245,0.7)";
    ctx.fillText(opts.xLabel, margin.left + iw / 2, h - 8);
  }
  if (opts.yLabel) {
    ctx.save();
    ctx.translate(12, margin.top + ih / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = "center";
    ctx.fillText(opts.yLabel, 0, 0);
    ctx.restore();
  }

  ctx.strokeStyle = opts.color || "#22d3ee";
  ctx.lineWidth = opts.lineWidth || 1.6;
  ctx.beginPath();
  points.forEach((p, i) => {
    const x = px(p.x), y = py(p.y);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  if (opts.scatter) {
    ctx.fillStyle = opts.color || "#22d3ee";
    for (const p of points) {
      ctx.beginPath(); ctx.arc(px(p.x), py(p.y), 2, 0, Math.PI * 2); ctx.fill();
    }
  }
}

function fftRadix2(samples) {
  let n = 1;
  while (n < samples.length) n <<= 1;
  if (n > 8192) n = 8192;
  const real = new Float64Array(n), imag = new Float64Array(n);
  for (let i = 0; i < n; i++) real[i] = i < samples.length ? samples[i] : 0;
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { [real[i], real[j]] = [real[j], real[i]]; [imag[i], imag[j]] = [imag[j], imag[i]]; }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = -2 * Math.PI / len;
    const wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1, ci = 0;
      for (let j = 0; j < len / 2; j++) {
        const ar = real[i + j + len / 2] * cr - imag[i + j + len / 2] * ci;
        const ai = real[i + j + len / 2] * ci + imag[i + j + len / 2] * cr;
        real[i + j + len / 2] = real[i + j] - ar;
        imag[i + j + len / 2] = imag[i + j] - ai;
        real[i + j] += ar;
        imag[i + j] += ai;
        const ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = ncr;
      }
    }
  }
  return { real, imag, n };
}

function estimateSampleRate(samples, meta) {
  if (meta?.sample_rate_hz && Number(meta.sample_rate_hz) > 0) return Number(meta.sample_rate_hz);
  if (samples.length < 2) return 1;
  const dt = Number(samples[Math.min(10, samples.length - 1)].t_ms) - Number(samples[0].t_ms);
  if (dt > 0) return samples.length * 1000 / dt;
  return 1;
}

function drawWavePage() {
  const wave = App.state?.wave;
  const latest = wave?.latest;
  const empty = $("waveEmpty");
  const badge = $("waveBadge");
  if (!wave?.available || !latest) {
    empty.style.display = "grid";
    badge.className = "badge unknown";
    badge.textContent = "等待接入";
    $("waveSummary").innerHTML = `<span>接口: HTTP POST /api/wave/ingest · ROS2 /wave_analysis (std_msgs/String JSON)</span>`;
    $("waveSampleMeta").textContent = "0 点";
    $("waveTimeMeta").textContent = "—";
    return;
  }
  empty.style.display = "none";
  const samples = latest.samples || [];
  const meta = latest.meta || {};
  badge.className = `badge ${latest.status === "invalid" ? "bad" : latest.status === "warn" ? "warn" : "ok"}`;
  badge.textContent = `${latest.status || "valid"} · ${latest.source || "wave"}`;
  $("waveSummary").innerHTML =
    `<span>来源 <b>${escapeHtml(latest.source || "—")}</b> · seq ${latest.seq ?? "—"} · ` +
    `模式 ${escapeHtml(latest.mode || "—")} · 采样 ${samples.length} 点 · ` +
    `最近 ${timeText(latest.received_at)}</span>`;

  const timePoints = samples.filter((s) => Number.isFinite(Number(s.t_ms)) && Number.isFinite(Number(s.amplitude)));
  drawLineChart($("waveCanvas"), timePoints.map((s) => ({ x: s.t_ms, y: s.amplitude })), {
    color: "#22d3ee", xLabel: "时间 ms", yLabel: "振幅",
  });
  const ampValues = timePoints.map((s) => Number(s.amplitude));
  const ampMin = ampValues.reduce((a, b) => Math.min(a, b), Infinity);
  const ampMax = ampValues.reduce((a, b) => Math.max(a, b), -Infinity);
  $("waveTimeMeta").textContent = `${timePoints.length} 点 · 幅值 ${fmtNumber(ampMin)} ~ ${fmtNumber(ampMax)}`;

  // FFT
  const nForFft = Math.min(8192, Math.max(1, timePoints.length));
  const fftSamples = timePoints.slice(0, nForFft).map((s) => Number(s.amplitude));
  if (fftSamples.length >= 4) {
    const { real, imag, n } = fftRadix2(fftSamples);
    const sr = estimateSampleRate(timePoints.slice(0, nForFft), meta);
    const pts = [];
    for (let i = 0; i <= n / 2; i++) {
      const mag = Math.hypot(real[i], imag[i]);
      pts.push({ x: i * sr / n, y: mag });
    }
    const maxMag = pts.reduce((a, p) => Math.max(a, p.y), 1e-9);
    drawLineChart($("fftCanvas"), pts.filter((_, i) => i % Math.max(1, Math.floor(pts.length / 2000)) === 0 || i === pts.length - 1),
      { color: "#a78bfa", xLabel: "频率 Hz", yLabel: "幅度", yDigits: 1 });
    $("waveFftMeta").textContent = `FFT ${n} 点 · 采样率 ${fmtNumber(sr, 1)} Hz · 峰值 ${fmtNumber(maxMag, 1)}`;
  } else {
    drawLineChart($("fftCanvas"), []);
    $("waveFftMeta").textContent = "需要至少 4 个时域点";
  }

  const rangePoints = samples.filter((s) => Number.isFinite(Number(s.distance_m)) && Number.isFinite(Number(s.amplitude)))
    .sort((a, b) => a.distance_m - b.distance_m);
  drawLineChart($("rangeCanvas"), rangePoints.map((s) => ({ x: s.distance_m, y: s.amplitude })),
    { color: "#4ade80", xLabel: "距离 m", yLabel: "振幅", scatter: rangePoints.length < 200 });
  $("waveRangeMeta").textContent = rangePoints.length ? `${rangePoints.length} 个距离门 · ${fmtNumber(rangePoints[0].distance_m)}~${fmtNumber(rangePoints[rangePoints.length - 1].distance_m)} m` : "样本缺少 distance_m 字段";

  const anglePoints = samples.filter((s) => Number.isFinite(Number(s.angle_deg)) && Number.isFinite(Number(s.amplitude)))
    .sort((a, b) => a.angle_deg - b.angle_deg);
  drawLineChart($("angleCanvas"), anglePoints.map((s) => ({ x: s.angle_deg, y: s.amplitude })),
    { color: "#fb923c", xLabel: "角度 °", yLabel: "振幅", scatter: anglePoints.length < 200 });
  $("waveAngleMeta").textContent = anglePoints.length ? `${anglePoints.length} 个角度门` : "样本缺少 angle_deg 字段";

  // 数据表
  const table = $("waveSampleTable");
  const head = `<tr><th>#</th><th class="num">t_ms</th><th class="num">振幅</th><th class="num">距离m</th><th class="num">角度°</th><th class="num">SNR dB</th><th class="num">频率Hz</th></tr>`;
  const rows = samples.slice(0, 16).map((s, i) =>
    `<tr><td>${s.index ?? i}</td><td class="num">${fmtNumber(s.t_ms, 3)}</td><td class="num">${fmtNumber(s.amplitude, 4)}</td>` +
    `<td class="num">${s.distance_m !== undefined ? fmtNumber(s.distance_m, 3) : "—"}</td>` +
    `<td class="num">${s.angle_deg !== undefined ? fmtNumber(s.angle_deg, 2) : "—"}</td>` +
    `<td class="num">${s.snr_db !== undefined ? fmtNumber(s.snr_db, 2) : "—"}</td>` +
    `<td class="num">${s.frequency_hz !== undefined ? fmtNumber(s.frequency_hz, 1) : "—"}</td></tr>`).join("");
  table.innerHTML = head + rows;
  $("waveSampleMeta").textContent = `${samples.length} 点 · 历史 ${wave.history?.length || 0} 帧 · 累计接收 ${wave.received_count || 0} 次`;
}

const DEMO_WAVE_JSON = JSON.stringify({
  source: "wave_mock_demo",
  seq: 1,
  status: "valid",
  mode: "fmcw",
  meta: { sample_rate_hz: 100000, center_frequency_hz: 10e9, bandwidth_hz: 1e9 },
  samples: Array.from({ length: 512 }, (_, i) => ({
    t_ms: i * 0.01,
    amplitude: +(0.8 * Math.sin(2 * Math.PI * 220 * i * 0.01 / 1000) + 0.45 * Math.sin(2 * Math.PI * 470 * i * 0.01 / 1000) + 0.2 * Math.sin(2 * Math.PI * 910 * i * 0.01 / 1000)).toFixed(4),
    snr_db: +(18 + 2 * Math.sin(i / 30)).toFixed(2),
  })),
}, null, 2);

function setupWavePage() {
  $("waveClearBtn").addEventListener("click", async () => {
    await fetch("/api/wave/clear", { method: "POST" });
  });
  $("waveDemoBtn").addEventListener("click", () => {
    $("waveTestJson").value = DEMO_WAVE_JSON;
  });
  $("waveTestBtn").addEventListener("click", async () => {
    const box = $("waveTestResult");
    try {
      const payload = JSON.parse($("waveTestJson").value);
      const res = await fetch("/api/wave/ingest", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const data = await res.json();
      box.textContent = data.ok ? `✅ 已接收 ${data.sample_count} 个采样点` : `❌ ${data.error || "请求失败"}`;
      box.style.color = data.ok ? "#4ade80" : "#f87171";
    } catch (err) {
      box.textContent = `❌ JSON 解析失败: ${err.message}`;
      box.style.color = "#f87171";
    }
  });
  $("waveTestJson").value = DEMO_WAVE_JSON;
}

/* ----------------------------- 运行状态页面 ----------------------------- */
function flattenConfig(cfg, prefix = "", out = []) {
  for (const [key, value] of Object.entries(cfg || {})) {
    const name = prefix ? `${prefix}.${key}` : key;
    if (value && typeof value === "object" && !Array.isArray(value)) {
      flattenConfig(value, name, out);
    } else {
      out.push([name, value]);
    }
  }
  return out;
}

function updateStatusPage() {
  if (!App.state) return;
  const node = App.state.node || {};
  const topics = App.state.topics || [];
  $("statusCards").innerHTML = `
    <div class="status-card"><div class="k">服务运行时长</div><div class="v">${fmtNumber(node.uptime, 0)}s</div><div class="s">启动 ${timeText(node.start_time)}</div></div>
    <div class="status-card"><div class="k">数据代数 / 内存</div><div class="v">${App.generation}</div><div class="s">RSS ${fmtNumber(node.memory_rss_kb / 1024, 1)} MB</div></div>
    <div class="status-card"><div class="k">订阅话题(活跃)</div><div class="v">${topics.filter((t) => t.count > 0).length}/${topics.length}</div><div class="s">SSE 客户端 ${node.sse_clients}</div></div>
    <div class="status-card"><div class="k">机器人轨迹</div><div class="v">${(App.state.robots || []).length}</div><div class="s">哨兵决策 ${(App.state.sentry || []).length} 条</div></div>
    <div class="status-card"><div class="k">解析波</div><div class="v">${App.state.wave?.available ? "已接入" : "等待接入"}</div><div class="s">${App.state.wave?.history?.length || 0} 帧历史</div></div>
    <div class="status-card"><div class="k">ROS Graph 话题</div><div class="v">${App.state.ros_graph?.topic_count || 0}</div><div class="s">${node.name} PID ${node.pid}</div></div>`;

  const topicRows = topics.sort((a, b) => (b.rate_hz || 0) - (a.rate_hz || 0));
  $("topicCount").textContent = `${topics.length} 个订阅`;
  $("topicTable").innerHTML =
    `<tr><th>话题</th><th>类型</th><th class="num">速率Hz</th><th class="num">消息数</th><th class="num">数据年龄</th><th>摘要</th></tr>` +
    topicRows.map((t) => {
      const extra = t.extra ? Object.entries(t.extra).map(([k, v]) => `${k}=${v}`).join(" ") : "";
      const ageClass = t.age > 3 ? "style='color:#f59e0b'" : "";
      return `<tr><td>${escapeHtml(t.name)}</td><td>${escapeHtml(t.type)}</td>` +
        `<td class="num">${fmtNumber(t.rate_hz, 2)}</td><td class="num">${t.count}</td>` +
        `<td class="num" ${ageClass}>${ageText(t.age)}</td><td>${escapeHtml(extra)}</td></tr>`;
    }).join("");

  const images = App.state.images || {};
  const mapImg = images.map_view;
  const detImg = images.detect_view;
  $("mapStreamState").textContent = mapImg?.available
    ? `${mapImg.width}×${mapImg.height} · ${fmtNumber(mapImg.jpeg_size / 1024, 0)}KB · seq ${mapImg.sequence}`
    : "等待 display_panel /map_view…";
  $("detectStreamState").textContent = detImg?.available
    ? `${detImg.width}×${detImg.height} · ${fmtNumber(detImg.jpeg_size / 1024, 0)}KB · seq ${detImg.sequence}`
    : "等待 detect /detect_view…";

  const cfgRows = flattenConfig(App.state.config).filter(([k]) =>
    !["project_root", "map_path", "file"].includes(k.split(".")[0]) || k === "file");
  $("configTable").innerHTML =
    `<tr><th>配置项</th><th>值</th></tr>` +
    cfgRows.slice(0, 80).map(([k, v]) =>
      `<tr><td>${escapeHtml(k)}</td><td>${escapeHtml(typeof v === "object" ? JSON.stringify(v) : v)}</td></tr>`).join("");

  const ekf = App.state.ekf_slots || [];
  $("ekfTable").innerHTML = ekf.length
    ? `<tr><th>slot</th><th>robot</th><th class="num">检测Hz</th><th class="num">新息范数</th><th class="num">协方差迹</th><th class="num">抖动</th></tr>` +
      ekf.map((s) =>
        `<tr><td>${s.slot_id}</td><td>${s.robot_id}</td><td class="num">${fmtNumber(s.detection_rate_hz, 2)}</td>` +
        `<td class="num">${fmtNumber(s.innovation_norm, 3)}</td><td class="num">${fmtNumber(s.covariance_trace, 3)}</td>` +
        `<td class="num">${fmtNumber(s.jitter, 3)}</td></tr>`).join("")
    : `<tr><td colspan="6" class="empty">当前项目尚无 EKF 诊断发布者,消息类型已预留 (detect_result/msg/EkfDiagnosticsArray)</td></tr>`;

  const logs = App.state.logs || [];
  $("logList").innerHTML = logs.slice(0, 60).map((log) => {
    const t = timeText(log.timestamp).slice(0, 8);
    return `<div class="log-row ${log.level}"><span class="t">${t}</span><span class="lv">${log.level.toUpperCase()}</span><span>${escapeHtml(log.message)}</span></div>`;
  }).join("") || `<div class="empty">暂无事件</div>`;

  const terms = ["radar", "livox", "location", "detect", "image", "sentry", "ai_nav", "wave", "map", "ekf"];
  const graph = (App.state.ros_graph?.topics || []).filter((t) =>
    terms.some((term) => t.name.toLowerCase().includes(term))).slice(0, 100);
  $("graphTable").innerHTML =
    `<tr><th>话题</th><th>类型</th></tr>` +
    (graph.length ? graph.map((t) =>
      `<tr><td>${escapeHtml(t.name)}</td><td>${escapeHtml(t.types.join(", "))}</td></tr>`).join("")
      : `<tr><td colspan="2" class="empty">ROS graph 尚未获取</td></tr>`);
}

/* ----------------------------- 页面切换 / 主循环 ----------------------------- */
function setupTabs() {
  const tabs = document.querySelectorAll(".tab");
  const pages = {
    map: $("page-map"), wave: $("page-wave"), status: $("page-status"),
  };
  function activate(page, push = true) {
    App.activePage = page;
    tabs.forEach((tab) => tab.classList.toggle("active", tab.dataset.page === page));
    Object.entries(pages).forEach(([name, el]) => { el.hidden = name !== page; });
    if (push && location.hash !== `#${page}`) history.replaceState(null, "", `#${page}`);
    if (page === "status") updateStatusPage();
  }
  tabs.forEach((tab) => tab.addEventListener("click", () => activate(tab.dataset.page)));
  const initial = (location.hash || "").replace("#", "");
  activate(["map", "wave", "status"].includes(initial) ? initial : "map", false);
  window.addEventListener("hashchange", () => {
    const page = (location.hash || "").replace("#", "");
    if (["map", "wave", "status"].includes(page)) activate(page, false);
  });
}

function loadMapImage() {
  App.mapImage = new Image();
  App.mapImage.onload = () => { App.mapImageLoaded = true; };
  App.mapImage.onerror = () => { App.mapImageLoaded = false; };
  App.mapImage.src = `/api/map.png?t=${Date.now()}`;
}

function renderActive(now) {
  if (App.activePage === "map") drawMap();
  else if (App.activePage === "wave") drawWavePage();
  else if (App.activePage === "status") updateStatusPage();
}

function loop(now) {
  if (App.activePage === "map" && now - App.lastRender > 50) {
    App.lastRender = now;
    if (App.state) drawMap();
  } else if (App.activePage === "wave" && now - App.lastRender > 100) {
    App.lastRender = now;
    if (App.state) drawWavePage();
  } else if (App.activePage === "status" && now - App.lastRender > 500) {
    App.lastRender = now;
    if (App.state) updateStatusPage();
  }
  requestAnimationFrame(loop);
}

function clockTick() {
  $("clockBadge").textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false });
}

window.addEventListener("DOMContentLoaded", () => {
  setupTabs();
  buildDebugControls();
  setupMapInteraction();
  setupWavePage();
  loadMapImage();
  loadDebugOnce();
  loadStateOnce();
  connectEvents();
  setInterval(clockTick, 1000);
  clockTick();
  requestAnimationFrame(loop);
});
