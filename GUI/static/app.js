"use strict";

const $ = (id) => document.getElementById(id);

async function post(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  let data = {};
  try { data = await res.json(); } catch (e) {}
  if (res.status === 409) flashBanner("BUSY: " + (data.operation || "another op"), "busy");
  else if (!res.ok) flashBanner("ERROR: " + (data.error || res.status), "error");
  return data;
}

async function getJSON(url) {
  const res = await fetch(url);
  return res.json();
}

function flashBanner(text, cls) {
  const b = $("op-banner");
  b.textContent = text;
  b.className = "op-banner " + (cls || "idle");
}

function fmtArr(a, n = 1) {
  if (!a || !a.length) return "—";
  return a.map((v) => Number(v).toFixed(n)).join(", ");
}

// ---- status polling --------------------------------------------------------

async function pollStatus() {
  try {
    const s = await getJSON("/api/status");
    renderExecutor(s.executor);
    renderRobot(s.robot);
    renderMachine(s.machine);
    renderRecipeState(s.recipe);
  } catch (e) {
    flashBanner("status unreachable", "error");
  }
}

function renderExecutor(ex) {
  if (ex.last_error) flashBanner("last error: " + ex.last_error, "error");
  else if (ex.busy) flashBanner("BUSY: " + ex.operation, "busy");
  else flashBanner("idle", "idle");
}

function renderRobot(r) {
  const st = $("robot-state");
  if (r.fault) { st.textContent = "FAULT"; st.className = "pill fault"; }
  else if (r.operational) { st.textContent = "operational"; st.className = "pill ok"; }
  else { st.textContent = "not operational"; st.className = "pill"; }

  $("joints").textContent = fmtArr(r.joint_angles_deg, 1);
  $("tcp-pos").textContent = fmtArr(r.tcp_position_m, 3);
  $("tcp-rpy").textContent = fmtArr(r.tcp_orientation_deg, 1);
  const g = r.gripper || {};
  $("gripper").textContent =
    g.width_m == null ? "—" :
    `w=${(g.width_m * 1000).toFixed(1)}mm  f=${g.force_n.toFixed(1)}N  ${g.is_moving ? "moving" : "still"}`;
  const fl = r.floating || {};
  $("floating-state").textContent = fl.mode
    ? `${fl.mode} [${(fl.selection || []).join(",") || "all"}]`
    : "off";
  const w = r.wrench || {};
  $("wrench").textContent = w.fx == null ? "—" :
    `F ${w.fx.toFixed(1)},${w.fy.toFixed(1)},${w.fz.toFixed(1)}`;
}

function renderMachine(m) {
  const st = $("machine-state");
  if (!m.connected) { st.textContent = "offline"; st.className = "pill"; }
  else if (m.faulted) { st.textContent = "FAULT"; st.className = "pill fault"; }
  else if (m.homed) { st.textContent = "operational"; st.className = "pill ok"; }
  else { st.textContent = "not homed"; st.className = "pill"; }

  const fx = (v, n = 1) => (v == null ? "—" : Number(v).toFixed(n));
  $("m-x").textContent = fx(m.x_mm);
  $("m-z").textContent = fx(m.z_mm);
  $("m-rot").textContent = fx(m.rot_deg);
  $("m-vise").textContent = m.vise_state || "—";
  $("m-force").textContent = fx(m.force_kg, 2);
  $("m-blade").textContent = m.blade_on == null ? "—" : (m.blade_on ? "ON" : "off");
  const flags = [];
  if (m.homed) flags.push("homed");
  if (m.busy) flags.push("busy");
  if (m.faulted) flags.push("FAULTED");
  if (m.active_command) flags.push("→ " + m.active_command);
  $("m-state").textContent = flags.join("  ") || "—";
}

// ---- recipe log polling ----------------------------------------------------

let lastLogSeq = 0;
async function pollLog() {
  try {
    const { records } = await getJSON("/api/recipe/log?after=" + lastLogSeq);
    if (records && records.length) {
      const pre = $("recipe-log");
      for (const rec of records) {
        lastLogSeq = rec.seq;
        const div = document.createElement("div");
        div.className = "log-" + rec.level;
        div.textContent = rec.message;
        pre.appendChild(div);
      }
      pre.scrollTop = pre.scrollHeight;
    }
  } catch (e) {}
}

// ---- recipe panel ----------------------------------------------------------

let currentRecipe = null;

async function loadRecipeList() {
  const { recipes } = await getJSON("/api/recipes");
  const sel = $("recipe-select");
  sel.innerHTML = "";
  for (const name of recipes) {
    const o = document.createElement("option");
    o.value = o.textContent = name;
    sel.appendChild(o);
  }
}

async function loadRecipe() {
  const name = $("recipe-select").value;
  if (!name) return;
  const detail = await getJSON("/api/recipes/" + encodeURIComponent(name));
  if (detail.error) { flashBanner(detail.error, "error"); return; }
  currentRecipe = detail;
  renderRecipe(detail);
}

// Live highlight, driven by /api/status (works for both Run Full and Next Phase).
function renderRecipeState(r) {
  if (!r || !currentRecipe || r.name !== currentRecipe.name) return;
  updatePhaseHighlight(r.phase_index);
}

function updatePhaseHighlight(index) {
  document.querySelectorAll("#recipe-phases li").forEach((li, i) => {
    li.classList.toggle("current", i === index);
  });
}

function renderRecipe(d) {
  const phases = $("recipe-phases");
  phases.innerHTML = "";
  (d.phases || []).forEach((p, i) => {
    const li = document.createElement("li");
    li.textContent = p;
    if (i === d.phase_index) li.className = "current";
    phases.appendChild(li);
  });
}

// ---- waypoints -------------------------------------------------------------

async function loadWaypoints() {
  const { waypoints } = await getJSON("/api/waypoints");
  const sel = $("waypoint-select");
  sel.innerHTML = "";
  for (const w of waypoints) {
    const o = document.createElement("option");
    o.value = o.textContent = w;
    sel.appendChild(o);
  }
}

// ---- action dispatch -------------------------------------------------------

function speed() { return parseFloat($("speed").value); }

const ACTIONS = {
  "robot-home": () => post("/api/robot/home"),
  "robot-move": () => post("/api/robot/move_to", { name: $("waypoint-select").value }),
  "robot-zero-ft": () => post("/api/robot/zero_ft"),
  "robot-clear-fault": () => post("/api/robot/clear_fault"),
  "gripper-open": () => post("/api/robot/gripper", { action: "open" }),
  "gripper-close": () => post("/api/robot/gripper", { action: "close" }),
  "float-cart-on": () => post("/api/robot/floating", { mode: "cartesian", on: true }),
  "float-joint-on": () => post("/api/robot/floating", { mode: "joint", on: true }),
  "float-off": () => post("/api/robot/floating", { on: false }),

  "machine-home": () => post("/api/machine/home"),
  "vise-close": () => post("/api/machine/vise", { action: "close" }),
  "vise-open": () => post("/api/machine/vise", { action: "open" }),
  "move-x": () => post("/api/machine/move_x", { x_mm: parseFloat($("x-mm").value) }),
  "move-z": () => post("/api/machine/move_z", { z_mm: parseFloat($("z-mm").value) }),
  "rotate": () => post("/api/machine/rotate", { deg: parseFloat($("rot-deg").value) }),

  "blade-on": () => post("/api/machine/blade", { on: true }),
  "blade-off": () => post("/api/machine/blade", { on: false }),
  "machine-clear": () => post("/api/machine/clear"),
  "cut": () => {
    if (!confirm("Run cut sequence? Ensure the part is clamped and area is clear.")) return;
    return post("/api/machine/cut", {
      z_mm: parseFloat($("cut-z").value),
      x_mm: parseFloat($("cut-x").value),
      deg: parseFloat($("cut-deg").value),
    });
  },

  "recipe-load": () => loadRecipe(),
  "recipe-run": async () => {
    if (!confirm("Run the full recipe? The cell will move autonomously.")) return;
    await post("/api/recipe/run", { speed: speed(), loops: parseInt($("loops").value) || 1 });
  },
  "recipe-step": () => post("/api/recipe/step", { speed: speed() }),
  "recipe-stop": () => post("/api/recipe/stop"),
};

function doJog(axis, dir) {
  const isRot = axis.startsWith("r");
  const step = parseFloat($(isRot ? "jog-deg" : "jog-mm").value);
  if (!isFinite(step)) return;
  post("/api/robot/jog", { axis, delta: dir * step });
}

document.addEventListener("click", (e) => {
  const btn = e.target.closest("[data-act]");
  if (!btn) return;
  if (btn.dataset.act === "jog") {
    doJog(btn.dataset.axis, parseInt(btn.dataset.dir, 10));
    return;
  }
  const fn = ACTIONS[btn.dataset.act];
  if (fn) fn();
});

$("estop").addEventListener("click", async () => {
  await post("/api/robot/stop");
  await post("/api/machine/stop");
});

$("speed").addEventListener("input", () => {
  $("speed-val").textContent = speed().toFixed(1) + "×";
});

// ---- boot ------------------------------------------------------------------

loadWaypoints();
loadRecipeList();
setInterval(pollStatus, 333);
setInterval(pollLog, 500);
pollStatus();
