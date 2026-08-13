const state = {schema: null, values: {}, baseline: {}, factory: {}, group: null};
const $ = id => document.getElementById(id);

function toast(message, error = false) {
  const el = $("toast");
  el.textContent = message;
  el.className = error ? "show error" : "show";
  clearTimeout(el.timer);
  el.timer = setTimeout(() => el.className = "", 4500);
}

async function api(url, options) {
  const response = await fetch(url, options);
  const body = await response.json();
  if (!response.ok) throw new Error(body.error || `HTTP ${response.status}`);
  return body;
}

function format(value, digits = 2) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "—";
}

function displayValue(value, spec) {
  if (spec.type === "boolean") return value ? "Activado" : "Desactivado";
  return `${value}${spec.unit ? ` ${spec.unit}` : ""}`;
}

function movementEffect(name) {
  if (name === "goalkeeper.mode") return "selecciona la rama attack/guard; no genera movimiento por sí solo";
  if (name.startsWith("goalkeeper.ready.")) return "locomoción del cuerpo al colocarse en READY";
  if (name.startsWith("goalkeeper.blocking.")) return "locomoción del cuerpo al cubrir la portería";
  if (name.startsWith("goalkeeper.chase.") || name.includes("during_chase") || name.includes("chase_ao")) return "locomoción y trayectoria al perseguir el balón";
  if (name.startsWith("goalkeeper.adjust.")) return "pasos de alineación y giro antes del despeje";
  if (name.startsWith("goalkeeper.claim.")) return "decisión de abandonar la cobertura; después puede activar persecución";
  if (name.startsWith("goalkeeper.camera.")) return "movimiento de la cabeza/cámara, no de las piernas";
  if (name.startsWith("goalkeeper.prediction.block.")) return "locomoción del cuerpo durante block_shot";
  if (name.startsWith("goalkeeper.prediction.")) return "cálculo y decisión del predictor; sólo mueve si termina activando block_shot";
  if (name.includes("fallen_robot") || name.startsWith("obstacle_avoidance.")) return "protección de trayectoria o salida de la acción de patada";
  if (name.startsWith("goalkeeper.kick.") || name.startsWith("RLVisionKick.")) return "selección, preparación o ejecución de la patada";
  return "comportamiento del portero";
}

function updateDirty() {
  const dirty = JSON.stringify(state.values) !== JSON.stringify(state.baseline);
  $("dirtyLabel").textContent = dirty ? "Cambios pendientes (aún no aplicados)" : "Sin cambios pendientes";
}

function selectGroup(group) {
  state.group = group;
  renderTabs();
  renderForm();
  document.querySelector(".controls").scrollIntoView({behavior: "smooth", block: "start"});
}

function renderTabs() {
  const tabs = $("tabs");
  tabs.innerHTML = "";
  state.schema.groups.forEach(group => {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = group;
    button.className = group === state.group ? "active" : "";
    button.onclick = () => selectGroup(group);
    tabs.appendChild(button);
  });
}

function renderForm() {
  const form = $("parameters");
  form.innerHTML = "";
  Object.entries(state.schema.parameters)
    .filter(([, spec]) => spec.group === state.group)
    .forEach(([name, spec]) => {
      const wrap = document.createElement("div");
      wrap.className = "control";
      const head = document.createElement("header");
      const label = document.createElement("label");
      label.textContent = spec.label;
      const unit = document.createElement("small");
      unit.textContent = spec.unit || "";
      head.append(label, unit);
      const code = document.createElement("code");
      code.textContent = name;
      const desc = document.createElement("p");
      desc.textContent = spec.description;
      let input;
      if (spec.type === "boolean") {
        input = document.createElement("input");
        input.type = "checkbox";
        input.checked = Boolean(state.values[name]);
        const line = document.createElement("div");
        line.className = "switch";
        line.append(input, document.createTextNode(input.checked ? "Activado" : "Desactivado"));
        input.onchange = () => {
          state.values[name] = input.checked;
          line.lastChild.textContent = input.checked ? "Activado" : "Desactivado";
          updateDirty();
        };
        wrap.append(head, code, desc, line);
      } else if (spec.type === "choice") {
        input = document.createElement("select");
        spec.options.forEach(option => {
          const item = document.createElement("option");
          item.value = option;
          item.textContent = option;
          input.appendChild(item);
        });
        input.value = state.values[name];
        input.onchange = () => { state.values[name] = input.value; updateDirty(); };
        wrap.append(head, code, desc, input);
      } else {
        input = document.createElement("input");
        input.type = "number";
        input.min = spec.minimum;
        input.max = spec.maximum;
        input.step = spec.step;
        input.value = state.values[name];
        input.oninput = () => {
          state.values[name] = spec.type === "integer" ? Number.parseInt(input.value, 10) : Number(input.value);
          updateDirty();
        };
        wrap.append(head, code, desc, input);
      }
      form.appendChild(wrap);
    });
}

function renderParameterHelp() {
  const target = $("parameterHelp");
  target.innerHTML = "";
  state.schema.groups.forEach(group => {
    const section = document.createElement("section");
    const heading = document.createElement("h3");
    heading.textContent = group;
    section.appendChild(heading);
    const table = document.createElement("div");
    table.className = "help-table";
    Object.entries(state.schema.parameters)
      .filter(([, spec]) => spec.group === group)
      .forEach(([name, spec]) => {
        const row = document.createElement("article");
        const title = document.createElement("div");
        title.innerHTML = `<strong>${spec.label}</strong><code>${name}</code>`;
        const explanation = document.createElement("p");
        explanation.textContent = spec.description;
        const meta = document.createElement("small");
        const original = displayValue(state.factory[name], spec);
        const range = spec.type === "number" || spec.type === "integer"
          ? `${spec.minimum}–${spec.maximum}${spec.unit ? ` ${spec.unit}` : ""}`
          : spec.type === "choice" ? spec.options.join(" / ") : "Activado / Desactivado";
        meta.textContent = `Original: ${original} · Opciones/rango: ${range} · Afecta: ${movementEffect(name)}.`;
        row.append(title, explanation, meta);
        table.appendChild(row);
      });
    section.appendChild(table);
    target.appendChild(section);
  });
}

async function load() {
  try {
    const [schema, config, factory] = await Promise.all([
      api("/api/schema"), api("/api/config"), api("/api/factory-defaults")
    ]);
    state.schema = schema;
    state.values = config.values;
    state.baseline = structuredClone(config.values);
    state.factory = factory.values;
    state.group = schema.groups[0];
    renderTabs();
    renderForm();
    renderParameterHelp();
    updateDirty();
  } catch (error) {
    toast(error.message, true);
  }
}

async function apply(persist) {
  const buttons = [...document.querySelectorAll(".actions button")];
  buttons.forEach(button => button.disabled = true);
  try {
    await api("/api/apply", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({values: state.values, persist})
    });
    state.baseline = structuredClone(state.values);
    updateDirty();
    toast(persist ? "Parámetros aplicados y guardados para próximos arranques" : "Parámetros aplicados sólo al proceso actual");
  } catch (error) {
    toast(error.message, true);
  } finally {
    buttons.forEach(button => button.disabled = false);
  }
}

function loadFactoryDefaults() {
  if (!confirm("¿Cargar los 93 valores originales en el formulario? Aún no se aplicarán al robot.")) return;
  state.values = structuredClone(state.factory);
  state.group = "Bloqueo reactivo";
  renderTabs();
  renderForm();
  updateDirty();
  document.querySelector(".controls").scrollIntoView({behavior: "smooth", block: "start"});
  toast("Valores originales cargados. Revisa y pulsa ‘Aplicar y guardar’ para completar la restauración.");
}

function statusView(status) {
  const online = Boolean(status.connected);
  $("connection").textContent = online ? "Brain conectado" : "Brain sin conexión";
  $("connection").className = `pill ${online ? "online" : "offline"}`;
  $("decision").textContent = status.decision || "—";
  $("kickType").textContent = status.kick_type || "—";
  $("predictionState").textContent = !status.prediction_enabled ? "desactivada" : status.threatens_goal ? "TIRO DETECTADO" : status.prediction_valid ? "trayectoria válida" : "esperando muestras";
  $("timeToIntercept").textContent = status.threatens_goal ? `${format(status.time_to_intercept)} s` : "—";
  $("gameState").textContent = status.game_state || "?";
  $("localizationState").textContent = status.localization_ready ? "lista" : status.localization_required ? "no calibrada" : "no requerida";
  $("ballDetected").textContent = status.ball_detected ? "detectado" : "no detectado";
  $("speed").textContent = `${format(status.speed)} m/s`;
  $("velocity").textContent = `(${format(status.velocity_x)}, ${format(status.velocity_y)})`;
  $("rSquared").textContent = format(status.r_squared, 3);
  $("residual").textContent = `${format(status.residual, 3)} m`;
  $("sampleCount").textContent = Number.isFinite(Number(status.sample_count)) ? String(status.sample_count) : "—";
  $("interceptPoint").textContent = status.threatens_goal ? `(${format(status.intercept_x)}, ${format(status.intercept_y)}) m` : "—";
  const marker = $("intercept");
  marker.classList.toggle("active", Boolean(status.threatens_goal));
  const normalized = Math.max(-1.5, Math.min(1.5, Number(status.intercept_y) || 0));
  marker.style.left = `${50 + normalized / 3 * 76}%`;
}

async function poll() {
  try { statusView(await api("/api/status")); }
  catch { statusView({connected: false}); }
  setTimeout(poll, 250);
}

$("reload").onclick = load;
$("apply").onclick = () => apply(false);
$("save").onclick = () => apply(true);
$("restoreFactory").onclick = loadFactoryDefaults;
$("helpLink").onclick = () => { $("help").open = true; };
document.querySelectorAll(".go-group").forEach(button => {
  button.onclick = () => selectGroup(button.dataset.group);
});
if (location.hash === "#help") $("help").open = true;
load();
poll();
