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
  if (!confirm("¿Cargar los 94 valores originales en el formulario? Aún no se aplicarán al robot.")) return;
  state.values = structuredClone(state.factory);
  state.group = "Bloqueo reactivo";
  renderTabs();
  renderForm();
  updateDirty();
  document.querySelector(".controls").scrollIntoView({behavior: "smooth", block: "start"});
  toast("Valores originales cargados. Revisa y pulsa ‘Aplicar y guardar’ para completar la restauración.");
}

function loadFastPredictorProfile() {
  Object.assign(state.values, {
    "goalkeeper.prediction.enabled": true,
    "goalkeeper.prediction.require_localization": true,
    "goalkeeper.prediction.history_msec": 600,
    "goalkeeper.prediction.max_samples": 20,
    "goalkeeper.prediction.min_samples": 5,
    "goalkeeper.prediction.min_span_msec": 100,
    "goalkeeper.prediction.activation_hold_msec": 400,
    "goalkeeper.prediction.post_block_claim_msec": 2500
  });
  state.group = "Predicción";
  renderTabs();
  renderForm();
  updateDirty();
  toast("Perfil rápido cargado. Revisa y pulsa ‘Aplicar y guardar’; todavía no se envió al robot.");
}

function statusView(status) {
  const online = Boolean(status.connected);
  $("connection").textContent = online ? "Brain conectado" : "Brain sin conexión";
  $("connection").className = `pill ${online ? "online" : "offline"}`;
  $("decision").textContent = status.decision || "—";
  $("kickType").textContent = status.kick_type || "—";
  $("predictionState").textContent = !status.prediction_enabled ? "DESACTIVADA · NO BLOQUEARÁ" : status.threatens_goal ? "TIRO DETECTADO" : status.prediction_valid ? "trayectoria válida" : "esperando muestras";
  $("predictionState").classList.toggle("critical", !status.prediction_enabled);
  $("timeToIntercept").textContent = status.threatens_goal ? `${format(status.time_to_intercept)} s` : "—";
  $("gameState").textContent = status.game_state || "?";
  $("localizationState").textContent = status.localization_ready ? "lista" : status.localization_required ? "no calibrada" : "no requerida";
  $("predictionReason").textContent = reasonLabel(status.prediction_reason);
  $("ballDetected").textContent = status.ball_detected ? "detectado" : "no detectado";
  const fitted = Boolean(status.fit_computed);
  $("speed").textContent = fitted ? `${format(status.speed)} m/s` : "—";
  $("velocity").textContent = fitted ? `(${format(status.velocity_x)}, ${format(status.velocity_y)})` : "—";
  $("rSquared").textContent = fitted ? format(status.r_squared, 3) : "—";
  $("rSquaredX").textContent = fitted ? format(status.r_squared_x, 3) : "—";
  $("rSquaredY").textContent = fitted ? format(status.r_squared_y, 3) : "—";
  $("residual").textContent = fitted ? `${format(status.residual, 3)} m` : "—";
  $("sampleCount").textContent = Number.isFinite(Number(status.sample_count)) ? String(status.sample_count) : "—";
  $("sampleSpan").textContent = `${format(status.sample_span_msec, 0)} ms`;
  $("observationAge").textContent = `${format(status.observation_age_msec, 0)} ms`;
  $("detectionLatency").textContent = `${format(status.estimated_detection_latency_msec, 0)} ms`;
  $("interceptPoint").textContent = status.threatens_goal ? `(${format(status.intercept_x)}, ${format(status.intercept_y)}) m` : "—";
  $("ballConfidence").textContent = `${format(status.ball_confidence, 1)} %`;
  $("robotPose").textContent = `(${format(status.robot_x)}, ${format(status.robot_y)}, ${format(status.robot_theta)})`;
  $("postBlockClearance").textContent = status.post_block_clearance ? "activo: buscar y patear" : "inactivo";
  renderField2D(status);
}

function svgPoint(element, x, y, visible = true) {
  element.setAttribute("cx", x);
  element.setAttribute("cy", y);
  element.classList.toggle("hidden", !visible);
}

function renderField2D(status) {
  const length = Number(status.field_length) || 14;
  const width = Number(status.field_width) || 9;
  const goalWidth = Number(status.goal_width) || 2.6;
  const left = 55, top = 25, drawWidth = 700, drawHeight = 450;
  const px = x => left + (Number(x) + length / 2) / length * drawWidth;
  const py = y => top + (width / 2 - Number(y)) / width * drawHeight;
  const validPair = pair => Array.isArray(pair) && pair.length >= 2 && pair.every(value => Number.isFinite(Number(value)));

  const bounds = $("fieldBounds");
  bounds.setAttribute("x", left); bounds.setAttribute("y", top);
  bounds.setAttribute("width", drawWidth); bounds.setAttribute("height", drawHeight);
  const goalX = px(-length / 2);
  $("goalShape").setAttribute("d", `M ${goalX} ${py(goalWidth / 2)} L ${goalX - 28} ${py(goalWidth / 2)} L ${goalX - 28} ${py(-goalWidth / 2)} L ${goalX} ${py(-goalWidth / 2)}`);
  const blockX = px(Number(status.block_line_x));
  $("blockLine").setAttribute("x1", blockX); $("blockLine").setAttribute("x2", blockX);
  $("blockLine").setAttribute("y1", py(goalWidth / 2 + .4)); $("blockLine").setAttribute("y2", py(-goalWidth / 2 - .4));

  const observations = (status.observations || []).filter(validPair);
  $("observationTrail").setAttribute("points", observations.map(p => `${px(p[0])},${py(p[1])}`).join(" "));
  const trajectory = (status.trajectory || []).filter(validPair);
  $("trajectoryLine").setAttribute("points", trajectory.map(p => `${px(p[0])},${py(p[1])}`).join(" "));

  const ballVisible = Boolean(status.ball_detected) && Number.isFinite(Number(status.ball_x)) && Number.isFinite(Number(status.ball_y));
  svgPoint($("ballMarker"), px(status.ball_x), py(status.ball_y), ballVisible);
  const robotVisible = Number.isFinite(Number(status.robot_x)) && Number.isFinite(Number(status.robot_y));
  const robot = $("robotMarker");
  robot.setAttribute("transform", `translate(${px(status.robot_x)} ${py(status.robot_y)}) rotate(${-Number(status.robot_theta || 0) * 180 / Math.PI})`);
  robot.classList.toggle("hidden", !robotVisible);
  const interceptVisible = Boolean(status.threatens_goal) && Number.isFinite(Number(status.intercept_x)) && Number.isFinite(Number(status.intercept_y));
  svgPoint($("interceptMarker"), px(status.intercept_x), py(status.intercept_y), interceptVisible);
  $("field2d").classList.toggle("threat", Boolean(status.threatens_goal));
}

const REASONS = {
  initializing: "inicializando", disabled: "predictor desactivado",
  localization_required: "localización requerida",
  insufficient_samples: "faltan muestras",
  insufficient_span: "intervalo de muestras insuficiente",
  invalid_observation: "observación inválida",
  speed_below_minimum: "velocidad menor al umbral",
  r_squared_below_minimum: "R² menor al umbral",
  residual_above_maximum: "residual demasiado alto",
  not_toward_own_goal: "el balón no va hacia nuestra portería",
  already_past_block_line: "el balón ya cruzó la línea de bloqueo",
  stops_before_block_line: "se detendría antes de la línea",
  time_outside_window: "tiempo de llegada fuera de ventana",
  outside_goal: "trayectoria fuera de la portería",
  threat_detected: "TIRO DETECTADO"
};

function reasonLabel(reason) { return REASONS[reason] || reason || "—"; }

async function updateTelemetry() {
  try {
    const result = await api("/api/telemetry?limit=40");
    $("logPath").textContent = `Registro: ${result.log_file}`;
    const rows = $("telemetryRows");
    rows.innerHTML = "";
    [...result.events].reverse().forEach(event => {
      const row = document.createElement("tr");
      const stamp = event.received_at_iso ? new Date(event.received_at_iso).toLocaleTimeString() : "—";
      [stamp, event.decision || "—", reasonLabel(event.prediction_reason),
       event.sample_count ?? "—", format(event.speed), format(event.r_squared, 3),
       format(event.residual, 3)].forEach(value => {
        const cell = document.createElement("td");
        cell.textContent = value;
        row.appendChild(cell);
      });
      rows.appendChild(row);
    });
  } catch {}
}

async function poll() {
  try { statusView(await api("/api/status")); }
  catch { statusView({connected: false}); }
  await updateTelemetry();
  setTimeout(poll, 250);
}

$("reload").onclick = load;
$("apply").onclick = () => apply(false);
$("save").onclick = () => apply(true);
$("restoreFactory").onclick = loadFactoryDefaults;
$("fastProfile").onclick = loadFastPredictorProfile;
$("helpLink").onclick = () => { $("help").open = true; };
document.querySelectorAll(".go-group").forEach(button => {
  button.onclick = () => selectGroup(button.dataset.group);
});
if (location.hash === "#help") $("help").open = true;
load();
poll();
