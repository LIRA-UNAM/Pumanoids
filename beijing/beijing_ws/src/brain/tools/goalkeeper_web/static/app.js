const state = {schema: null, values: {}, baseline: {}, factory: {}, recommended: {}, group: null};
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

function formatMs(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) && numeric >= 0 ? `${numeric.toFixed(0)} ms` : "—";
}

function motionVector(x, y, theta) {
  return `(${format(x)}, ${format(y)}, ${format(theta)})`;
}

const REACTION_STAGES = {
  idle: "sin decisión de movimiento",
  waiting_command: "decidió · esperando comando",
  waiting_motion: "comando enviado · esperando movimiento",
  moving: "movimiento confirmado",
  stopped: "movimiento finalizado",
  command_stopped_before_motion: "ALERTA · comando terminó sin movimiento"
};

function reactionStageLabel(stage) {
  return REACTION_STAGES[stage] || stage || "—";
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
  if (name.startsWith("goalkeeper.prediction.intercept.")) return "selección del punto adelantado y velocidad de salida frontal/diagonal";
  if (name.startsWith("goalkeeper.prediction.block.")) return "locomoción del cuerpo durante block_shot";
  if (name.startsWith("goalkeeper.prediction.")) return "cálculo y decisión del predictor; sólo mueve si termina activando block_shot";
  if (name.includes("fallen_robot") || name.startsWith("obstacle_avoidance.")) return "protección de trayectoria o salida de la acción de patada";
  if (name.startsWith("goalkeeper.kick.") || name.startsWith("RLVisionKick.")) return "selección, preparación o ejecución de la patada";
  return "comportamiento del portero";
}

function tuningEffect(name, spec) {
  if (spec.type === "boolean") return {
    up: `Activar: ${spec.description}`,
    down: "Desactivar: conserva el comportamiento alternativo u original indicado."
  };
  const exact = {
    "goalkeeper.chase.target_distance": ["se aproxima desde más lejos detrás del balón", "se acerca más al balón antes de entregar a Adjust"],
    "goalkeeper.chase.safe_distance": ["rodea con más margen y recorre una trayectoria mayor", "rodea más cerca y recorta camino, con menos margen"],
    "goalkeeper.adjust.range": ["mantiene el balón más lejos al preparar la patada", "se acerca más al balón; exige mejor control para no pasarlo entre los pies"],
    "goalkeeper.prediction.max_sample_jump": ["tolera cambios mayores de posición", "rechaza antes los saltos entre candidatos"],
    "goalkeeper.prediction.recency_weight": ["responde más a las muestras nuevas, pero amplifica ruido reciente", "suaviza más usando el historial"],
    "goalkeeper.prediction.deceleration": ["supone que el balón frena más y predice más tiempo de llegada", "supone que conserva más velocidad"],
    "goalkeeper.prediction.intercept.max_forward_distance": ["busca cortar antes y más lejos de la portería", "mantiene la intercepción más cerca de la línea defensiva"],
    "goalkeeper.prediction.intercept.front_max_forward_distance": ["sale más adelante ante tiros centrados", "espera más cerca de la portería"],
    "goalkeeper.prediction.intercept.front_min_forward_distance": ["fuerza una salida frontal mínima más larga cuando el cálculo conservador no encuentra solución", "reduce la salida frontal de emergencia"],
    "goalkeeper.prediction.intercept.front_lateral_threshold": ["clasifica más tiros como frontales y usa salida directa", "reserva la salida frontal para tiros más centrados"],
    "goalkeeper.prediction.intercept.robot_speed_min": ["supone mayor capacidad antes de medirla; puede elegir puntos exigentes", "elige puntos iniciales más conservadores"],
    "goalkeeper.prediction.intercept.robot_speed_max": ["permite que la velocidad medida adelante más el objetivo", "limita el optimismo del cálculo de alcance"],
    "goalkeeper.prediction.intercept.measured_speed_gain": ["confía más en la velocidad medida y adelanta el objetivo", "reduce el alcance estimado y acerca el objetivo a portería"],
    "goalkeeper.prediction.intercept.safety_time_sec": ["reserva más tiempo y elige puntos más conservadores", "intercepta más adelante, con mayor riesgo de llegar tarde"],
    "goalkeeper.prediction.intercept.search_step": ["calcula menos candidatos y el punto queda menos preciso", "afina el punto con más candidatos"],
    "goalkeeper.prediction.intercept.min_ball_separation": ["mantiene el objetivo más detrás del balón", "permite cortar más cerca del balón"],
  };
  if (exact[name]) return {up: `Aumentar: ${exact[name][0]}.`, down: `Disminuir: ${exact[name][1]}.`};
  if (name.includes("time") || name.includes("msec") || name.includes("history") || name.includes("interval"))
    return {up: "Aumentar: da más tiempo/estabilidad, normalmente con más demora.", down: "Disminuir: reacciona antes, pero tolera menos retrasos o ruido."};
  if (name.includes("tolerance") || name.includes("margin") || name.includes("range") || name.includes("distance"))
    return {up: "Aumentar: amplía la zona, distancia o error aceptado.", down: "Disminuir: exige una condición más cercana o precisa."};
  if (name.includes("threshold") || name.includes("min_"))
    return {up: "Aumentar: vuelve la condición de activación más exigente.", down: "Disminuir: permite activarla antes o con evidencia menor."};
  if (name.includes("limit") || name.includes("rate") || name.includes("gain"))
    return {up: "Aumentar: permite una respuesta más rápida o agresiva.", down: "Disminuir: suaviza y limita el movimiento o corrección."};
  if (name.includes("max_") || name.includes("count"))
    return {up: "Aumentar: admite o conserva más datos/alcance.", down: "Disminuir: restringe el cálculo y reduce su alcance."};
  return {up: "Aumentar: incrementa la influencia de este parámetro.", down: "Disminuir: reduce su influencia."};
}

function tuningGuide(name, spec) {
  const effect = tuningEffect(name, spec);
  const guide = document.createElement("div");
  guide.className = "tuning-guide";
  guide.innerHTML = `<span class="increase-effect">↑ ${effect.up}</span><span class="decrease-effect">↓ ${effect.down}</span>`;
  return guide;
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
        wrap.append(head, code, desc, tuningGuide(name, spec), line);
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
        wrap.append(head, code, desc, tuningGuide(name, spec), input);
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
        wrap.append(head, code, desc, tuningGuide(name, spec), input);
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
        row.append(title, explanation, tuningGuide(name, spec), meta);
        table.appendChild(row);
      });
    section.appendChild(table);
    target.appendChild(section);
  });
}

async function load() {
  try {
    const [schema, config, factory, recommended] = await Promise.all([
      api("/api/schema"), api("/api/config"), api("/api/factory-defaults"),
      api("/api/recommended-profile")
    ]);
    state.schema = schema;
    state.values = config.values;
    state.baseline = structuredClone(config.values);
    state.factory = factory.values;
    state.recommended = recommended.values;
    state.group = schema.groups[0];
    renderTabs();
    renderForm();
    renderParameterHelp();
    updateDirty();
    if (config.live === false) {
      toast("brain_node detenido: mostrando valores guardados. Use Guardar para el próximo arranque.", true);
    }
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
  if (!confirm(`¿Cargar los ${Object.keys(state.factory).length} valores originales en el formulario? Aún no se aplicarán al robot.`)) return;
  state.values = structuredClone(state.factory);
  state.group = state.schema.groups[0];
  renderTabs();
  renderForm();
  updateDirty();
  document.querySelector(".controls").scrollIntoView({behavior: "smooth", block: "start"});
  toast("Valores originales cargados. Revisa y pulsa ‘Aplicar y guardar’ para completar la restauración.");
}

function loadRecommendedProfile() {
  if (!confirm("¿Cargar el perfil recomendado medido en el formulario? Aún no se aplicará al robot.")) return;
  state.values = structuredClone(state.recommended);
  state.group = "Intercepción adelantada";
  renderTabs();
  renderForm();
  updateDirty();
  toast("Perfil recomendado cargado. Revisa y pulsa ‘Aplicar y guardar’; todavía no se envió al robot.");
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
  $("score").textContent = Number.isFinite(Number(status.own_score)) && Number.isFinite(Number(status.opponent_score))
    ? `${status.own_score} - ${status.opponent_score}` : "—";
  $("localizationState").textContent = status.localization_ready ? "lista" : status.localization_required ? "no calibrada" : "no requerida";
  $("predictionReason").textContent = reasonLabel(status.prediction_reason);
  $("ballDetected").textContent = status.ball_detected ? "detectado" : "no detectado";
  $("reactionStage").textContent = reactionStageLabel(status.reaction_stage);
  $("reactionStage").classList.toggle("critical", status.reaction_stage === "command_stopped_before_motion");
  $("ballMeasurementAge").textContent = formatMs(status.ball_measurement_age_msec);
  $("decisionAge").textContent = formatMs(status.decision_age_msec);
  $("decisionInputAge").textContent = formatMs(status.decision_input_age_msec);
  $("decisionToCommand").textContent = formatMs(status.decision_to_command_msec);
  $("commandToMotion").textContent = formatMs(status.command_to_motion_msec);
  $("decisionToMotion").textContent = formatMs(status.decision_to_motion_msec);
  $("commandToAlignedMotion").textContent = formatMs(status.command_to_aligned_motion_msec);
  $("decisionToAlignedMotion").textContent = formatMs(status.decision_to_aligned_motion_msec);
  $("commandRequested").textContent = motionVector(status.command_requested_x, status.command_requested_y, status.command_requested_theta);
  $("commandSent").textContent = motionVector(status.command_sent_x, status.command_sent_y, status.command_sent_theta);
  $("commandAge").textContent = formatMs(status.command_age_msec);
  $("odomVelocity").textContent = motionVector(status.odom_velocity_x, status.odom_velocity_y, status.odom_velocity_theta);
  $("odomSpeed").textContent = Number.isFinite(Number(status.odom_speed)) ? `${format(status.odom_speed)} m/s` : "—";
  $("ballKnown").textContent = typeof status.ball_location_known === "boolean"
    ? status.ball_location_known ? status.ball_detected ? "visible ahora" : "posición recordada" : "desconocida"
    : "—";
  $("claimState").textContent = typeof status.goalkeeper_may_claim === "boolean"
    ? status.goalkeeper_may_claim ? `sí (≤ ${format(status.claim_max_ball_range)} m)` : `no (umbral ${format(status.claim_max_ball_range)} m)`
    : "—";
  $("claimCost").textContent = Number.isFinite(Number(status.claim_cost))
    ? `${format(status.claim_cost)} / ${format(status.claim_max_cost)}` : "—";
  $("teamLead").textContent = typeof status.team_lead === "boolean" ? status.team_lead ? "sí" : "no" : "—";
  $("goalkeeperMode").textContent = status.goalkeeper_mode || "—";
  $("fieldFilterState").textContent = !status.field_filter_enabled
    ? "desactivado"
    : status.field_filter_localization_ready ? "activo" : "esperando localización";
  $("fieldFilterRejected").textContent = Number.isFinite(Number(status.field_filter_rejected_count))
    ? String(status.field_filter_rejected_count) : "—";
  $("ballJumpRejected").textContent = Number.isFinite(Number(status.ball_jump_rejected_count))
    ? String(status.ball_jump_rejected_count) : "—";
  $("ballJumpLast").textContent = Number.isFinite(Number(status.ball_jump_last_distance))
    ? `${format(status.ball_jump_last_distance)} m → (${format(status.ball_jump_last_x)}, ${format(status.ball_jump_last_y)})`
    : "—";
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
  $("continuityFilterState").textContent = status.continuity_filter_enabled ? "activo" : "desactivado";
  $("forwardInterceptState").textContent = !status.forward_intercept_enabled
    ? "desactivada · línea fija"
    : status.forward_intercept_active
      ? status.front_intercept ? "ACTIVA · salida frontal" : "ACTIVA · salida diagonal"
      : "activa · sin punto adelantado alcanzable";
  $("adaptiveTarget").textContent = status.threatens_goal
    ? `(${format(status.block_target_field_x)}, ${format(status.block_target_field_y)}) m` : "—";
  $("adaptiveTargetTime").textContent = status.threatens_goal
    ? `${format(status.block_target_time)} s` : "—";
  $("adaptiveReachSpeed").textContent = status.forward_intercept_enabled
    ? `${format(status.adaptive_reach_speed)} m/s` : "—";
  $("ballConfidence").textContent = `${format(status.ball_confidence, 1)} %`;
  $("robotPose").textContent = `(${format(status.robot_x)}, ${format(status.robot_y)}, ${format(status.robot_theta)})`;
  $("postBlockClearance").textContent = status.post_block_clearance ? "activo: buscar y patear" : "inactivo";
  $("urgentBlock").textContent = status.urgent_block ? "ACTIVO: máxima prioridad lateral" : "inactivo";
  $("urgentBlock").classList.toggle("critical", Boolean(status.urgent_block));
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
  const targetX = Number.isFinite(Number(status.block_target_field_x)) ? status.block_target_field_x : status.intercept_x;
  const targetY = Number.isFinite(Number(status.block_target_field_y)) ? status.block_target_field_y : status.intercept_y;
  const interceptVisible = Boolean(status.threatens_goal) && Number.isFinite(Number(targetX)) && Number.isFinite(Number(targetY));
  svgPoint($("interceptMarker"), px(targetX), py(targetY), interceptVisible);
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
  speed_above_maximum: "velocidad irreal: posible salto de visión",
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
      [stamp, event.decision || "—", reactionStageLabel(event.reaction_stage),
       formatMs(event.decision_to_command_msec), formatMs(event.command_to_motion_msec),
       motionVector(event.command_sent_x, event.command_sent_y, event.command_sent_theta),
       Number.isFinite(Number(event.odom_speed)) ? `${format(event.odom_speed)} m/s` : "—",
       event.goalkeeper_may_claim ? "sí" : "no",
       reasonLabel(event.prediction_reason), event.sample_count ?? "—"].forEach(value => {
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
$("fastProfile").onclick = loadRecommendedProfile;
$("helpLink").onclick = () => { $("help").open = true; };
document.querySelectorAll(".go-group").forEach(button => {
  button.onclick = () => selectGroup(button.dataset.group);
});
if (location.hash === "#help") $("help").open = true;
load();
poll();
