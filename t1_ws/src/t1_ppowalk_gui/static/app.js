(function () {
  const APPLY_DEBOUNCE_MS = 400;
  const COMMAND_DEBOUNCE_MS = 120;
  const AXES = [
    { key: "vx", param: "cmd_vx", label: "Forward vx", unit: "m/s", min: "lin_vel_x_min", max: "lin_vel_x_max" },
    { key: "vy", param: "cmd_vy", label: "Lateral vy", unit: "m/s", min: "lin_vel_y_min", max: "lin_vel_y_max" },
    { key: "wz", param: "cmd_wz", label: "Yaw rate wz", unit: "rad/s", min: "ang_vel_z_min", max: "ang_vel_z_max" },
  ];

  const state = {
    ws: null,
    requestId: 0,
    pending: new Map(),
    config: null,
    values: { policy: {}, bridge: {} },
    dirty: {},
    timers: {},
    status: null,
    command: { vx: 0, vy: 0, wz: 0 },
    commandTimer: null,
    source: null,  // velocity command source: "remote", "gui" or "cmd_vel"
    sourceHoldUntil: 0,  // ignore the status source briefly after switching it
    remoteEvents: null,
    modelsKey: "",
  };

  const SOURCE_HINTS = {
    gui: "Sent to the policy while it runs (cmd_vx / cmd_vy / cmd_wz). Slider ranges follow the limits in the Policy tab.",
    remote: "The remote's sticks command the velocity (see Remote controller below); the sliders are inactive.",
    cmd_vel: "The command comes from geometry_msgs/Twist on /cmd_vel; the sliders are inactive.",
  };

  // ---------------------------------------------------------------- transport

  function connect() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    state.ws = new WebSocket(`${proto}://${location.host}/ws`);
    state.ws.onopen = () => {
      setStatus("connected");
      document.getElementById("conn-lost").hidden = true;
    };
    state.ws.onclose = () => {
      setStatus("disconnected, retrying…");
      document.getElementById("conn-lost").hidden = false;
      for (const { reject } of state.pending.values()) reject(new Error("disconnected"));
      state.pending.clear();
      setTimeout(connect, 1000);
    };
    state.ws.onerror = () => setStatus("connection error");
    state.ws.onmessage = (event) => handleMessage(JSON.parse(event.data));
  }

  function setStatus(text) {
    document.getElementById("status").textContent = text;
  }

  function handleMessage(message) {
    if (message.type === "hello") {
      state.config = message.config;
      document.getElementById("policy-node").textContent = message.config.policy_node;
      buildScalarForm("policy", message.config.policy_specs);
      buildScalarForm("policy", message.config.offset_specs, "params-offsets");
      buildScalarForm("policy", message.config.remote_specs || [], "params-remote");
      buildScalarForm("bridge", message.config.bridge_specs);
      readParameters();
    } else if (message.type === "status") {
      renderStatus(message.status);
    } else if (message.type === "ack" && state.pending.has(message.request_id)) {
      const { resolve, reject } = state.pending.get(message.request_id);
      state.pending.delete(message.request_id);
      if (message.ok) resolve(message);
      else reject(new Error(message.error || message.reason || message.message || "request_failed"));
    }
  }

  function sendCommand(action, payload) {
    return new Promise((resolve, reject) => {
      if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
        reject(new Error("not_connected"));
        return;
      }
      const requestId = String(++state.requestId);
      state.pending.set(requestId, { resolve, reject });
      state.ws.send(JSON.stringify(Object.assign({ action, request_id: requestId }, payload || {})));
    });
  }

  async function run(action, payload, label) {
    try {
      const result = await sendCommand(action, payload);
      setStatus(`${label}: ${result.message || "ok"}`);
      return result;
    } catch (err) {
      setStatus(`${label} failed: ${err.message}`);
      return null;
    }
  }

  // In-page dialog instead of window.confirm(), which would block the page.
  function confirmModal(text) {
    return new Promise((resolve) => {
      const modal = document.getElementById("modal");
      document.getElementById("modal-text").textContent = text;
      modal.hidden = false;
      const done = (answer) => {
        modal.hidden = true;
        resolve(answer);
      };
      document.getElementById("modal-ok").onclick = () => done(true);
      document.getElementById("modal-cancel").onclick = () => done(false);
    });
  }

  // ------------------------------------------------------------------- status

  function fmtAge(age) {
    if (age === undefined || age < 0) return "never";
    return age < 1 ? `${(age * 1000).toFixed(0)} ms` : `${age.toFixed(1)} s`;
  }

  function renderStatus(status) {
    state.status = status;
    const policy = status.policy || {};
    const bridge = status.bridge || {};
    const mode = policy.online ? policy.mode : "offline";

    const up = document.getElementById("upright");
    if (!policy.online || policy.tilt_deg < 0) {
      up.className = "upright upright-unknown";
      document.getElementById("upright-label").textContent = "IMU —";
      document.getElementById("upright-tilt").textContent = "";
    } else {
      up.className = `upright ${policy.upright ? "upright-ok" : "upright-bad"}`;
      document.getElementById("upright-label").textContent = policy.upright ? "UPRIGHT" : "NOT UPRIGHT";
      document.getElementById("upright-tilt").textContent =
        `tilt ${policy.tilt_deg.toFixed(1)}° (max ${policy.max_start_tilt_deg.toFixed(0)}°)`;
    }

    document.getElementById("state").className = `state state-${mode.toLowerCase()}`;
    document.getElementById("policy-mode").textContent = mode;
    document.getElementById("bridge-state").textContent = bridge.online ? bridge.state.toUpperCase() : "offline";
    document.getElementById("motor-output").textContent =
      bridge.online ? (bridge.enable_lowcmd ? "ENABLED" : "disabled") : "—";
    document.getElementById("data-fresh").textContent = !policy.online ? "—"
      : policy.data_fresh ? `fresh (IMU ${fmtAge(policy.imu_age)})` : "STALE";
    document.getElementById("policy-step").textContent = policy.online ? policy.step : "—";
    document.getElementById("inference").textContent =
      policy.online && policy.step ? `${policy.inference_ms.toFixed(2)} ms` : "—";
    document.getElementById("command-live").textContent = !policy.online ? "—"
      : `${policy.command.map((v) => v.toFixed(2)).join(", ")} (${policy.command_source})`;
    document.getElementById("command-effective").textContent = !policy.online || !policy.command_raw ? "—"
      : `requested (${policy.command_raw.map((v) => v.toFixed(2)).join(", ")}) + offset (` +
        `${policy.command_offset.map((v) => v.toFixed(2)).join(", ")}) → (${policy.command.map((v) => v.toFixed(2)).join(", ")})`;
    renderRemote(policy);
    const offline = bridge.online ? (bridge.offline_joints || []) : [];
    document.getElementById("offline").textContent = !bridge.online ? "—"
      : offline.length ? offline.map((n) => n.replace(/_joint$/, "")).join(", ") : "none";
    const short = (list) => (list || []).map((n) => n.replace(/_joint$/, "")).join(", ") || "none";
    document.getElementById("commanded").textContent = !policy.online ? "—"
      : `${(policy.command_joints || []).length} — ${short(policy.command_joints)}`;
    document.getElementById("pinned").textContent = !policy.online ? "—" : short(policy.pinned_joints);
    document.getElementById("model-path").textContent =
      !policy.online ? "—" : policy.policy_loaded ? policy.policy_path : "none loaded";
    document.getElementById("model-check").textContent = !policy.online || !policy.policy_loaded ? "—"
      : policy.model_warnings ? `WARNING: ${policy.model_warnings}` : "matches the parameters";
    document.getElementById("rpc-result").textContent = !policy.online ? "—" : policy.rpc_result || "none sent";
    document.getElementById("ankle-space").textContent = !policy.online ? "—"
      : policy.ankle_space === "UNVERIFIED"
        ? "UNVERIFIED: serial ankle states equal the crank angles, do not walk"
        : policy.ankle_space;
    document.getElementById("custom-mode").disabled = !policy.online;
    document.getElementById("prepare-mode").disabled = !policy.online || mode === "RUNNING" || mode === "RAMPING";

    // Start: only from IDLE/HOLDING, with a model, fresh data and the robot upright.
    const idle = mode === "IDLE" || mode === "HOLDING";
    const reasons = [];
    if (!policy.online) reasons.push("policy node offline");
    else {
      if (!policy.policy_loaded) reasons.push("no model loaded");
      if (!policy.data_fresh) reasons.push("no fresh LowState data");
      const legsOffline = (policy.offline_detected || []).filter((n) => /_(Hip|Knee|Ankle)_/.test(n));
      if (legsOffline.length) reasons.push(`leg motor error: ${legsOffline.join(", ")}`);
      if (!policy.upright) reasons.push(`robot not upright (tilt ${policy.tilt_deg.toFixed(0)}° > ${policy.max_start_tilt_deg.toFixed(0)}°)`);
      if (!idle) reasons.push(`policy is ${mode}`);
    }
    if (bridge.online && !bridge.enable_lowcmd && reasons.length === 0) {
      reasons.push("note: motor output is disabled, commands will not reach the motors");
    }
    const blocked = reasons.some((r) => !r.startsWith("note:"));
    document.getElementById("start").disabled = blocked;
    const remoteStart = policy.online && policy.remote && policy.remote.remote_start
      ? " START (+) on the remote does the same, with the same checks." : "";
    document.getElementById("start-hint").textContent =
      (reasons.length ? `Start ${blocked ? "blocked" : "allowed"}: ${reasons.join("; ")}.` : "Start allowed.") +
      remoteStart;
    document.getElementById("home").disabled = !policy.online || !policy.data_fresh;
    document.getElementById("stop").disabled = !policy.online || mode === "IDLE";
    document.getElementById("release").disabled = !policy.online || mode === "IDLE";
    renderModels(policy, idle);
    const toggle = document.getElementById("motor-toggle");
    toggle.querySelector(".label").textContent =
      bridge.enable_lowcmd ? "Disable motor output" : "Enable motor output";
    toggle.disabled = !bridge.online;
  }

  function renderRemote(policy) {
    const remote = (policy.online && policy.remote) || {};
    const live = policy.online && remote.fresh;
    const sticks = remote.sticks || [0, 0, 0, 0];
    const pair = (a, b) => `(${a.toFixed(2)}, ${b.toFixed(2)})`;
    document.getElementById("remote-state").textContent = !policy.online ? "—"
      : `${live ? "live" : "NO DATA"} · START ${remote.remote_start ? "on" : "off"} · BACK ${remote.remote_home ? "on" : "off"}`;
    document.getElementById("remote-left").textContent = live ? pair(sticks[0], sticks[1]) : "—";
    document.getElementById("remote-right").textContent = live ? pair(sticks[2], sticks[3]) : "—";
    document.getElementById("remote-buttons").textContent =
      live ? ((remote.buttons || []).join(", ") || "none") : "—";
    document.getElementById("remote-event").textContent = remote.events
      ? `Last remote event (${fmtAge(remote.event_age)} ago): ${remote.last_event}` : "";
    const events = policy.online ? (remote.events || 0) : null;
    if (events !== null && state.remoteEvents !== null && events > state.remoteEvents) {
      setStatus(`Remote: ${remote.last_event}`);
    }
    if (events !== null) state.remoteEvents = events;
    if (policy.online && Date.now() > state.sourceHoldUntil) {
      const src = String(policy.command_source || "").startsWith("cmd_vel") ? "cmd_vel"
        : remote.use_remote ? "remote" : "gui";
      if (src !== state.source) setSource(src);
    }
  }

  // Model dropdown: the *.onnx files of the node's models_dir. Not rebuilt while
  // it is open (focused).
  function renderModels(policy, idle) {
    const select = document.getElementById("policy-select");
    const dir = policy.models_dir || "";
    const models = policy.online ? (policy.models || []) : [];
    const current = policy.online && policy.policy_loaded ? policy.policy_path : "";
    const key = JSON.stringify([dir, models, current]);
    if (key !== state.modelsKey && document.activeElement !== select) {
      state.modelsKey = key;
      select.innerHTML = "";
      const paths = models.map((name) => `${dir}/${name}`);
      if (!current) {
        const none = new Option("(no model loaded)", "");
        none.disabled = true;
        select.add(none);
      } else if (!paths.includes(current)) {
        select.add(new Option(`${current} (outside the folder)`, current));
      }
      models.forEach((name, i) => select.add(new Option(name, paths[i])));
      select.value = current;
    }
    select.disabled = !policy.online || !idle;
    document.getElementById("models-hint").textContent = !policy.online ? ""
      : policy.models_error ? `Cannot read ${dir}: ${policy.models_error}`
      : `${models.length} model${models.length === 1 ? "" : "s"} in ${dir}. Choosing one loads it ` +
        "right away (only while the policy is stopped); the next Start uses it. A model with the " +
        "wrong input/output size is rejected and the current one stays loaded.";
  }

  // ---------------------------------------------------------- velocity command

  function setSource(src) {
    state.source = src;
    for (const radio of document.querySelectorAll('input[name="source"]')) radio.checked = radio.value === src;
    const gui = src === "gui";
    for (const input of document.querySelectorAll("#command input")) input.disabled = !gui;
    document.getElementById("zero").disabled = !gui;
    document.getElementById("command-hint").textContent = SOURCE_HINTS[src] || "";
  }

  async function changeSource(src) {
    const previous = state.source;
    state.sourceHoldUntil = Date.now() + 5000;
    try {
      const result = await sendCommand("set_source", { source: src });
      if (!result.ok) throw new Error(result.reason || "rejected");
      setSource(src);
      setStatus(`Command source: ${src}`);
    } catch (err) {
      setStatus(`Command source REJECTED (${err.message})`);
      if (previous) setSource(previous);
    } finally {
      state.sourceHoldUntil = Date.now() + 500;  // let the status catch up
    }
  }

  function buildCommandForm() {
    const container = document.getElementById("command");
    container.innerHTML = "";
    const p = state.values.policy;
    for (const axis of AXES) {
      const lo = p[axis.min] ?? -1, hi = p[axis.max] ?? 1;
      const row = document.createElement("div");
      row.className = "param-row command-row";
      const label = document.createElement("label");
      label.textContent = `${axis.label} (${axis.unit})`;
      const slider = document.createElement("input");
      slider.type = "range";
      slider.min = lo;
      slider.max = hi;
      slider.step = 0.01;
      slider.id = `cmd-${axis.key}`;
      const number = document.createElement("input");
      number.type = "number";
      number.min = lo;
      number.max = hi;
      number.step = 0.05;
      number.id = `cmd-${axis.key}-num`;
      const value = p[axis.param] ?? 0;
      slider.value = number.value = value;
      state.command[axis.key] = value;
      const onChange = (v) => {
        const x = Math.min(hi, Math.max(lo, parseFloat(v) || 0));
        state.command[axis.key] = x;
        slider.value = number.value = x;
        scheduleCommand();
      };
      slider.addEventListener("input", () => onChange(slider.value));
      number.addEventListener("change", () => onChange(number.value));
      row.appendChild(label);
      row.appendChild(slider);
      row.appendChild(number);
      container.appendChild(row);
    }
    if (state.source) setSource(state.source);
  }

  function scheduleCommand() {
    clearTimeout(state.commandTimer);
    state.commandTimer = setTimeout(() => run("set_command", state.command, "Command"), COMMAND_DEBOUNCE_MS);
  }

  // ---------------------------------------------------------- parameter forms

  function key(node, name) {
    return `${node}.${name}`;
  }

  function buildScalarForm(node, specs, containerId) {
    const container = document.getElementById(containerId || `params-${node}`);
    container.innerHTML = "";
    for (const spec of specs) {
      const row = document.createElement("div");
      row.className = "param-row";
      const label = document.createElement("label");
      const text = spec.unit ? `${spec.label} (${spec.unit})` : spec.label;
      label.textContent = text;
      label.title = `${spec.name}: ${text}`;
      const input = document.createElement("input");
      input.id = `param_${node}_${spec.name}`;
      if (spec.type === "bool") {
        input.type = "checkbox";
        input.addEventListener("change", () => apply(node, spec.name, input.checked));
      } else {
        input.type = "number";
        input.step = spec.step;
        input.min = spec.min;
        input.max = spec.max;
        input.addEventListener("input", () => schedule(node, spec.name, () =>
          input.value === "" ? undefined : parseFloat(input.value)));
      }
      row.appendChild(label);
      row.appendChild(input);
      container.appendChild(row);
    }
  }

  function buildObservationScales(terms, scales) {
    const container = document.getElementById("obs-scales");
    container.innerHTML = "";
    (terms || []).forEach((term, i) => {
      const row = document.createElement("div");
      row.className = "param-row";
      const label = document.createElement("label");
      label.textContent = term;
      const input = document.createElement("input");
      input.type = "number";
      input.step = 0.01;
      input.value = scales && scales.length === terms.length ? scales[i] : 1.0;
      input.addEventListener("input", () => editArrayElement("policy", "observation_scales", i, input));
      row.appendChild(label);
      row.appendChild(input);
      container.appendChild(row);
    });
  }

  function buildJointTable() {
    const policy = state.values.policy;
    const names = policy.joint_names || state.values.bridge.joint_names || [];
    const table = document.getElementById("joint-table");
    table.innerHTML = "";
    const head = table.createTHead().insertRow();
    head.insertCell().textContent = "#";
    head.insertCell().textContent = "Joint";
    for (const flag of state.config.joint_flags || []) {
      const th = head.insertCell();
      th.textContent = flag.label;
      th.title = flag.title;
      th.className = "col-flag";
    }
    for (const col of state.config.joint_columns) {
      const th = head.insertCell();
      th.textContent = col.unit ? `${col.label} (${col.unit})` : col.label;
      th.className = `col-${col.node}`;
    }
    const body = table.createTBody();
    names.forEach((joint, row) => {
      const tr = body.insertRow();
      tr.insertCell().textContent = row;
      tr.insertCell().textContent = joint.replace(/_joint$/, "");
      for (const flag of state.config.joint_flags || []) {
        const td = tr.insertCell();
        td.className = "col-flag";
        const box = document.createElement("input");
        box.type = "checkbox";
        const list = policy[flag.name] || [];
        box.checked = list.includes(joint);
        const locked = (policy.action_joint_names || []).includes(joint) ||
          (policy.always_commanded_joints || []).includes(joint);
        if (flag.name === "command_joints" && locked) {
          box.disabled = true;  // action joints and always-commanded joints (waist)
          box.title = "always commanded";
        }
        box.addEventListener("change", () => toggleJointFlag(flag.name, joint, box.checked));
        td.appendChild(box);
      }
      for (const col of state.config.joint_columns) {
        const td = tr.insertCell();
        td.className = `col-${col.node}`;
        const values = state.values[col.node][col.name];
        const indexNames = state.values[col.node][col.index_by] || [];
        const index = indexNames.indexOf(joint);
        if (index < 0) {
          td.textContent = "—";  // e.g. no action scale for the neck
          continue;
        }
        const input = document.createElement("input");
        input.type = "number";
        input.step = col.step;
        input.id = `joint_${col.node}_${col.name}_${index}`;
        if (!Array.isArray(values)) {
          input.disabled = true;
        } else if (col.readonly) {
          input.value = values[index];
          input.disabled = true;
          input.title = "read-only (from the training config)";
        } else {
          input.value = values.length === 1 ? values[0] : values[index];
          input.addEventListener("input", () => editArrayElement(col.node, col.name, index, input));
        }
        td.appendChild(input);
      }
    });
  }

  async function toggleJointFlag(name, joint, on) {
    const current = (state.values.policy[name] || []).slice();
    const names = state.values.policy.joint_names || [];
    const set = new Set(current);
    if (on) set.add(joint); else set.delete(joint);
    // Keep joint-table order.
    const next = names.filter((n) => set.has(n));
    const result = await apply("policy", name, next);
    if (!result) showValues();  // rejected: restore the checkboxes
  }

  function editArrayElement(node, name, index, input) {
    schedule(node, name, () => {
      if (input.value === "") return undefined;
      let values = (state.values[node][name] || []).slice();
      const count = (state.values[node].action_joint_names || []).length;
      if (name === "action_scale" && values.length === 1 && count > 1) {
        values = new Array(count).fill(values[0]);
      }
      values[index] = parseFloat(input.value);
      return values;
    });
  }

  function schedule(node, name, getValue) {
    const k = key(node, name);
    state.dirty[k] = true;
    clearTimeout(state.timers[k]);
    state.timers[k] = setTimeout(() => {
      delete state.timers[k];
      const value = getValue();
      if (value !== undefined) apply(node, name, value);
    }, APPLY_DEBOUNCE_MS);
  }

  async function apply(node, name, value) {
    const k = key(node, name);
    try {
      const result = await sendCommand("set_parameters", { node, parameters: { [name]: value } });
      if (!result.ok) throw new Error(result.reason || "rejected");
      state.values[node][name] = value;
      setStatus(`${node}: ${name} applied`);
      markInput(node, name, "applied");
      if (/^(lin|ang)_vel_/.test(name)) buildCommandForm();
      return result;
    } catch (err) {
      const message = `${node}: ${name} REJECTED (${err.message})`;
      setStatus(message);
      markInput(node, name, "rejected");
      delete state.dirty[k];
      await readParameters();
      setStatus(message);  // keep the reason visible after the re-read
      return null;
    } finally {
      delete state.dirty[k];
    }
  }

  function markInput(node, name, cls) {
    const inputs = document.querySelectorAll(`[id^="param_${node}_${name}"], [id^="joint_${node}_${name}_"]`);
    for (const input of inputs) {
      input.classList.remove("applied", "rejected");
      input.classList.add(cls);
      setTimeout(() => input.classList.remove(cls), 1200);
    }
  }

  function showValues() {
    for (const node of ["policy", "bridge"]) {
      const specs = node === "policy"
        ? state.config.policy_specs.concat(state.config.offset_specs || [], state.config.remote_specs || [])
        : state.config.bridge_specs;
      for (const spec of specs) {
        if (state.dirty[key(node, spec.name)]) continue;
        const input = document.getElementById(`param_${node}_${spec.name}`);
        const value = state.values[node][spec.name];
        if (!input || value === undefined || value === null) continue;
        if (spec.type === "bool") input.checked = value;
        else input.value = value;
      }
    }
    const policy = state.values.policy;
    buildObservationScales(policy.observation_terms, policy.observation_scales);
    if (policy.use_remote !== undefined && Date.now() > state.sourceHoldUntil) {
      state.source = policy.use_cmd_vel ? "cmd_vel" : policy.use_remote ? "remote" : "gui";
    }
    buildCommandForm();
    buildJointTable();
  }

  async function readParameters() {
    try {
      const result = await sendCommand("read_parameters");
      for (const node of ["policy", "bridge"]) state.values[node] = result.parameters[node] || {};
      showValues();
      const errors = Object.entries(result.errors || {}).map(([n, e]) => `${n}: ${e}`);
      setStatus(errors.length ? `read: ${errors.join(", ")}` : "parameters loaded");
    } catch (err) {
      setStatus(`read error: ${err.message}`);
    }
  }

  // ------------------------------------------------------------------ actions

  document.getElementById("home").addEventListener("click", async () => {
    const mode = state.status && state.status.policy ? state.status.policy.mode : "";
    const running = mode === "RUNNING" || mode === "RAMPING";
    const ok = await confirmModal(running
      ? "Stop the policy and move slowly to the home pose?"
      : "Move slowly to the home pose and hold it? (Motor output must be enabled to move.)");
    if (ok) run("home", null, "Home");
  });
  document.getElementById("start").addEventListener("click", () => run("start", null, "Start"));
  document.getElementById("stop").addEventListener("click", () => run("stop", null, "Stop"));
  document.getElementById("release").addEventListener("click", async () => {
    if (await confirmModal("Release: damp every joint (kp = 0). A standing robot will collapse. Continue?")) {
      run("release", null, "Release");
    }
  });
  document.getElementById("zero").addEventListener("click", () => {
    for (const axis of AXES) {
      state.command[axis.key] = 0;
      const s = document.getElementById(`cmd-${axis.key}`);
      const n = document.getElementById(`cmd-${axis.key}-num`);
      if (s) s.value = 0;
      if (n) n.value = 0;
    }
    clearTimeout(state.commandTimer);
    run("set_command", state.command, "Zero command");
  });
  document.getElementById("motor-toggle").addEventListener("click", async () => {
    const enabled = state.status && state.status.bridge && state.status.bridge.enable_lowcmd;
    if (enabled) {
      apply("bridge", "enable_lowcmd", false);
      return;
    }
    if (await confirmModal("Enable low-level motor output (LowCmd)? The robot must be in CUSTOM MODE and secured (hanging). While IDLE every joint is damped.")) {
      apply("bridge", "enable_lowcmd", true);
    }
  });
  document.getElementById("custom-mode").addEventListener("click", async () => {
    if (await confirmModal("Switch the robot to Custom mode? The built-in controller stops and only this node's LowCmd moves the joints. Hang the robot first.")) {
      run("enter_custom_mode", null, "Custom mode");
    }
  });
  document.getElementById("prepare-mode").addEventListener("click", async () => {
    if (await confirmModal("Switch the robot back to Prepare mode (built-in controller)? Disable motor output first.")) {
      run("exit_custom_mode", null, "Prepare mode");
    }
  });
  document.getElementById("policy-select").addEventListener("change", async (event) => {
    const select = event.target;
    if (select.value) {
      const name = select.options[select.selectedIndex].text;
      if (await apply("policy", "policy_path", select.value)) setStatus(`Loaded model ${name}`);
    }
    state.modelsKey = "";  // re-render with the node's loaded model
    select.blur();
  });
  document.getElementById("home-zero").addEventListener("click", () => {
    const n = (state.values.policy.joint_names || []).length;
    apply("policy", "home_pose", new Array(n).fill(0)).then(() => showValues());
  });
  document.getElementById("home-default").addEventListener("click", () => {
    apply("policy", "home_pose", (state.values.policy.default_joint_pos || []).slice()).then(() => showValues());
  });
  document.querySelectorAll("button.refresh").forEach((b) => b.addEventListener("click", readParameters));

  // Parameter groups as tabs. Hidden panels keep updating; the selected tab is
  // remembered per browser.
  const TAB_KEY = "t1_ppowalk_gui.tab";
  const tabs = Array.from(document.querySelectorAll('[role="tab"]'));
  function selectTab(tab, focus) {
    for (const t of tabs) {
      const on = t === tab;
      t.setAttribute("aria-selected", on ? "true" : "false");
      t.tabIndex = on ? 0 : -1;
      document.getElementById(t.getAttribute("aria-controls")).hidden = !on;
    }
    if (focus) tab.focus();
    try { localStorage.setItem(TAB_KEY, tab.id); } catch (err) { /* storage unavailable */ }
  }
  tabs.forEach((tab, i) => {
    tab.addEventListener("click", () => selectTab(tab, false));
    tab.addEventListener("keydown", (event) => {
      const step = { ArrowRight: 1, ArrowLeft: -1 }[event.key];
      if (step) {
        event.preventDefault();
        selectTab(tabs[(i + step + tabs.length) % tabs.length], true);
      }
    });
  });
  let savedTab = null;
  try { savedTab = document.getElementById(localStorage.getItem(TAB_KEY)); } catch (err) { /* ignore */ }
  selectTab(tabs.includes(savedTab) ? savedTab : tabs[0], false);
  for (const radio of document.querySelectorAll('input[name="source"]')) {
    radio.addEventListener("change", () => {
      if (radio.checked) changeSource(radio.value);
    });
  }

  connect();
})();
