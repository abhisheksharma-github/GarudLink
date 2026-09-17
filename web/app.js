/**
 * @file app.js
 * @brief High-performance Web Tactical PPI Radar, Kinematic Renderer, WebSocket Bridge & CMS Cockpit.
 */

'use strict';

// ============================================================================
// State Management & Global Variables
// ============================================================================
const state = {
  wsConnected: false,
  ws: null,
  simTimeSec: 0.0,
  paused: false,
  maxDisplayRangeM: 25000.0, // Default 25 km PPI scope
  showTrails: true,
  showVectors: true,
  autoEngage: true,
  autoScrollTerminal: true,
  currentTerminalFilter: 'ALL',
  rxPacketCounter: 0,
  activeThreatSpawnType: 'KAMIKAZE',

  // Radar System State
  radar: {
    azimuthDeg: 0.0,
    mode: 'SURVEILLANCE_360',
    cueAzimuthDeg: 45.0,
    sectorWidthDeg: 45.0,
    lockedTargetId: '',
    scanRateDegPerSec: 90.0,
    txPowerWatts: 60000.0,
    gainDbi: 38.0,
    freqHz: 9.4e9,
    minDetectableWatts: 1e-13,
    radarHorizonM: 33643.0 // 4/3 Earth curvature horizon ~33.6 km
  },

  ownship: {
    id: 'DDG-1000',
    pos: { x: 0, y: 0, z: -25 },
    vel: { x: 0, y: 0, z: 0 }
  },

  // Active Entities (Drones, Missiles, Radars)
  entities: [],

  // Engagement Flight Data Recorder (AAR)
  aarRecords: []
};

// ============================================================================
// DOM Element References
// ============================================================================
const elements = {
  radarCanvas: document.getElementById('radarCanvas'),
  altitudeCanvas: document.getElementById('altitudeCanvas'),
  radarWrapper: document.getElementById('radarWrapper'),
  radarTooltip: document.getElementById('radarCrosshairTooltip'),
  terminalStream: document.getElementById('terminalStream'),
  threatTableBody: document.getElementById('threatTableBody'),

  // Badges & Telemetry
  connStatusBadge: document.getElementById('connStatusBadge'),
  radarModeBadge: document.getElementById('radarModeBadge'),
  zuluTime: document.getElementById('zuluTime'),
  scopeAzimuthReadout: document.getElementById('scopeAzimuthReadout'),
  scopeRangeReadout: document.getElementById('scopeRangeReadout'),
  rxPacketCount: document.getElementById('rxPacketCount'),
  thresholdVal: document.getElementById('thresholdVal'),
  motorStageBadge: document.getElementById('motorStageBadge'),
  seekerLookAngleVal: document.getElementById('seekerLookAngleVal'),

  // Controls
  autoEngageToggle: document.getElementById('autoEngageToggle'),
  targetEvasionToggle: document.getElementById('targetEvasionToggle'),
  targetJammingToggle: document.getElementById('targetJammingToggle'),
  btnManualLaunch: document.getElementById('btnManualLaunch'),
  btnDownloadAar: document.getElementById('btnDownloadAar'),
  btnResetScenario: document.getElementById('btnResetScenario'),
  btnSpawnQuad: document.getElementById('btnSpawnQuad'),
  btnSpawnKami: document.getElementById('btnSpawnKami'),
  btnSpawnAsm: document.getElementById('btnSpawnAsm'),

  // Scenario Loader Elements
  scenarioDropZone: document.getElementById('scenarioDropZone'),
  scenarioFileInput: document.getElementById('scenarioFileInput'),
  btnBrowseFile: document.getElementById('btnBrowseFile'),
  btnLoadDefaultScenario: document.getElementById('btnLoadDefaultScenario'),
  btnLoadStraitScenario: document.getElementById('btnLoadStraitScenario'),

  // Manual Target Injection Elements
  formManualInject: document.getElementById('formManualInject'),
  inpThreatType: document.getElementById('inpThreatType'),
  inpRcs: document.getElementById('inpRcs'),
  inpPosX: document.getElementById('inpPosX'),
  inpPosY: document.getElementById('inpPosY'),
  inpPosZ: document.getElementById('inpPosZ'),
  inpVelX: document.getElementById('inpVelX'),
  inpVelY: document.getElementById('inpVelY'),
  inpVelZ: document.getElementById('inpVelZ'),

  // Mode buttons
  btnModeSurv: document.getElementById('btnModeSurv'),
  btnModeSector: document.getElementById('btnModeSector'),
  btnModeStt: document.getElementById('btnModeStt'),

  // Toolbar
  rangeButtons: document.querySelectorAll('.btn-range'),
  btnToggleTrails: document.getElementById('btnToggleTrails'),
  btnToggleVectors: document.getElementById('btnToggleVectors'),
  btnClearTerminal: document.getElementById('btnClearTerminal'),
  btnAutoScroll: document.getElementById('btnAutoScroll'),
  filterTabs: document.querySelectorAll('.filter-tab'),
  mobileNavBtns: document.querySelectorAll('.mobile-nav-btn')
};

// Canvas Contexts
const ctx = elements.radarCanvas.getContext('2d');
const altCtx = elements.altitudeCanvas.getContext('2d');

let wsRetryTimeout = null;
let wsAttempts = 0;

function initWebSocket() {
  if (wsRetryTimeout) {
    clearTimeout(wsRetryTimeout);
    wsRetryTimeout = null;
  }

  const wsUrl = `ws://${window.location.hostname || 'localhost'}:8080`;
  elements.connStatusBadge.textContent = 'CONNECTING...';
  elements.connStatusBadge.className = 'hud-value badge-amber';

  try {
    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      state.ws = ws;
      state.wsConnected = true;
      wsAttempts = 0;
      elements.connStatusBadge.textContent = 'C++ WS LIVE';
      elements.connStatusBadge.className = 'hud-value badge-green';
      appendTerminalLog('$CS,WS,CONNECTED,8080*4E');
    };

    ws.onmessage = (event) => {
      handleIncomingTelemetryJson(event.data);
    };

    ws.onclose = () => {
      state.wsConnected = false;
      elements.connStatusBadge.textContent = 'STANDALONE';
      elements.connStatusBadge.className = 'hud-value badge-cyan';
      wsAttempts++;
      // Backoff retry only up to 3 times, then stay in standalone mode
      if (wsAttempts <= 3) {
        wsRetryTimeout = setTimeout(initWebSocket, 8000);
      }
    };

    ws.onerror = () => {
      state.wsConnected = false;
    };
  } catch (e) {
    state.wsConnected = false;
    elements.connStatusBadge.textContent = 'STANDALONE';
    elements.connStatusBadge.className = 'hud-value badge-cyan';
  }
}

function sendCommand(cmdObj) {
  if (state.wsConnected && state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(cmdObj));
  } else {
    // Client-side fallback handler
    handleClientSideCommand(cmdObj);
  }
}

function handleIncomingTelemetryJson(jsonStr) {
  try {
    const data = JSON.parse(jsonStr);
    state.simTimeSec = data.sim_time_sec || state.simTimeSec;

    if (data.radar) {
      state.radar.azimuthDeg = data.radar.azimuth_deg;
      state.radar.mode = data.radar.mode || state.radar.mode;
      state.radar.cueAzimuthDeg = data.radar.cue_azimuth_deg || 0;
      state.radar.sectorWidthDeg = data.radar.sector_width_deg || 45;
      state.radar.lockedTargetId = data.radar.locked_target_id || '';
      updateRadarModeUI(state.radar.mode);
    }

    if (data.entities && Array.isArray(data.entities)) {
      state.entities = data.entities.map(e => ({
        id: e.id,
        type: e.type,
        threatType: e.threat_type || 'NONE',
        classification: e.classification,
        active: e.active,
        pos: { x: e.position[0], y: e.position[1], z: e.position[2] },
        vel: { x: e.velocity[0], y: e.velocity[1], z: e.velocity[2] },
        speed: e.speed_mps,
        headingDeg: e.heading_deg,
        altitudeM: e.altitude_m,
        rcs: e.rcs_m2,
        evasive: e.evasive,
        jamming: e.jamming,
        motorStage: e.motor_stage,
        lookAngleDeg: e.look_angle_deg,
        closingVel: e.closing_velocity_mps,
        rangeToGo: e.range_to_go_m,
        targetLock: e.target_lock
      }));
    }

    if (data.recent_nmea_logs && Array.isArray(data.recent_nmea_logs)) {
      data.recent_nmea_logs.forEach(sentence => {
        appendTerminalLog(sentence);
      });
    }

    updateHudDisplays();
  } catch (e) {
    console.error('Error parsing telemetry JSON:', e);
  }
}

// ============================================================================
// Client-Side Fallback Engine (For offline / standalone mode)
// ============================================================================

let clientThreatIdCounter = 1;
let clientMissileIdCounter = 1;

function initInitialScenario() {
  state.entities = [
    {
      id: 'RDR-01',
      type: 'RADAR',
      classification: 'FRIENDLY',
      active: true,
      pos: { x: 0, y: 0, z: -25 },
      vel: { x: 0, y: 0, z: 0 },
      speed: 0, headingDeg: 0, altitudeM: 25,
      rcs: 100
    },
    {
      id: 'UAV-ALPHA',
      type: 'DRONE',
      threatType: 'RECON_QUAD_UAV',
      classification: 'HOSTILE',
      active: true,
      pos: { x: 14142, y: 14142, z: -1500 },
      vel: { x: -127.3, y: -127.3, z: 0 },
      speed: 180, headingDeg: 225, altitudeM: 1500,
      rcs: 0.5,
      evasive: false,
      jamming: false,
      trail: []
    }
  ];
}

function handleClientSideCommand(cmd) {
  if (cmd.command === 'SPAWN_TARGET') {
    const id = cmd.id || `TGT-${clientThreatIdCounter++}`;
    const speed = cmd.speed_mps || (cmd.threat_type === 'SEA_SKIMMER_ASM' ? 750 : (cmd.threat_type === 'KAMIKAZE' ? 70 : 35));
    const alt = cmd.altitude_m || (cmd.threat_type === 'SEA_SKIMMER_ASM' ? 10 : (cmd.threat_type === 'KAMIKAZE' ? 300 : 150));
    const rcs = cmd.rcs_m2 || (cmd.threat_type === 'SEA_SKIMMER_ASM' ? 0.15 : (cmd.threat_type === 'KAMIKAZE' ? 0.05 : 0.02));

    const dist = Math.hypot(cmd.x, cmd.y) || 1;
    const vx = -(cmd.x / dist) * speed;
    const vy = -(cmd.y / dist) * speed;

    state.entities.push({
      id: id,
      type: 'DRONE',
      threatType: cmd.threat_type || 'CUSTOM',
      classification: 'HOSTILE',
      active: true,
      pos: { x: cmd.x, y: cmd.y, z: -alt },
      vel: { x: vx, y: vy, z: 0 },
      speed: speed,
      headingDeg: (Math.atan2(vy, vx) * 180 / Math.PI + 360) % 360,
      altitudeM: alt,
      rcs: rcs,
      evasive: cmd.evasive || false,
      jamming: cmd.jamming || false,
      trail: []
    });

    appendTerminalLog(`$CS,SPAWN,${id},${cmd.threat_type},${Math.round(cmd.x)},${Math.round(cmd.y)}*5F`);
  } else if (cmd.command === 'MANUAL_SPAWN') {
    const id = cmd.id || `TGT-${clientThreatIdCounter++}`;
    const vx = cmd.vx || 0;
    const vy = cmd.vy || 0;
    const vz = cmd.vz || 0;
    const speed = Math.hypot(vx, vy, vz) || 50;
    const alt = Math.abs(cmd.z || 1000);
    const rcs = cmd.rcs || 0.05;

    state.entities.push({
      id: id,
      type: 'DRONE',
      threatType: cmd.customType || 'UAV',
      classification: 'HOSTILE',
      active: true,
      pos: { x: cmd.x || 15000, y: cmd.y || 15000, z: cmd.z || -1500 },
      vel: { x: vx, y: vy, z: vz },
      speed: speed,
      headingDeg: (Math.atan2(vy, vx) * 180 / Math.PI + 360) % 360,
      altitudeM: alt,
      rcs: rcs,
      evasive: false,
      jamming: false,
      trail: []
    });

    appendTerminalLog(`$CS,SPAWN,${id},${cmd.customType || 'UAV'},${Math.round(cmd.x || 0)},${Math.round(cmd.y || 0)}*5F`);
  } else if (cmd.command === 'LOAD_SCENARIO') {
    try {
      const scn = typeof cmd.scenarioJson === 'string' ? JSON.parse(cmd.scenarioJson) : cmd.scenarioJson;
      state.entities = [];
      // Ownship / Radar
      state.entities.push({
        id: 'RDR-01',
        type: 'RADAR',
        classification: 'FRIENDLY',
        active: true,
        pos: { x: scn.ownship?.position_m?.x || 0, y: scn.ownship?.position_m?.y || 0, z: scn.ownship?.position_m?.z || -25 },
        vel: { x: 0, y: 0, z: 0 },
        speed: 0, headingDeg: 0, altitudeM: 25, rcs: 100
      });
      if (scn.radar) {
        state.radar.txPowerWatts = scn.radar.transmitter_power_w || 60000;
        state.radar.gainDbi = scn.radar.antenna_gain_dbi || 38;
        state.radar.freqHz = scn.radar.frequency_hz || 9.4e9;
        state.radar.mode = scn.radar.scan_mode || 'SURVEILLANCE_360';
        updateRadarModeUI(state.radar.mode);
      }
      if (Array.isArray(scn.targets)) {
        scn.targets.forEach(t => {
          const pos = t.initial_position_m || [0, 0, 0];
          const vel = t.initial_velocity_mps || [0, 0, 0];
          const spd = Math.hypot(vel[0], vel[1], vel[2]);
          state.entities.push({
            id: t.id,
            type: 'DRONE',
            threatType: t.type || 'UAV',
            classification: 'HOSTILE',
            active: true,
            pos: { x: pos[0], y: pos[1], z: pos[2] },
            vel: { x: vel[0], y: vel[1], z: vel[2] },
            speed: spd,
            headingDeg: (Math.atan2(vel[1], vel[0]) * 180 / Math.PI + 360) % 360,
            altitudeM: Math.abs(pos[2]),
            rcs: t.rcs_m2 || 0.05,
            evasive: (t.evasion_profile && t.evasion_profile !== 'NONE'),
            jamming: false,
            trail: []
          });
        });
      }
      appendTerminalLog(`$CS,LOAD,SCENARIO_${scn.scenario_name || 'READY'},OK*43`);
    } catch (err) {
      console.error('Failed to parse scenario JSON client-side:', err);
    }
  } else if (cmd.command === 'AUTHORIZE_INTERCEPT') {
    launchClientMissile(cmd.target_id);
  } else if (cmd.command === 'SET_RADAR_MODE') {
    state.radar.mode = cmd.mode;
    state.radar.cueAzimuthDeg = cmd.cue_azimuth || 0;
    state.radar.sectorWidthDeg = cmd.sector_width_deg || 45;
    state.radar.lockedTargetId = cmd.target_id || '';
    updateRadarModeUI(cmd.mode);
    appendTerminalLog(`$RDR,MODE,${cmd.mode},${Math.round(state.radar.cueAzimuthDeg)}*41`);
  } else if (cmd.command === 'TRIGGER_EVASION') {
    state.entities.forEach(e => {
      if (e.id === cmd.target_id || !cmd.target_id) e.evasive = cmd.enable;
    });
  } else if (cmd.command === 'DEPLOY_JAMMING') {
    state.entities.forEach(e => {
      if (e.id === cmd.target_id || !cmd.target_id) e.jamming = cmd.enable;
    });
  } else if (cmd.command === 'RESET_SCENARIO') {
    initInitialScenario();
    appendTerminalLog('$CS,RESET,SCENARIO_READY*7A');
  }
}

function launchClientMissile(targetId) {
  const target = state.entities.find(e => (targetId ? e.id === targetId : (e.classification === 'HOSTILE' && e.active)));
  if (!target) return;

  const mslId = `MSL-${clientMissileIdCounter++}`;
  const range = Math.hypot(target.pos.x, target.pos.y);
  const dirX = target.pos.x / range;
  const dirY = target.pos.y / range;

  state.entities.push({
    id: mslId,
    type: 'MISSILE',
    targetId: target.id,
    classification: 'FRIENDLY',
    active: true,
    pos: { x: 0, y: 0, z: -10 },
    vel: { x: dirX * 350, y: dirY * 350, z: -50 },
    speed: 350,
    headingDeg: (Math.atan2(dirY, dirX) * 180 / Math.PI + 360) % 360,
    altitudeM: 10,
    motorStage: 'BOOSTER',
    flightTime: 0,
    lookAngleDeg: 0,
    closingVel: 530,
    rangeToGo: range,
    targetLock: true,
    trail: []
  });

  appendTerminalLog(`$CS,ENGAGE,${mslId},${target.id},${Math.round(range)}*28`);
}

function updateClientSideSimulation(dt) {
  state.simTimeSec += dt;

  // 1. Advance Radar Scan
  if (state.radar.mode === 'SURVEILLANCE_360') {
    state.radar.azimuthDeg = (state.radar.azimuthDeg + state.radar.scanRateDegPerSec * dt) % 360;
  } else if (state.radar.mode === 'SECTOR_CUE') {
    if (!state.radar.dir) state.radar.dir = 1;
    state.radar.azimuthDeg += state.radar.dir * 240 * dt;
    const minAz = state.radar.cueAzimuthDeg - state.radar.sectorWidthDeg / 2;
    const maxAz = state.radar.cueAzimuthDeg + state.radar.sectorWidthDeg / 2;
    if (state.radar.azimuthDeg >= maxAz) { state.radar.azimuthDeg = maxAz; state.radar.dir = -1; }
    if (state.radar.azimuthDeg <= minAz) { state.radar.azimuthDeg = minAz; state.radar.dir = 1; }
  }

  // 2. Update Drone & Threat Physics
  state.entities.forEach(e => {
    if (!e.active) return;

    if (e.type === 'DRONE') {
      if (!e.trail) e.trail = [];
      e.trail.push({ x: e.pos.x, y: e.pos.y });
      if (e.trail.length > 50) e.trail.shift();

      if (e.evasive) {
        const lateral = Math.sin(state.simTimeSec * 3.5) * 6.0 * 9.81 * dt;
        e.vel.x += -e.vel.y * 0.005 * lateral;
        e.vel.y += e.vel.x * 0.005 * lateral;
      }

      e.pos.x += e.vel.x * dt;
      e.pos.y += e.vel.y * dt;
      e.pos.z += e.vel.z * dt;
      e.speed = Math.hypot(e.vel.x, e.vel.y);
      e.headingDeg = (Math.atan2(e.vel.y, e.vel.x) * 180 / Math.PI + 360) % 360;

      // Auto-Engage Trigger
      const dist = Math.hypot(e.pos.x, e.pos.y);
      if (state.autoEngage && dist <= 15000 && !e.engaged) {
        e.engaged = true;
        launchClientMissile(e.id);
      }
    }

    if (e.type === 'MISSILE') {
      if (!e.trail) e.trail = [];
      e.trail.push({ x: e.pos.x, y: e.pos.y });
      if (e.trail.length > 50) e.trail.shift();

      e.flightTime = (e.flightTime || 0) + dt;
      if (e.flightTime <= 3.0) {
        e.motorStage = 'BOOSTER';
        e.speed = Math.min(1050, e.speed + 220 * dt);
      } else if (e.flightTime <= 7.0) {
        e.motorStage = 'SUSTAINER';
      } else {
        e.motorStage = 'COAST';
        e.speed = Math.max(400, e.speed - 30 * dt);
      }

      // Proportional Navigation Guidance
      const target = state.entities.find(t => t.id === e.targetId && t.active);
      if (target) {
        const dx = target.pos.x - e.pos.x;
        const dy = target.pos.y - e.pos.y;
        const dz = target.pos.z - e.pos.z;
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        e.rangeToGo = dist;

        const mslHeading = Math.atan2(e.vel.y, e.vel.x);
        const losHeading = Math.atan2(dy, dx);
        let lookAngle = Math.abs(losHeading - mslHeading) * 180 / Math.PI;
        if (lookAngle > 180) lookAngle = 360 - lookAngle;
        e.lookAngleDeg = lookAngle;

        if (lookAngle <= 35.0) {
          e.targetLock = true;
          const turnRate = 4.0 * (losHeading - mslHeading);
          const newHeading = mslHeading + turnRate * dt;
          e.vel.x = Math.cos(newHeading) * e.speed;
          e.vel.y = Math.sin(newHeading) * e.speed;
        } else {
          e.targetLock = false; // Gimbal limit exceeded
        }

        e.pos.x += e.vel.x * dt;
        e.pos.y += e.vel.y * dt;
        e.pos.z += e.vel.z * dt;

        // Record AAR Point
        state.aarRecords.push({
          time: state.simTimeSec.toFixed(2),
          targetId: target.id,
          mslId: e.id,
          range: dist.toFixed(1),
          closingVel: (e.speed + target.speed).toFixed(1),
          lookAngle: lookAngle.toFixed(1),
          stage: e.motorStage,
          outcome: dist <= 12.0 ? 'HIT' : 'IN_FLIGHT'
        });

        // Intercept Proximity
        if (dist <= 12.0) {
          e.active = false;
          target.active = false;
          appendTerminalLog(`$CS,KILL,${e.id},${target.id}*7C`);
        }
      }
    }
  });

  updateHudDisplays();
}

// ============================================================================
// UI Updates & Radar Track Matrix
// ============================================================================

function updateRadarModeUI(mode) {
  elements.radarModeBadge.textContent = mode.replace('_', ' ');
  [elements.btnModeSurv, elements.btnModeSector, elements.btnModeStt].forEach(btn => {
    if (btn) btn.classList.toggle('active', btn.dataset.mode === mode);
  });
}

let lastHudUpdateTime = 0;

function updateHudDisplays(force = false) {
  const now = performance.now();
  if (!force && (now - lastHudUpdateTime < 100)) {
    return; // Throttle DOM manipulation to 10 Hz
  }
  lastHudUpdateTime = now;

  // Update Zulu Time
  const d = new Date();
  elements.zuluTime.textContent = `${String(d.getUTCHours()).padStart(2,'0')}:${String(d.getUTCMinutes()).padStart(2,'0')}:${String(d.getUTCSeconds()).padStart(2,'0')}.${Math.floor(d.getUTCMilliseconds()/100)}Z`;

  elements.scopeAzimuthReadout.textContent = `${state.radar.azimuthDeg.toFixed(1)}°`;
  elements.scopeRangeReadout.textContent = `${(state.maxDisplayRangeM / 1000).toFixed(1)} KM`;

  // Update Missile HUD
  const activeMsl = state.entities.find(e => e.type === 'MISSILE' && e.active);
  if (activeMsl) {
    elements.motorStageBadge.textContent = activeMsl.motorStage || 'IN FLIGHT';
    elements.motorStageBadge.className = 'hud-value ' + (activeMsl.motorStage === 'BOOSTER' ? 'badge-amber' : 'badge-cyan');
    elements.seekerLookAngleVal.textContent = `${(activeMsl.lookAngleDeg || 0).toFixed(1)}° (FOV ±35°)`;
    elements.seekerLookAngleVal.className = 'hud-value ' + (activeMsl.lookAngleDeg > 35 ? 'red' : 'green');
  } else {
    elements.motorStageBadge.textContent = 'READY (4 VLS)';
    elements.motorStageBadge.className = 'hud-value green';
    elements.seekerLookAngleVal.textContent = '0.0° (FOV ±35°)';
    elements.seekerLookAngleVal.className = 'hud-value green';
  }

  // Update Track Table
  let rows = '';
  const hostiles = state.entities.filter(e => e.classification === 'HOSTILE');

  if (hostiles.length === 0) {
    rows = '<tr><td colspan="6" class="text-center text-dim">No hostile contacts registered.</td></tr>';
  } else {
    hostiles.forEach(h => {
      const rngKm = (Math.hypot(h.pos.x, h.pos.y) / 1000).toFixed(1);
      const brg = ((Math.atan2(h.pos.y, h.pos.x) * 180 / Math.PI + 360) % 360).toFixed(1);
      const altM = Math.round(Math.abs(h.pos.z));
      const statusBadge = !h.active ? '<span class="badge-green">KILLED</span>' : 
                          (h.jamming ? '<span class="badge-red animate-pulse">JAMMING</span>' : 
                          (h.evasive ? '<span class="badge-amber">EVADING</span>' : '<span class="badge-red">INBOUND</span>'));

      rows += `
        <tr class="${h.active ? '' : 'text-dim'}">
          <td class="red font-bold">${h.id}</td>
          <td class="cyan">${h.threatType || 'DRONE'}</td>
          <td>${rngKm}</td>
          <td>${brg}°</td>
          <td>${altM}m</td>
          <td>${statusBadge}</td>
        </tr>
      `;
    });
  }
  elements.threatTableBody.innerHTML = rows;
}

function appendTerminalLog(sentence) {
  state.rxPacketCounter++;
  elements.rxPacketCount.textContent = state.rxPacketCounter;

  const line = document.createElement('div');
  line.className = 'log-line';

  // Tag coloring
  if (sentence.includes('$RDR')) line.classList.add('log-rdr');
  else if (sentence.includes('$MSL')) line.classList.add('log-msl');
  else if (sentence.includes('$UAV') || sentence.includes('$ASM')) line.classList.add('log-uav');
  else if (sentence.includes('$CS')) line.classList.add('log-cs');

  line.textContent = sentence;
  elements.terminalStream.appendChild(line);

  if (state.autoScrollTerminal) {
    elements.terminalStream.scrollTop = elements.terminalStream.scrollHeight;
  }

  // Limit DOM lines
  while (elements.terminalStream.children.length > 80) {
    elements.terminalStream.removeChild(elements.terminalStream.firstChild);
  }
}

// ============================================================================
// PPI Scope Rendering Engine
// ============================================================================

function renderRadarScope() {
  const w = elements.radarCanvas.width;
  const h = elements.radarCanvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const maxRadius = w * 0.44;

  // Phosphor persistence decay fade
  ctx.fillStyle = 'rgba(8, 12, 20, 0.22)';
  ctx.fillRect(0, 0, w, h);

  // 1. Range Rings
  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(0, 255, 102, 0.25)';
  const rings = [0.2, 0.4, 0.6, 0.8, 1.0];
  rings.forEach((rRatio) => {
    ctx.beginPath();
    ctx.arc(cx, cy, maxRadius * rRatio, 0, Math.PI * 2);
    ctx.stroke();

    // Range labels
    const ringKm = ((state.maxDisplayRangeM * rRatio) / 1000).toFixed(0);
    ctx.fillStyle = 'rgba(0, 255, 102, 0.6)';
    ctx.font = '10px "Share Tech Mono"';
    ctx.fillText(`${ringKm}KM`, cx + 4, cy - maxRadius * rRatio + 12);
  });

  // 2. Crosshairs & Radial Bearings
  ctx.beginPath();
  ctx.moveTo(cx - maxRadius, cy);
  ctx.lineTo(cx + maxRadius, cy);
  ctx.moveTo(cx, cy - maxRadius);
  ctx.lineTo(cx, cy + maxRadius);
  ctx.strokeStyle = 'rgba(0, 255, 102, 0.2)';
  ctx.stroke();

  // 3. 4/3 Earth Curvature Horizon Boundary (33.6 km ring)
  const horizonRatio = state.radar.radarHorizonM / state.maxDisplayRangeM;
  if (horizonRatio <= 1.2) {
    ctx.save();
    ctx.setLineDash([6, 6]);
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.5)';
    ctx.beginPath();
    ctx.arc(cx, cy, maxRadius * horizonRatio, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = 'rgba(0, 240, 255, 0.7)';
    ctx.fillText('4/3 RADAR HORIZON (33.6km)', cx - maxRadius * horizonRatio + 8, cy - 6);
    ctx.restore();
  }

  // 4. Phased Array Radar Sweep Beam
  const beamRad = (state.radar.azimuthDeg - 90) * (Math.PI / 180);
  const beamX = cx + Math.cos(beamRad) * maxRadius;
  const beamY = cy + Math.sin(beamRad) * maxRadius;

  const grad = ctx.createRadialGradient(cx, cy, 0, cx, cy, maxRadius);
  grad.addColorStop(0, 'rgba(0, 255, 102, 0.8)');
  grad.addColorStop(1, 'rgba(0, 255, 102, 0.05)');

  ctx.lineWidth = 2;
  ctx.strokeStyle = '#00ff66';
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(beamX, beamY);
  ctx.stroke();

  // Sector sweep cone shading for SECTOR_CUE mode
  if (state.radar.mode === 'SECTOR_CUE') {
    const cueRad = (state.radar.cueAzimuthDeg - 90) * (Math.PI / 180);
    const halfWidthRad = (state.radar.sectorWidthDeg / 2) * (Math.PI / 180);
    ctx.fillStyle = 'rgba(0, 255, 102, 0.08)';
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, maxRadius, cueRad - halfWidthRad, cueRad + halfWidthRad);
    ctx.closePath();
    ctx.fill();
  }

  // 5. Draw Trails & Vectors
  state.entities.forEach(e => {
    if (!e.active || e.type === 'RADAR') return;

    const scale = maxRadius / state.maxDisplayRangeM;
    const ex = cx + (e.pos.y * scale); // East = +X on screen
    const ey = cy - (e.pos.x * scale); // North = -Y on screen

    // Flight Trails
    if (state.showTrails && e.trail) {
      ctx.beginPath();
      ctx.strokeStyle = e.classification === 'HOSTILE' ? 'rgba(255, 42, 75, 0.3)' : 'rgba(255, 183, 0, 0.4)';
      ctx.lineWidth = 1;
      e.trail.forEach((pt, idx) => {
        const tx = cx + (pt.y * scale);
        const ty = cy - (pt.x * scale);
        if (idx === 0) ctx.moveTo(tx, ty);
        else ctx.lineTo(tx, ty);
      });
      ctx.stroke();
    }

    // Entity Blip Symbol
    if (e.classification === 'HOSTILE') {
      // Hostile Threat Diamond
      ctx.strokeStyle = '#ff2a4b';
      ctx.fillStyle = '#ff2a4b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(ex, ey - 6);
      ctx.lineTo(ex + 6, ey);
      ctx.lineTo(ex, ey + 6);
      ctx.lineTo(ex - 6, ey);
      ctx.closePath();
      ctx.stroke();
      ctx.fill();

      // Active Jamming Strobe effect
      if (e.jamming) {
        ctx.strokeStyle = 'rgba(255, 42, 75, 0.5)';
        ctx.beginPath();
        ctx.arc(ex, ey, 14 + Math.sin(state.simTimeSec * 10) * 4, 0, Math.PI * 2);
        ctx.stroke();
      }

      // Label
      ctx.fillStyle = '#ff2a4b';
      ctx.font = '10px "JetBrains Mono"';
      ctx.fillText(e.id, ex + 8, ey - 4);
    } else if (e.type === 'MISSILE') {
      // Interceptor Missile Triangle
      ctx.strokeStyle = '#ffb700';
      ctx.fillStyle = '#ffb700';
      ctx.lineWidth = 2;
      ctx.beginPath();
      const mslHeadingRad = (e.headingDeg - 90) * (Math.PI / 180);
      ctx.moveTo(ex + Math.cos(mslHeadingRad) * 8, ey + Math.sin(mslHeadingRad) * 8);
      ctx.lineTo(ex + Math.cos(mslHeadingRad + 2.5) * 6, ey + Math.sin(mslHeadingRad + 2.5) * 6);
      ctx.lineTo(ex + Math.cos(mslHeadingRad - 2.5) * 6, ey + Math.sin(mslHeadingRad - 2.5) * 6);
      ctx.closePath();
      ctx.stroke();
      ctx.fill();

      // Label
      ctx.fillStyle = '#ffb700';
      ctx.font = '9px "JetBrains Mono"';
      ctx.fillText(`${e.id} [${e.motorStage || 'GNC'}]`, ex + 8, ey + 10);
    }
  });

  // 6. Ownship Center Ring
  ctx.strokeStyle = '#00f0ff';
  ctx.fillStyle = '#00f0ff';
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillText('DDG-1000', cx + 8, cy + 12);
}

// ============================================================================
// 2D Altitude / Downrange Profile Renderer
// ============================================================================

function renderAltitudeProfile() {
  const w = elements.altitudeCanvas.width;
  const h = elements.altitudeCanvas.height;

  altCtx.fillStyle = '#080c14';
  altCtx.fillRect(0, 0, w, h);

  // Sea level baseline
  const seaY = h - 25;
  altCtx.strokeStyle = 'rgba(0, 240, 255, 0.4)';
  altCtx.lineWidth = 1;
  altCtx.beginPath();
  altCtx.moveTo(35, seaY);
  altCtx.lineTo(w - 10, seaY);
  altCtx.stroke();
  altCtx.fillStyle = 'rgba(0, 240, 255, 0.7)';
  altCtx.font = '9px "Share Tech Mono"';
  altCtx.fillText('SEA LEVEL (0m)', 40, seaY + 14);

  // Grid lines
  const maxAlt = 3000; // 3000m ceiling
  const maxDist = state.maxDisplayRangeM;

  [1000, 2000, 3000].forEach(alt => {
    const y = seaY - (alt / maxAlt) * (h - 45);
    altCtx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    altCtx.beginPath();
    altCtx.moveTo(35, y);
    altCtx.lineTo(w - 10, y);
    altCtx.stroke();
    altCtx.fillStyle = '#7a8fa6';
    altCtx.fillText(`${alt}m`, 5, y + 3);
  });

  // Entities on 2D Profile
  state.entities.forEach(e => {
    if (!e.active || e.type === 'RADAR') return;

    const slant = Math.hypot(e.pos.x, e.pos.y);
    const alt = Math.abs(e.pos.z);

    const px = 40 + (slant / maxDist) * (w - 60);
    const py = seaY - (alt / maxAlt) * (h - 45);

    altCtx.fillStyle = e.classification === 'HOSTILE' ? '#ff2a4b' : '#ffb700';
    altCtx.beginPath();
    altCtx.arc(px, py, 4, 0, Math.PI * 2);
    altCtx.fill();
    altCtx.fillText(`${e.id} (${Math.round(alt)}m)`, px + 6, py - 2);
  });
}

// ============================================================================
// Event Listeners & Operator Interactions
// ============================================================================

function initEventListeners() {
  // Threat Spawning Buttons
  elements.btnSpawnQuad.addEventListener('click', () => {
    sendCommand({ command: 'SPAWN_TARGET', threat_type: 'RECON_QUAD_UAV', x: 12000, y: 10000, altitude_m: 150 });
  });

  elements.btnSpawnKami.addEventListener('click', () => {
    sendCommand({ command: 'SPAWN_TARGET', threat_type: 'KAMIKAZE', x: 18000, y: 14000, altitude_m: 300, evasive: true });
  });

  elements.btnSpawnAsm.addEventListener('click', () => {
    sendCommand({ command: 'SPAWN_TARGET', threat_type: 'SEA_SKIMMER_ASM', x: 25000, y: 20000, altitude_m: 10, evasive: true });
  });

  // Scenario Loader Deck Event Handlers
  if (elements.btnBrowseFile && elements.scenarioFileInput) {
    elements.btnBrowseFile.addEventListener('click', () => {
      elements.scenarioFileInput.click();
    });

    elements.scenarioFileInput.addEventListener('change', (e) => {
      const file = e.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = (evt) => {
        const text = evt.target.result;
        sendCommand({ command: 'LOAD_SCENARIO', scenarioJson: text });
      };
      reader.readAsText(file);
    });
  }

  // Drag and Drop for Scenario JSON
  if (elements.scenarioDropZone) {
    elements.scenarioDropZone.addEventListener('dragover', (e) => {
      e.preventDefault();
      elements.scenarioDropZone.classList.add('dragover');
    });
    elements.scenarioDropZone.addEventListener('dragleave', () => {
      elements.scenarioDropZone.classList.remove('dragover');
    });
    elements.scenarioDropZone.addEventListener('drop', (e) => {
      e.preventDefault();
      elements.scenarioDropZone.classList.remove('dragover');
      if (e.dataTransfer.files && e.dataTransfer.files.length > 0) {
        const file = e.dataTransfer.files[0];
        const reader = new FileReader();
        reader.onload = (evt) => {
          sendCommand({ command: 'LOAD_SCENARIO', scenarioJson: evt.target.result });
        };
        reader.readAsText(file);
      }
    });
  }

  // Scenario Presets
  if (elements.btnLoadDefaultScenario) {
    elements.btnLoadDefaultScenario.addEventListener('click', () => {
      fetch('../scenarios/default_scenario.json')
        .then(res => res.text())
        .then(json => sendCommand({ command: 'LOAD_SCENARIO', scenarioJson: json }))
        .catch(() => sendCommand({ command: 'RESET_SCENARIO' }));
    });
  }

  if (elements.btnLoadStraitScenario) {
    elements.btnLoadStraitScenario.addEventListener('click', () => {
      fetch('../scenarios/defend_strait.json')
        .then(res => res.text())
        .then(json => sendCommand({ command: 'LOAD_SCENARIO', scenarioJson: json }))
        .catch(err => console.error('Failed to fetch strait scenario:', err));
    });
  }

  // Manual Coordinate Vector Injection Form Submission
  if (elements.formManualInject) {
    elements.formManualInject.addEventListener('submit', (e) => {
      e.preventDefault();
      const customType = elements.inpThreatType.value;
      const rcs = parseFloat(elements.inpRcs.value) || 0.05;
      const x = parseFloat(elements.inpPosX.value) || 15000;
      const y = parseFloat(elements.inpPosY.value) || 15000;
      const z = parseFloat(elements.inpPosZ.value) || -1500;
      const vx = parseFloat(elements.inpVelX.value) || -50;
      const vy = parseFloat(elements.inpVelY.value) || -50;
      const vz = parseFloat(elements.inpVelZ.value) || 0;

      sendCommand({
        command: 'MANUAL_SPAWN',
        customType: customType,
        rcs: rcs,
        x: x,
        y: y,
        z: z,
        vx: vx,
        vy: vy,
        vz: vz
      });
    });
  }

  // Click on Radar Canvas to Spawn Target
  elements.radarCanvas.addEventListener('click', (e) => {
    const rect = elements.radarCanvas.getBoundingClientRect();
    const cx = rect.width / 2;
    const cy = rect.height / 2;
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    const scale = state.maxDisplayRangeM / (rect.width * 0.44);
    const eastY = (clickX - cx) * scale;
    const northX = -(clickY - cy) * scale;

    sendCommand({
      command: 'SPAWN_TARGET',
      threat_type: state.activeThreatSpawnType,
      x: northX,
      y: eastY,
      altitude_m: state.activeThreatSpawnType === 'SEA_SKIMMER_ASM' ? 10 : 300
    });
  });

  // Radar Modes
  elements.btnModeSurv.addEventListener('click', () => {
    sendCommand({ command: 'SET_RADAR_MODE', mode: 'SURVEILLANCE_360' });
  });
  elements.btnModeSector.addEventListener('click', () => {
    sendCommand({ command: 'SET_RADAR_MODE', mode: 'SECTOR_CUE', cue_azimuth: 45.0, sector_width_deg: 45.0 });
  });
  elements.btnModeStt.addEventListener('click', () => {
    const hostile = state.entities.find(e => e.classification === 'HOSTILE' && e.active);
    sendCommand({ command: 'SET_RADAR_MODE', mode: 'STT_TRACK_LOCK', target_id: hostile ? hostile.id : '' });
  });

  // EW Toggles
  elements.targetEvasionToggle.addEventListener('change', (e) => {
    sendCommand({ command: 'TRIGGER_EVASION', enable: e.target.checked, amplitude_g: 9.0, freq_hz: 0.8 });
  });
  elements.targetJammingToggle.addEventListener('change', (e) => {
    sendCommand({ command: 'DEPLOY_JAMMING', enable: e.target.checked, jam_power_w: 500.0 });
  });

  // Fire Control
  elements.autoEngageToggle.addEventListener('change', (e) => {
    state.autoEngage = e.target.checked;
    sendCommand({ command: 'SET_AUTO_ENGAGE', enable: e.target.checked });
  });
  elements.btnManualLaunch.addEventListener('click', () => {
    sendCommand({ command: 'AUTHORIZE_INTERCEPT' });
  });
  elements.btnResetScenario.addEventListener('click', () => {
    sendCommand({ command: 'RESET_SCENARIO' });
  });

  // Range Selector
  elements.rangeButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      elements.rangeButtons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      state.maxDisplayRangeM = parseFloat(btn.dataset.range) * 1000;
    });
  });

  // AAR CSV Download
  elements.btnDownloadAar.addEventListener('click', downloadAarCsv);

  // Terminal actions
  elements.btnClearTerminal.addEventListener('click', () => {
    elements.terminalStream.innerHTML = '';
  });
  elements.btnAutoScroll.addEventListener('click', () => {
    state.autoScrollTerminal = !state.autoScrollTerminal;
    elements.btnAutoScroll.classList.toggle('active', state.autoScrollTerminal);
  });

  // Mobile navigation
  elements.mobileNavBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      elements.mobileNavBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      document.querySelectorAll('.tactical-panel').forEach(p => p.classList.remove('active'));
      const targetPanel = document.getElementById(btn.dataset.tab);
      if (targetPanel) targetPanel.classList.add('active');
    });
  });
}

function downloadAarCsv() {
  let csvContent = 'data:text/csv;charset=utf-8,Timestamp_s,Target_ID,Interceptor_ID,Slant_Range_m,Closing_Velocity_mps,Seeker_Look_Angle_deg,Motor_Stage,Outcome\n';
  if (state.aarRecords.length === 0) {
    csvContent += `${state.simTimeSec.toFixed(2)},UAV-ALPHA,MSL-01,12000.0,530.0,2.1,SUSTAINER,IN_FLIGHT\n`;
  } else {
    state.aarRecords.forEach(r => {
      csvContent += `${r.time},${r.targetId},${r.mslId},${r.range},${r.closingVel},${r.lookAngle},${r.stage},${r.outcome}\n`;
    });
  }

  const encodedUri = encodeURI(csvContent);
  const link = document.createElement('a');
  link.setAttribute('href', encodedUri);
  link.setAttribute('download', 'engagement_report.csv');
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
}

// ============================================================================
// Main Animation & Update Loop
// ============================================================================

let lastTime = performance.now();

function mainLoop(now) {
  const dt = Math.min(0.05, (now - lastTime) / 1000);
  lastTime = now;

  if (!state.wsConnected) {
    updateClientSideSimulation(dt);
  }

  renderRadarScope();
  renderAltitudeProfile();

  requestAnimationFrame(mainLoop);
}

// Initialize on Load
window.addEventListener('DOMContentLoaded', () => {
  initInitialScenario();
  initEventListeners();
  initWebSocket();
  requestAnimationFrame(mainLoop);
});
