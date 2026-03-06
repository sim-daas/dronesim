/**
 * Surveillance Drone System — GCS Dashboard JavaScript
 *
 * Polls fleet telemetry every second, updates DOM, and dispatches
 * control commands to the Flask backend.
 */

// ─── Configuration ─────────────────────────────────────────────────────────

const POLL_INTERVAL_MS = 1000;
const LOG_POLL_INTERVAL_MS = 2000;
let logIndex = 0;

// ─── Telemetry Polling ─────────────────────────────────────────────────────

async function pollFleet() {
    try {
        const resp = await fetch('/api/fleet');
        if (!resp.ok) throw new Error(`HTTP ${resp.status}`);

        const data = await resp.json();
        const drones = data.drones;

        let connectedCount = 0;

        for (const [droneId, telem] of Object.entries(drones)) {
            updateDroneCard(parseInt(droneId), telem);
            if (telem.connected && telem.reachable) connectedCount++;
        }

        // Update header
        document.getElementById('fleet-count').textContent =
            `${connectedCount}/${Object.keys(drones).length}`;
        document.getElementById('last-update').textContent =
            new Date().toLocaleTimeString('en-GB', { hour12: false });

        // Connection indicator
        const dot = document.getElementById('connection-dot');
        dot.classList.toggle('active', connectedCount > 0);

    } catch (err) {
        console.error('Fleet poll error:', err);
        const dot = document.getElementById('connection-dot');
        dot.classList.remove('active');
    }
}

function updateDroneCard(droneId, telem) {
    const card = document.getElementById(`drone-card-${droneId}`);
    if (!card) return;

    const connected = telem.connected && telem.reachable;
    card.classList.toggle('connected', connected);

    // Connection badge
    const connBadge = document.getElementById(`drone-${droneId}-conn`);
    connBadge.textContent = connected ? 'ONLINE' : (telem.reachable ? 'NO PX4' : 'OFFLINE');
    connBadge.classList.toggle('online', connected);

    // Armed badge
    const armedBadge = document.getElementById(`drone-${droneId}-armed-badge`);
    armedBadge.textContent = telem.armed ? 'ARMED' : 'DISARMED';
    armedBadge.classList.toggle('armed', telem.armed);

    // Telemetry values
    setTelem(droneId, 'lat', connected ? telem.lat?.toFixed(6) + '°' : '--');
    setTelem(droneId, 'lon', connected ? telem.lon?.toFixed(6) + '°' : '--');
    setTelem(droneId, 'alt', connected ? telem.alt?.toFixed(1) + ' m' : '--');
    setTelem(droneId, 'speed', connected ? telem.ground_speed?.toFixed(1) + ' m/s' : '--');
    setTelem(droneId, 'mode', connected ? telem.flight_mode : '--');
    setTelem(droneId, 'heading', connected ? telem.heading_deg?.toFixed(0) + '°' : '--');

    // Battery
    const batPercent = telem.battery_percent || 0;
    setTelem(droneId, 'battery', connected ? `${batPercent.toFixed(0)}%` : '--');

    const batBar = document.getElementById(`drone-${droneId}-battery-bar`);
    if (batBar) {
        batBar.style.width = connected ? `${Math.min(100, batPercent)}%` : '0%';
        batBar.classList.remove('low', 'critical');
        if (batPercent < 15) batBar.classList.add('critical');
        else if (batPercent < 30) batBar.classList.add('low');
    }

    // GPS
    const gpsText = connected
        ? `${telem.gps_fix_type} (${telem.gps_num_satellites})`
        : '--';
    setTelem(droneId, 'gps', gpsText);
}

function setTelem(droneId, field, value) {
    const el = document.getElementById(`drone-${droneId}-${field}`);
    if (el) el.textContent = value;
}

// ─── Control Commands ──────────────────────────────────────────────────────

async function sendCommand(droneId, action) {
    addConsoleLog(`Sending '${action}' to Drone ${droneId}...`, 'info');

    try {
        const resp = await fetch(`/api/drone/${droneId}/${action}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
        });
        const result = await resp.json();

        if (result.success) {
            addConsoleLog(`✓ Drone ${droneId} ${action}: ${result.message}`, 'success');
        } else {
            addConsoleLog(`✗ Drone ${droneId} ${action}: ${result.message}`, 'error');
        }
    } catch (err) {
        addConsoleLog(`✗ Drone ${droneId} ${action} failed: ${err.message}`, 'error');
    }
}

async function sendTakeoff(droneId) {
    const altitude = prompt('Takeoff altitude (meters):', '5');
    if (altitude === null) return;

    const alt = parseFloat(altitude);
    if (isNaN(alt) || alt <= 0 || alt > 100) {
        addConsoleLog('Invalid altitude value', 'error');
        return;
    }

    addConsoleLog(`Sending takeoff to ${alt}m to Drone ${droneId}...`, 'info');

    try {
        const resp = await fetch(`/api/drone/${droneId}/takeoff`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ altitude: alt }),
        });
        const result = await resp.json();

        if (result.success) {
            addConsoleLog(`✓ Drone ${droneId} takeoff to ${alt}m: ${result.message}`, 'success');
        } else {
            addConsoleLog(`✗ Drone ${droneId} takeoff: ${result.message}`, 'error');
        }
    } catch (err) {
        addConsoleLog(`✗ Drone ${droneId} takeoff failed: ${err.message}`, 'error');
    }
}

// ─── Console ───────────────────────────────────────────────────────────────

function addConsoleLog(message, type = 'info') {
    const consoleEl = document.getElementById('console-output');
    if (!consoleEl) return;

    const timestamp = new Date().toLocaleTimeString('en-GB', { hour12: false });
    const entry = document.createElement('div');
    entry.className = `log-entry log-${type}`;
    entry.textContent = `[${timestamp}] ${message}`;

    consoleEl.appendChild(entry);

    // Auto-scroll to bottom
    consoleEl.scrollTop = consoleEl.scrollHeight;

    // Limit entries
    while (consoleEl.children.length > 300) {
        consoleEl.removeChild(consoleEl.firstChild);
    }
}

function clearConsole() {
    const consoleEl = document.getElementById('console-output');
    if (consoleEl) consoleEl.innerHTML = '';
}

async function pollLogs() {
    try {
        const resp = await fetch(`/api/logs?since=${logIndex}`);
        if (!resp.ok) return;
        const data = await resp.json();
        if (data.logs && data.logs.length > 0) {
            for (const msg of data.logs) {
                let type = 'info';
                if (msg.includes('✓')) type = 'success';
                else if (msg.includes('✗') || msg.includes('Error')) type = 'error';
                addConsoleLog(msg, type);
            }
            logIndex += data.logs.length;
        }
    } catch (err) {
        // Silently ignore log poll errors
    }
}

// ─── Initialize ────────────────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
    addConsoleLog('GCS Dashboard initialized', 'info');
    addConsoleLog('Starting fleet telemetry polling...', 'info');

    // Start polling
    pollFleet();
    setInterval(pollFleet, POLL_INTERVAL_MS);

    // Start log polling
    setInterval(pollLogs, LOG_POLL_INTERVAL_MS);
});
