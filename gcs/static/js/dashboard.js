/**
 * Surveillance Drone System — GCS Dashboard JavaScript
 *
 * Polls fleet telemetry every second, updates DOM, and dispatches
 * control commands to the Flask backend.
 */

// ─── Configuration ─────────────────────────────────────────────────────────

const POLL_INTERVAL_MS = 200;
const LOG_POLL_INTERVAL_MS = 2000;
let logIndex = 0;

// ─── Map & Mission State ───────────────────────────────────────────────────

let map = null;
let missionWaypoints = [];
let missionPolyline = null;
let droneMarkers = {};
let mapBounds = [[0, 0], [1000, 1000]]; // Arbitrary bounds for image overlay

// ─── Custom UI Helpers ─────────────────────────────────────────────────────

function toggleDroneSelect() {
    const opts = document.getElementById('drone-select-options');
    if (opts) opts.classList.toggle('open');
}

document.addEventListener('click', (e) => {
    if (!e.target.closest('.custom-multi-select')) {
        const opts = document.getElementById('drone-select-options');
        if (opts && opts.classList.contains('open')) {
            opts.classList.remove('open');
        }
    }
});

// Basic coordinate scaling from lat/lon to map pixels (to match simulator)
// Since we use an ImageOverlay instead of real geo-tiles, we map simulated 
// lat/lon coordinates (e.g. 47.3977) to the [0, 1000] pixel bounds.
const SIM_HOME_LAT = 47.397742;
const SIM_HOME_LON = 8.545594;
const SCALE_FACTOR = 100000; // Degrees to arbitrary pixels

function geoToPixel(lat, lon) {
    const y = 500 + (lat - SIM_HOME_LAT) * SCALE_FACTOR;
    const x = 500 + (lon - SIM_HOME_LON) * SCALE_FACTOR;
    return [y, x];
}

function pixelToGeo(y, x) {
    const lat = SIM_HOME_LAT + (y - 500) / SCALE_FACTOR;
    const lon = SIM_HOME_LON + (x - 500) / SCALE_FACTOR;
    return { lat, lon };
}

// ─── Initialization ────────────────────────────────────────────────────────

function initMap() {
    // Create map with arbitrary CRS for image overlay
    map = L.map('map', {
        crs: L.CRS.Simple,
        center: [500, 500],
        zoom: 0
    });

    const imageUrl = '/static/images/back.png';
    L.imageOverlay(imageUrl, mapBounds).addTo(map);
    map.fitBounds(mapBounds);

    // Mission polyline
    missionPolyline = L.polyline([], { color: '#1E90FF', weight: 3, dashArray: '5, 10' }).addTo(map);

    map.on('click', onMapClick);
    addConsoleLog('Map initialized with custom backdrop', 'success');
}

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

    // Update Map Marker
    if (connected && telem.lat && telem.lon) {
        updateMapMarker(droneId, telem);
    }
}

function updateMapMarker(droneId, telem) {
    if (!map) return;

    const pos = geoToPixel(telem.lat, telem.lon);

    // Create colored icon based on drone ID
    const colors = ['#1E90FF', '#00FF85', '#FF6F61', '#9b59b6', '#e67e22'];
    const color = colors[(droneId - 1) % colors.length];

    if (!droneMarkers[droneId]) {
        const iconHtml = `
            <div style="background-color: ${color}; width: 16px; height: 16px; border-radius: 50%; 
                        border: 2px solid white; box-shadow: 0 0 10px ${color}; 
                        display: flex; align-items: center; justify-content: center;
                        font-family: inherit; font-size: 10px; font-weight: bold; color: white;">
                ${droneId}
            </div>
        `;

        const icon = L.divIcon({
            className: 'custom-drone-icon',
            html: iconHtml,
            iconSize: [16, 16],
            iconAnchor: [8, 8]
        });

        droneMarkers[droneId] = L.marker(pos, { icon: icon })
            .bindTooltip(`Drone ${droneId}<br>Alt: ${telem.alt.toFixed(1)}m<br>Bat: ${telem.battery_percent.toFixed(0)}%`)
            .addTo(map);
    } else {
        droneMarkers[droneId].setLatLng(pos);
        droneMarkers[droneId].getTooltip().setContent(`Drone ${droneId}<br>Alt: ${telem.alt.toFixed(1)}m<br>Bat: ${telem.battery_percent.toFixed(0)}%`);
    }
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

// ─── Mission Planning ──────────────────────────────────────────────────────

function onMapClick(e) {
    const latlng = e.latlng;

    // Convert back from pixel space to pseudo-geo coordinates for PX4
    const geo = pixelToGeo(latlng.lat, latlng.lng);

    // Draw on map using pixel latlng
    missionPolyline.addLatLng(latlng);

    // Store actual geo coordinates for the drone
    const altInput = parseFloat(document.getElementById('mission-alt').value);
    missionWaypoints.push({
        lat: geo.lat,
        lon: geo.lon,
        alt: !isNaN(altInput) ? altInput : 10.0
    });

    updateMissionStats();
}

function updateMissionStats() {
    document.getElementById('wp-count').textContent = `${missionWaypoints.length} WPs`;

    if (missionWaypoints.length < 2) {
        document.getElementById('mission-dist').textContent = "0.0 km";
        return;
    }

    let dist = 0;
    // VERY rough distance estimation (Pythagorean on lat/lon)
    for (let i = 1; i < missionWaypoints.length; i++) {
        const w1 = missionWaypoints[i - 1];
        const w2 = missionWaypoints[i];
        const dLat = (w2.lat - w1.lat) * 111320; // rough meters per degree lat
        const dLon = (w2.lon - w1.lon) * 111320 * Math.cos(w1.lat * (Math.PI / 180));
        dist += Math.sqrt(dLat * dLat + dLon * dLon);
    }

    document.getElementById('mission-dist').textContent = `${(dist / 1000).toFixed(2)} km`;
}

function clearMission() {
    missionWaypoints = [];
    if (missionPolyline) missionPolyline.setLatLngs([]);
    updateMissionStats();
    addConsoleLog('Mission waypoints cleared', 'info');
}

async function uploadMission() {
    if (missionWaypoints.length === 0) {
        alert("Please click on the map to add waypoints first.");
        return;
    }

    const checkboxes = document.querySelectorAll('.drone-cb:checked');
    if (checkboxes.length === 0) {
        alert("Please select at least one drone from the dropdown.");
        return;
    }

    const speedInput = parseFloat(document.getElementById('mission-speed').value);
    const speed = !isNaN(speedInput) ? speedInput : 5.0;

    const droneIds = Array.from(checkboxes).map(cb => parseInt(cb.value));
    addConsoleLog(`Uploading ${missionWaypoints.length} waypoints to Drones: ${droneIds.join(', ')}...`, 'info');

    for (const droneId of droneIds) {
        try {
            const resp = await fetch(`/api/drone/${droneId}/mission/upload`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    waypoints: missionWaypoints,
                    speed: speed
                }),
            });
            const result = await resp.json();

            if (result.success) {
                addConsoleLog(`✓ Mission uploaded to Drone ${droneId}`, 'success');
            } else {
                addConsoleLog(`✗ Drone ${droneId} upload failed: ${result.message}`, 'error');
            }
        } catch (err) {
            addConsoleLog(`✗ Drone ${droneId} request failed: ${err.message}`, 'error');
        }
    }
}

function startMission() {
    const checkboxes = document.querySelectorAll('.drone-cb:checked');
    if (checkboxes.length === 0) return;
    for (const cb of checkboxes) {
        sendCommand(parseInt(cb.value), 'mission/start');
    }
}

function pauseMission() {
    const checkboxes = document.querySelectorAll('.drone-cb:checked');
    if (checkboxes.length === 0) return;
    for (const cb of checkboxes) {
        sendCommand(parseInt(cb.value), 'mission/pause');
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
    initMap();
    addConsoleLog('GCS Dashboard initialized', 'info');
    addConsoleLog('Starting fleet telemetry polling...', 'info');

    // Start polling
    pollFleet();
    setInterval(pollFleet, POLL_INTERVAL_MS);

    // Start log polling
    setInterval(pollLogs, LOG_POLL_INTERVAL_MS);
});
