/**
 * Surveillance Drone System — GCS Dashboard JavaScript
 *
 * Polls fleet telemetry every second, updates DOM, and dispatches
 * control commands to the Flask backend.
 * Phase 4: Mission mode selector, area drawing, auto-tiling.
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
let mapBounds = [[0, 0], [1000, 1000]];

// ─── Mission Mode State ────────────────────────────────────────────────────

let missionMode = 'waypoints'; // 'waypoints' | 'rectangle' | 'circle'
let areaClicks = [];            // Temporary click storage for area definition
let drawnShape = null;          // Leaflet shape on map (rectangle or circle)
let previewPolylines = [];      // Array of Leaflet polylines for tiling preview
let tilingResult = null;        // Cached result from /api/mission/tile

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

// ─── Coordinate System ─────────────────────────────────────────────────────

// Center of the Gazebo world (0,0) as defined in default.sdf
const GAZEBO_ORIGIN_LAT = 47.397971057728974;
const GAZEBO_ORIGIN_LON = 8.546163739800146;

// The ground image (heightmap) is offset from Gazebo (0,0) by this pose in model.sdf:
// <pose>141.97 -141.17 -137.94 0 0 0</pose> -> Image Center is at X=141.97, Y=-141.17.
// Knowing this offset properly maps the center of our Leaflet image to the correct GPS origin.
const MAP_OFFSET_X = 141.97;
const MAP_OFFSET_Y = -141.17;

// Actual Map Dimensions. Using 4514.0 for the true orthophoto square scaling
const GAZEBO_WORLD_METERS_X = 4514.0;
const GAZEBO_WORLD_METERS_Y = 4514.0;

const MAP_BOUNDS_PIXELS = 1000.0;
const PIXELS_PER_METER_X = MAP_BOUNDS_PIXELS / GAZEBO_WORLD_METERS_X;
const PIXELS_PER_METER_Y = MAP_BOUNDS_PIXELS / GAZEBO_WORLD_METERS_Y;

// PX4 uses exactly R=6371000 for local WGS84 Cartesian projections
const METERS_PER_DEGREE_LAT = 111194.9266;
// We calculate the map center coordinates to anchor our Leaflet image at 500,500
const SIM_HOME_LAT = GAZEBO_ORIGIN_LAT + (MAP_OFFSET_Y / METERS_PER_DEGREE_LAT);
const METERS_PER_DEGREE_LON = 111194.9266 * Math.cos(SIM_HOME_LAT * (Math.PI / 180));
const SIM_HOME_LON = GAZEBO_ORIGIN_LON + (MAP_OFFSET_X / METERS_PER_DEGREE_LON);

function geoToPixel(lat, lon) {
    const latDiffMeters = (lat - SIM_HOME_LAT) * METERS_PER_DEGREE_LAT;
    const lonDiffMeters = (lon - SIM_HOME_LON) * METERS_PER_DEGREE_LON;
    const y = 500 + latDiffMeters * PIXELS_PER_METER_Y;
    const x = 500 + lonDiffMeters * PIXELS_PER_METER_X;
    return [y, x];
}

function pixelToGeo(y, x) {
    const meterYDiff = (y - 500) / PIXELS_PER_METER_Y;
    const meterXDiff = (x - 500) / PIXELS_PER_METER_X;
    const lat = SIM_HOME_LAT + (meterYDiff / METERS_PER_DEGREE_LAT);
    const lon = SIM_HOME_LON + (meterXDiff / METERS_PER_DEGREE_LON);
    return { lat, lon };
}

// ─── Mission Mode ──────────────────────────────────────────────────────────

function onMissionModeChange() {
    missionMode = document.getElementById('mission-mode').value;
    clearArea();
    clearMission();

    const waypointCtrl = document.getElementById('waypoint-controls');
    const tilingCtrl = document.getElementById('tiling-controls');
    const tilingInputs = document.querySelectorAll('.tiling-only');
    const areaSpan = document.getElementById('mission-area');

    if (missionMode === 'waypoints') {
        waypointCtrl.style.display = '';
        tilingCtrl.style.display = 'none';
        tilingInputs.forEach(el => el.style.display = 'none');
        areaSpan.style.display = 'none';
    } else {
        waypointCtrl.style.display = 'none';
        tilingCtrl.style.display = '';
        tilingInputs.forEach(el => el.style.display = '');
        areaSpan.style.display = '';
    }

    const modeLabels = { waypoints: 'Waypoint', rectangle: 'Rectangle', circle: 'Circle' };
    addConsoleLog(`Mission mode switched to: ${modeLabels[missionMode]}`, 'info');
}

// ─── Initialization ────────────────────────────────────────────────────────

function initMap() {
    map = L.map('map', {
        crs: L.CRS.Simple,
        center: [500, 500],
        zoom: 0
    });

    const imageUrl = '/static/images/back.png';
    L.imageOverlay(imageUrl, mapBounds).addTo(map);
    map.fitBounds(mapBounds);

    // Mission polyline (for waypoint mode)
    missionPolyline = L.polyline([], { color: '#1E90FF', weight: 3, dashArray: '5, 10' }).addTo(map);

    map.on('click', onMapClick);
    addConsoleLog('Map initialized with custom backdrop', 'success');
}

// ─── Map Click Handler ─────────────────────────────────────────────────────

function onMapClick(e) {
    const latlng = e.latlng;
    const geo = pixelToGeo(latlng.lat, latlng.lng);

    if (missionMode === 'waypoints') {
        // Original manual waypoint mode
        missionPolyline.addLatLng(latlng);
        const altInput = parseFloat(document.getElementById('mission-alt').value);
        missionWaypoints.push({
            lat: geo.lat,
            lon: geo.lon,
            alt: !isNaN(altInput) ? altInput : 10.0
        });
        updateMissionStats();

    } else if (missionMode === 'rectangle') {
        areaClicks.push({ pixel: latlng, geo: geo });
        if (areaClicks.length === 1) {
            addConsoleLog('Rectangle: First corner set. Click second corner.', 'info');
            // Draw a temporary marker
            L.circleMarker(latlng, { radius: 5, color: '#FF8C00', fillOpacity: 0.8 }).addTo(map);
        }
        if (areaClicks.length === 2) {
            drawRectangle();
        }

    } else if (missionMode === 'circle') {
        areaClicks.push({ pixel: latlng, geo: geo });
        if (areaClicks.length === 1) {
            addConsoleLog('Circle: Center set. Click edge to define radius.', 'info');
            L.circleMarker(latlng, { radius: 5, color: '#FF8C00', fillOpacity: 0.8 }).addTo(map);
        }
        if (areaClicks.length === 2) {
            drawCircle();
        }
    }
}

// ─── Area Drawing ──────────────────────────────────────────────────────────

function drawRectangle() {
    const c1 = areaClicks[0].pixel;
    const c2 = areaClicks[1].pixel;

    if (drawnShape) map.removeLayer(drawnShape);
    drawnShape = L.rectangle([c1, c2], {
        color: '#FF8C00',
        weight: 2,
        fillColor: '#FF8C00',
        fillOpacity: 0.15,
        dashArray: '6, 4'
    }).addTo(map);

    // Calculate area
    const g1 = areaClicks[0].geo;
    const g2 = areaClicks[1].geo;
    const dx = Math.abs(g2.lon - g1.lon) * METERS_PER_DEGREE_LON;
    const dy = Math.abs(g2.lat - g1.lat) * METERS_PER_DEGREE_LAT;
    const area = dx * dy;

    document.getElementById('mission-area').textContent = `${area.toFixed(0)} m²`;
    addConsoleLog(`Rectangle drawn: ${dx.toFixed(0)}m × ${dy.toFixed(0)}m = ${area.toFixed(0)} m²`, 'success');
}

function drawCircle() {
    const center = areaClicks[0].pixel;
    const edge = areaClicks[1].pixel;

    // Radius in pixels
    const dx = edge.lng - center.lng;
    const dy = edge.lat - center.lat;
    const radiusPixels = Math.sqrt(dx * dx + dy * dy);

    // Radius in meters
    const radiusMeters = radiusPixels / ((PIXELS_PER_METER_X + PIXELS_PER_METER_Y) / 2);

    if (drawnShape) map.removeLayer(drawnShape);
    drawnShape = L.circle(center, {
        radius: radiusPixels,  // CRS.Simple uses pixel units
        color: '#FF8C00',
        weight: 2,
        fillColor: '#FF8C00',
        fillOpacity: 0.15,
        dashArray: '6, 4'
    }).addTo(map);

    const area = Math.PI * radiusMeters * radiusMeters;
    document.getElementById('mission-area').textContent = `${area.toFixed(0)} m²`;
    addConsoleLog(`Circle drawn: radius=${radiusMeters.toFixed(0)}m, area=${area.toFixed(0)} m²`, 'success');
}

// ─── Tiling Preview & Upload ───────────────────────────────────────────────

async function generatePreview() {
    if (areaClicks.length < 2) {
        addConsoleLog('Draw an area first by clicking 2 points on the map.', 'error');
        return;
    }

    const altitude = parseFloat(document.getElementById('mission-alt').value) || 10;
    const speed = parseFloat(document.getElementById('mission-speed').value) || 5;
    const sweepWidth = parseFloat(document.getElementById('sweep-width').value) || 20;

    let body = {
        altitude: altitude,
        speed: speed,
        sweep_width: sweepWidth,
        angle: 0,
    };

    if (missionMode === 'rectangle') {
        body.shape = 'rectangle';
        body.corners = [
            [areaClicks[0].geo.lat, areaClicks[0].geo.lon],
            [areaClicks[1].geo.lat, areaClicks[1].geo.lon],
        ];
    } else if (missionMode === 'circle') {
        const g1 = areaClicks[0].geo;
        const g2 = areaClicks[1].geo;
        const dx = (g2.lon - g1.lon) * METERS_PER_DEGREE_LON;
        const dy = (g2.lat - g1.lat) * METERS_PER_DEGREE_LAT;
        const radiusM = Math.sqrt(dx * dx + dy * dy);
        body.shape = 'circle';
        body.center = [g1.lat, g1.lon];
        body.radius_m = radiusM;
    }

    addConsoleLog('Generating tiling preview...', 'info');

    try {
        const resp = await fetch('/api/mission/tile', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body),
        });
        const result = await resp.json();

        if (!result.success) {
            addConsoleLog(`✗ Tiling failed: ${result.message}`, 'error');
            return;
        }

        tilingResult = result;

        // Clear old previews
        clearPreviewPolylines();

        // Draw each drone's path in a different color
        const colors = ['#FF8C00', '#00FFAA', '#FF5577', '#AA88FF'];
        result.assignments.forEach((assignment, idx) => {
            const wps = assignment.waypoints;
            const pixelCoords = wps.map(wp => geoToPixel(wp.lat, wp.lon));
            const poly = L.polyline(pixelCoords, {
                color: colors[idx % colors.length],
                weight: 2,
                dashArray: '4, 8',
                opacity: 0.9,
            }).addTo(map);
            previewPolylines.push(poly);
        });

        // Update stats
        const tilingInfo = document.getElementById('tiling-info');
        tilingInfo.style.display = '';
        document.getElementById('tile-area-info').textContent = `Area: ${result.area_m2} m²`;
        document.getElementById('tile-path-info').textContent = `Path: ${(result.total_distance_m / 1000).toFixed(2)} km (${result.total_waypoints} WPs)`;
        document.getElementById('tile-drone-info').textContent = result.num_drones > 0
            ? `Drones: ${result.assignments.map(a => a.drone_id ? `#${a.drone_id}` : '?').join(', ')}`
            : 'Drones: None available';

        // Also update mission stats
        document.getElementById('wp-count').textContent = `${result.total_waypoints} WPs`;
        document.getElementById('mission-dist').textContent = `${(result.total_distance_m / 1000).toFixed(2)} km`;

        // Enable confirm button
        document.getElementById('btn-confirm-upload').disabled = (result.num_drones === 0);

        addConsoleLog(
            `✓ Preview: ${result.total_waypoints} WPs, ${(result.total_distance_m / 1000).toFixed(2)} km, ` +
            `${result.num_drones} drone(s), area=${result.area_m2} m²`,
            'success'
        );

    } catch (err) {
        addConsoleLog(`✗ Tile request failed: ${err.message}`, 'error');
    }
}

async function confirmAndUpload() {
    if (!tilingResult || !tilingResult.assignments) {
        addConsoleLog('Generate a preview first.', 'error');
        return;
    }

    const speed = parseFloat(document.getElementById('mission-speed').value) || 5;

    addConsoleLog(`Uploading tiled missions to ${tilingResult.num_drones} drone(s)...`, 'info');

    for (const assignment of tilingResult.assignments) {
        if (!assignment.drone_id) continue;
        const droneId = assignment.drone_id;

        try {
            // Upload
            const uploadResp = await fetch(`/api/drone/${droneId}/mission/upload`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    waypoints: assignment.waypoints,
                    speed: speed,
                }),
            });
            const uploadResult = await uploadResp.json();
            if (uploadResult.success) {
                addConsoleLog(`✓ Mission uploaded to Drone ${droneId} (${assignment.waypoints.length} WPs)`, 'success');
            } else {
                addConsoleLog(`✗ Drone ${droneId} upload failed: ${uploadResult.message}`, 'error');
                continue;
            }

            // Start
            const startResp = await fetch(`/api/drone/${droneId}/mission/start`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
            });
            const startResult = await startResp.json();
            if (startResult.success) {
                addConsoleLog(`✓ Mission started on Drone ${droneId}`, 'success');
            } else {
                addConsoleLog(`✗ Drone ${droneId} start failed: ${startResult.message}`, 'error');
            }
        } catch (err) {
            addConsoleLog(`✗ Drone ${droneId} request failed: ${err.message}`, 'error');
        }
    }
}

function pauseTilingMission() {
    if (!tilingResult || !tilingResult.assignments) return;
    for (const assignment of tilingResult.assignments) {
        if (assignment.drone_id) {
            sendCommand(assignment.drone_id, 'mission/pause');
        }
    }
}

// ─── Clearing ──────────────────────────────────────────────────────────────

function clearPreviewPolylines() {
    previewPolylines.forEach(p => map.removeLayer(p));
    previewPolylines = [];
}

function clearArea() {
    areaClicks = [];
    tilingResult = null;
    if (drawnShape) { map.removeLayer(drawnShape); drawnShape = null; }
    clearPreviewPolylines();
    document.getElementById('tiling-info').style.display = 'none';
    document.getElementById('btn-confirm-upload').disabled = true;
    document.getElementById('mission-area').textContent = '0 m²';
    document.getElementById('wp-count').textContent = '0 WPs';
    document.getElementById('mission-dist').textContent = '0.0 km';
    // Remove any temporary markers
    map.eachLayer(layer => {
        if (layer instanceof L.CircleMarker && !(layer instanceof L.Circle) && !droneMarkers[layer]) {
            // Only remove orange temp markers, not drone markers
            if (layer.options && layer.options.color === '#FF8C00') {
                map.removeLayer(layer);
            }
        }
    });
    addConsoleLog('Area and preview cleared', 'info');
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

        document.getElementById('fleet-count').textContent =
            `${connectedCount}/${Object.keys(drones).length}`;
        document.getElementById('last-update').textContent =
            new Date().toLocaleTimeString('en-GB', { hour12: false });

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

    const missionsSpan = document.getElementById(`drone-${droneId}-missions`);
    if (missionsSpan && telem.nummissions !== undefined) {
        missionsSpan.textContent = telem.nummissions;
    }

    const connBadge = document.getElementById(`drone-${droneId}-conn`);
    connBadge.textContent = connected ? 'ONLINE' : (telem.reachable ? 'NO PX4' : 'OFFLINE');
    connBadge.classList.toggle('online', connected);

    const armedBadge = document.getElementById(`drone-${droneId}-armed-badge`);
    armedBadge.textContent = telem.armed ? 'ARMED' : 'DISARMED';
    armedBadge.classList.toggle('armed', telem.armed);

    setTelem(droneId, 'lat', connected ? telem.lat?.toFixed(6) + '°' : '--');
    setTelem(droneId, 'lon', connected ? telem.lon?.toFixed(6) + '°' : '--');
    setTelem(droneId, 'alt', connected ? telem.alt?.toFixed(1) + ' m' : '--');
    setTelem(droneId, 'speed', connected ? telem.ground_speed?.toFixed(1) + ' m/s' : '--');
    setTelem(droneId, 'mode', connected ? telem.flight_mode : '--');
    setTelem(droneId, 'heading', connected ? telem.heading_deg?.toFixed(0) + '°' : '--');

    const batPercent = telem.battery_percent || 0;
    setTelem(droneId, 'battery', connected ? `${batPercent.toFixed(0)}%` : '--');

    const batBar = document.getElementById(`drone-${droneId}-battery-bar`);
    if (batBar) {
        batBar.style.width = connected ? `${Math.min(100, batPercent)}%` : '0%';
        batBar.classList.remove('low', 'critical');
        if (batPercent < 15) batBar.classList.add('critical');
        else if (batPercent < 30) batBar.classList.add('low');
    }

    const gpsText = connected
        ? `${telem.gps_fix_type} (${telem.gps_num_satellites})`
        : '--';
    setTelem(droneId, 'gps', gpsText);

    if (connected && telem.lat && telem.lon) {
        updateMapMarker(droneId, telem);
    }
}

function updateMapMarker(droneId, telem) {
    if (!map) return;

    const pos = geoToPixel(telem.lat, telem.lon);
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
    const altitude = document.getElementById('mission-alt').value;
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

// ─── Manual Mission Planning (Waypoint Mode) ──────────────────────────────

function updateMissionStats() {
    document.getElementById('wp-count').textContent = `${missionWaypoints.length} WPs`;

    if (missionWaypoints.length < 2) {
        document.getElementById('mission-dist').textContent = "0.0 km";
        return;
    }

    let dist = 0;
    for (let i = 1; i < missionWaypoints.length; i++) {
        const w1 = missionWaypoints[i - 1];
        const w2 = missionWaypoints[i];
        const dLat = (w2.lat - w1.lat) * 111320;
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
    consoleEl.scrollTop = consoleEl.scrollHeight;

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
