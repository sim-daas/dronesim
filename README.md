# Drone Mission System with Autonomous Recovery

## Overview
This system implements a complete autonomous drone mission workflow with battery simulation and precision landing recovery.

## Workflow

### 1. Mission Execution (dashboard.py → mission.py)
- User clicks "Execute Mission" in dashboard
- Dashboard disconnects to avoid conflicts
- Launches `mission.py` as separate process
- `mission.py` arms drone, uploads waypoints, starts mission
- Random interruption timer: 20-40 seconds (simulates battery low)

### 2. Mission Interruption & Recovery (dashboard.py)
- After random time or mission completion, `mission.py` exits
- Dashboard automatically reconnects (3 retry attempts)
- Sends drone to home position using waypoint command
- Monitors arrival at home position

### 3. Precision Landing (lander.py)
- Once at home position, launches `lander.py`
- `lander.py` uses ArUco markers and CNN for precision landing
- Executes final landing command

## Files

### mission.py
- **Arms drone** (waits for successful arming with retries)
- Uploads mission waypoints
- Starts mission execution
- Runs for random duration (20-40 seconds) then exits
- Handles both natural completion and interruption

### dashboard.py
- Main GUI application
- **Execute Mission**: Combined upload and start
- **Auto-reconnection**: Automatically reconnects after mission
- **Recovery process**: Returns drone to home position
- **Error handling**: 2-3 retries with manual intervention popups

### lander.py
- Precision landing using computer vision
- ArUco marker detection
- CNN analysis for landing zone
- Executes final landing command

## Key Features

1. **Battery Simulation**: Random 20-40 second mission duration
2. **Smooth Transition**: Waypoint commands instead of RTL for smoother operation
3. **Automatic Recovery**: No manual intervention needed for normal operation
4. **Retry Logic**: 2-3 attempts for all critical operations
5. **Manual Fallback**: Popup notifications when automatic recovery fails
6. **Unified Workflow**: Same recovery process for completion and interruption

## Usage

1. Start PX4 SITL: `make px4_sitl gazebo`
2. Run dashboard: `python3 dashboard.py`
3. Connect to drone
4. Add waypoints by clicking on map
5. Click "Execute Mission"
6. System handles everything automatically:
   - Mission execution
   - Battery simulation/interruption
   - Return to home
   - Precision landing

## Error Handling

- Connection failures: 3 retry attempts
- Arming failures: 3 retry attempts  
- Goto/RTL failures: Try both with retries
- Landing failures: Fallback to basic landing
- All failures: Manual intervention popup

## Simulation Features

- **Random mission duration**: Simulates real-world battery constraints
- **Automatic recovery**: Tests autonomous failure handling
- **Precision landing**: Demonstrates advanced landing capabilities
- **Error scenarios**: Tests robustness of recovery systems