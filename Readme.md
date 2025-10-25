# Drone Mission System with Autonomous Recovery

## Overview
This system implements a complete autonomous drone mission workflow with battery simulation and precision landing recovery.

[![Demo Video](https://img.youtube.com/vi/<video_id>/0.jpg)](https://www.youtube.com/watch?v=fPyEu_ca69Q)

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

# Module Description

This module provides a comprehensive simulation environment for autonomous drone missions, focusing on robust mission execution, battery-aware interruption, and precision landing using computer vision. The system is designed for both research and demonstration purposes, enabling users to test advanced drone autonomy workflows in a safe, repeatable, and fully automated manner.

## Simulation Use Case Features

- **End-to-End Mission Automation**: From mission upload to precision landing, the workflow is fully automated, requiring minimal user intervention.
- **Battery Simulation**: The mission duration is randomized (20-40 seconds) to emulate real-world battery constraints and trigger autonomous recovery.
- **Autonomous Recovery**: Upon mission interruption or completion, the drone returns to the home position and initiates a precision landing sequence.
- **Precision Landing**: Utilizes ArUco marker detection and a CNN-based landing zone analysis for accurate touchdown.
- **Robust Error Handling**: All critical operations (connection, arming, navigation, landing) include retry logic and fallback mechanisms, with user notifications for manual intervention if needed.
- **Unified Workflow**: The same recovery and landing process is used for both normal mission completion and simulated failures, ensuring consistent behavior.

## Docker Run Implementation

This module is designed to run seamlessly inside a Docker container, ensuring consistent dependencies and environment setup. To launch the simulation:

1. **Build the Docker image** (if not already built):
   ```sh
   docker build -t drone-mission-sim .
   ```
2. **Run the container**:
   ```sh
   docker run --rm -it \
     --name drone-mission-sim \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v $(pwd)/src/dronesim:/root/dronews/src/dronesim \
     drone-mission-sim
   ```
   - The container will provide all necessary tools, including PX4 SITL, Gazebo, and required Python dependencies.
   - The GUI dashboard can be accessed via X11 forwarding or a browser, depending on your setup.

---

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
