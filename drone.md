# Autonomous Drone Project Notes

## Hardware Components

### 1. **Brain/AI Processing Unit**
   - **Jetson Orin Nano Super**:
     - Acts as the brain of the drone for AI processing.
     - Communicates with the **Pixhawk 6** flight controller using **USB/UART**.

### 2. **Flight Controller**
   - **Pixhawk 6**:
     - Connected to the Jetson Orin Nano Super via USB/UART.
     - Receives control messages from the MAVLink ROS2 node.
     - Includes a radio transmitter and receiver for **manual mode control**.

### 3. **Drone Frame**
   - **Material**: Carbon fiber for lightweight and durability.
   - **Motors**: T-Motor MN series with a thrust of **2.2 kg per motor**.
   - **Battery**: 4S LiPo battery.
   - **Propellers**: 18-inch propellers.

## Software and Communication

### 1. **AI and Object Detection**
   - **Custom ROS2 + DeepStream Node**:
     - Performs AI-based detection and tracking for:
       - Weapons
       - People
       - Vehicles
     - The detected data is published to a dedicated **ROS2 topic**.

### 2. **Drone Movement Logic**
   - A separate logic code:
     - Decides the drone's movement based on the data from the AI node.
     - Publishes control commands to a **control topic**.

### 3. **MAVLink Communication**
   - **MAVLink ROS2 Node**:
     - Subscribes to control messages from the control topic.
     - Sends the control messages to the Pixhawk using the **MAVLink Python library**.

## Manual Control
   - A **radio transmitter and receiver** are connected to the Pixhawk for manual override and control.

---

### Summary of Workflow
1. **AI Node**: Detects objects and publishes data to a topic.
2. **Logic Code**: Processes detection data and publishes control commands.
3. **MAVLink Node**: Transfers control commands to Pixhawk.
4. **Pixhawk**: Executes commands and operates the drone.
5. **Manual Control**: Radio transmitter/receiver for manual control in emergency/manual mode.
