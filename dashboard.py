import sys
import asyncio
import threading
import time
import math
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QPushButton, QLabel, 
                             QLineEdit, QTextEdit, QListWidget, QFrame, 
                             QMessageBox, QGraphicsView, QGraphicsScene,
                             QGraphicsEllipseItem, QGraphicsTextItem, 
                             QGraphicsLineItem, QSplitter, QTabWidget)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QPointF
from PyQt5.QtGui import QFont, QPen, QBrush, QColor, QPainter
from mavsdk import System
import subprocess
import json
import os

class DroneGCSDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Custom Drone GCS Dashboard")
        self.setGeometry(100, 100, 1400, 900)
        
        # MAVSDK connection
        self.drone = System()
        self.connected = False
        self.telemetry_thread = None
        self.running = False
        self.event_loop = None
        
        # Mission data
        self.waypoints = []
        self.drone_position = {'lat': 0, 'lon': 0, 'alt': 0}
        self.map_center = {'lat': 47.3977, 'lon': 8.5456}  # Default: PX4 SITL Zurich location
        self.map_scale = 1000  # meters per pixel
        
        # Mission process tracking
        self.mission_process = None
        self.home_coordinates = {'lat': 47.3977, 'lon': 8.5456, 'alt': 10.0}  # Default home position
        self.reconnection_attempts = 0
        self.max_reconnection_attempts = 3
        # Telemetry data
        self.telemetry_data = {
            'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
            'ground_speed': 0.0, 'battery': 0.0,
            'armed': False, 'mode': 'UNKNOWN'
        }
        
        # Color scheme
        self.colors = {
            'background': '#1C1C1C',
            'card': '#2C2C2C',
            'divider': '#444444',
            'primary_text': '#E0E0E0',
            'secondary_text': '#B0B0B0',
            'accent': '#1E90FF',
            'success': '#00FF85',
            'warning': '#FF6F61',
            'info': '#456882',
            'hover': '#3e3e42',
            'disabled': '#555555'
        }
        
        self.setup_ui()
        
    def setup_ui(self):
        # Set main widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Apply dark theme with updated styles
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {self.colors['background']};
                color: {self.colors['primary_text']};
            }}
            QWidget {{
                background-color: {self.colors['background']};
                color: {self.colors['primary_text']};
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 10pt;
            }}
            QFrame {{
                background-color: {self.colors['card']};
                border: 1px solid {self.colors['divider']};
                border-radius: 8px;
                margin: 4px;
                padding: 8px;
            }}
            QPushButton {{
                background-color: {self.colors['accent']};
                color: white;
                border: none;
                border-radius: 4px;
                padding: 4px 8px;
                font-weight: bold;
                min-height: 16px;
                font-size: 9pt;
            }}
            QPushButton:hover {{
                background-color: {self.colors['hover']};
            }}
            QPushButton:pressed {{
                background-color: {self.colors['info']};
            }}
            QPushButton:disabled {{
                background-color: {self.colors['disabled']};
                color: {self.colors['secondary_text']};
            }}
            QLabel {{
                color: {self.colors['primary_text']};
                font-weight: normal;
            }}
            QLineEdit {{
                background-color: {self.colors['card']};
                border: 2px solid {self.colors['divider']};
                border-radius: 4px;
                padding: 6px;
                color: {self.colors['primary_text']};
            }}
            QLineEdit:focus {{
                border-color: {self.colors['accent']};
            }}
            QTextEdit, QListWidget {{
                background-color: {self.colors['card']};
                border: 1px solid {self.colors['divider']};
                border-radius: 4px;
                color: {self.colors['primary_text']};
                padding: 4px;
            }}
            QGraphicsView {{
                background-color: {self.colors['card']};
                border: 1px solid {self.colors['divider']};
                border-radius: 4px;
            }}
            QTabWidget::pane {{
                border: 1px solid {self.colors['divider']};
                background-color: {self.colors['card']};
            }}
            QTabBar::tab {{
                background-color: {self.colors['card']};
                color: {self.colors['primary_text']};
                padding: 8px 16px;
                margin-right: 2px;
                border: 1px solid {self.colors['divider']};
                border-bottom: none;
                border-radius: 4px 4px 0px 0px;
            }}
            QTabBar::tab:selected {{
                background-color: {self.colors['accent']};
                color: white;
            }}
            QTabBar::tab:hover {{
                background-color: {self.colors['hover']};
            }}
        """)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(5)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Top control bar (compact) - all controls on left side
        top_layout = QHBoxLayout()
        self.setup_connection_panel(top_layout)
        self.setup_mission_control_panel(top_layout)
        self.setup_top_telemetry_panel(top_layout)
        self.setup_top_map_controls_panel(top_layout)  # Add map controls to top bar
        top_layout.addStretch()  # Push everything to left, leaving right side empty
        main_layout.addLayout(top_layout)
        
        # Main content area - vertical layout to stack right panel above map controls
        content_layout = QHBoxLayout()
        content_layout.setSpacing(5)
        
        # Left side: Large map
        self.setup_map_panel(content_layout)
        
        # Right side: Only tabbed interface (map controls moved to top bar)
        self.setup_tabbed_panels(content_layout)
        
        main_layout.addLayout(content_layout)
        
    def setup_connection_panel(self, parent):
        conn_frame = QFrame()
        conn_frame.setFixedWidth(300)  # Increased to 1.5x (200 * 1.5 = 300)
        parent.addWidget(conn_frame)
        
        layout = QVBoxLayout(conn_frame)
        layout.setContentsMargins(8, 3, 8, 3)  # Reduced height margins
        
        # Connection buttons (vertical)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['success']}; min-width: 80px; min-height: 20px; }}")
        self.connect_btn.clicked.connect(self.connect_mavlink)
        layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['warning']}; min-width: 80px; min-height: 20px; }}")
        self.disconnect_btn.clicked.connect(self.disconnect_mavlink)
        self.disconnect_btn.setEnabled(False)
        layout.addWidget(self.disconnect_btn)
        
        # Connection status (smaller)
        self.status_label = QLabel("Disconnected")
        self.status_label.setFont(QFont("Arial", 8))
        self.status_label.setStyleSheet(f"color: {self.colors['warning']};")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
    def setup_mission_control_panel(self, parent):
        mission_frame = QFrame()
        mission_frame.setFixedWidth(560)  # Double the width
        parent.addWidget(mission_frame)
        
        layout = QVBoxLayout(mission_frame)
        layout.setContentsMargins(8, 5, 8, 5)
        
        # Title
        title = QLabel("Mission Control")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Compact button layout (horizontal)
        button_layout = QHBoxLayout()
        button_layout.setSpacing(4)
        
        # ARM/DISARM
        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['warning']}; min-width: 45px; }}")
        self.arm_btn.clicked.connect(self.arm_drone)
        button_layout.addWidget(self.arm_btn)
        
        self.disarm_btn = QPushButton("DISARM")
        self.disarm_btn.setStyleSheet(f"QPushButton {{ background-color: #e67e22; min-width: 50px; }}")
        self.disarm_btn.clicked.connect(self.disarm_drone)
        button_layout.addWidget(self.disarm_btn)
        
        # TAKEOFF/LAND
        self.takeoff_btn = QPushButton("TAKEOFF")
        self.takeoff_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['accent']}; min-width: 55px; }}")
        self.takeoff_btn.clicked.connect(self.takeoff_drone)
        button_layout.addWidget(self.takeoff_btn)
        
        self.land_btn = QPushButton("LAND")
        self.land_btn.setStyleSheet(f"QPushButton {{ background-color: #9b59b6; min-width: 40px; }}")
        self.land_btn.clicked.connect(self.land_drone)
        button_layout.addWidget(self.land_btn)
        
        # RTL
        self.rtl_btn = QPushButton("RTL")
        self.rtl_btn.setStyleSheet(f"QPushButton {{ background-color: #c0392b; min-width: 35px; }}")
        self.rtl_btn.clicked.connect(self.return_to_launch)
        button_layout.addWidget(self.rtl_btn)
        
        layout.addLayout(button_layout)
    
    def setup_top_telemetry_panel(self, parent):
        telem_frame = QFrame()
        telem_frame.setFixedWidth(450)  # Increased to 1.5x (300 * 1.5 = 450)
        parent.addWidget(telem_frame)
        
        layout = QVBoxLayout(telem_frame)
        layout.setContentsMargins(8, 3, 8, 3)  # Reduced height margins
        
        # Remove title and go straight to telemetry data display
        data_widget = QWidget()
        data_layout = QGridLayout(data_widget)
        data_layout.setSpacing(4)
        data_layout.setContentsMargins(0, 0, 0, 0)
        
        # Create compact telemetry labels (only essential data, no battery)
        self.telem_labels = {}
        telem_items = [
            ('Lat:', 'lat'), ('Lon:', 'lon'), ('Alt:', 'alt'),
            ('Speed:', 'ground_speed'), ('Armed:', 'armed'), ('Mode:', 'mode')
        ]
        
        for i, (label_text, key) in enumerate(telem_items):
            row = i // 2
            col = (i % 2) * 2
            
            label = QLabel(label_text)
            label.setFont(QFont("Arial", 8))
            label.setStyleSheet(f"color: {self.colors['secondary_text']};")
            data_layout.addWidget(label, row, col, Qt.AlignLeft)
            
            self.telem_labels[key] = QLabel("--")
            self.telem_labels[key].setFont(QFont("Arial", 8, QFont.Bold))
            self.telem_labels[key].setStyleSheet(f"color: {self.colors['success']};")
            data_layout.addWidget(self.telem_labels[key], row, col + 1, Qt.AlignLeft)
        
        layout.addWidget(data_widget)
    
    def setup_top_map_controls_panel(self, parent):
        map_controls_frame = QFrame()
        map_controls_frame.setFixedWidth(420)  # Increased to 1.5x (280 * 1.5 = 420)
        parent.addWidget(map_controls_frame)
        
        layout = QVBoxLayout(map_controls_frame)
        layout.setContentsMargins(8, 2, 8, 2)  # Reduced height margins
        
        # Title (reduced height)
        title = QLabel("Map Controls")
        title.setFont(QFont("Arial", 9, QFont.Bold))  # Slightly smaller font
        title.setAlignment(Qt.AlignCenter)
        title.setMaximumHeight(15)  # Limit title height
        layout.addWidget(title)
        
        # Buttons layout (vertical)
        # Clear waypoints button
        clear_btn = QPushButton("Clear Waypoints")
        clear_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['warning']}; min-width: 100px; min-height: 18px; }}")
        clear_btn.clicked.connect(self.clear_waypoints)
        layout.addWidget(clear_btn)
        
        # Center on drone button
        center_btn = QPushButton("Center on Drone")
        center_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['accent']}; min-width: 100px; min-height: 18px; }}")
        center_btn.clicked.connect(self.center_on_drone)
        layout.addWidget(center_btn)
        
        # Waypoint count display
        self.waypoint_count_label = QLabel("Waypoints: 0")
        self.waypoint_count_label.setFont(QFont("Arial", 8, QFont.Bold))
        self.waypoint_count_label.setStyleSheet(f"color: {self.colors['success']};")
        self.waypoint_count_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.waypoint_count_label)
        
    def setup_map_panel(self, parent):
        # Large map (no title, expanded size) - takes full left side
        map_frame = QFrame()
        parent.addWidget(map_frame)
        
        map_layout = QVBoxLayout(map_frame)
        map_layout.setContentsMargins(5, 5, 5, 5)
        
        # Map graphics view (increased size, no title)
        self.map_view = QGraphicsView()
        self.map_view.setMinimumSize(800, 600)  # Increased from 600x400
        self.map_scene = QGraphicsScene()
        self.map_view.setScene(self.map_scene)
        self.map_view.setRenderHint(QPainter.Antialiasing)
        
        # Set up mouse events for both waypoint placement and panning
        self.map_view.mousePressEvent = self.map_mouse_press
        self.map_view.mouseMoveEvent = self.map_mouse_move
        self.map_view.mouseReleaseEvent = self.map_mouse_release
        
        # Pan state tracking
        self.is_panning = False
        self.last_pan_point = None
        
        # Enable drag mode for panning
        self.map_view.setDragMode(QGraphicsView.NoDrag)
        
        map_layout.addWidget(self.map_view)
        
        self.draw_map()
        
    def setup_tabbed_panels(self, parent):
        # Right side tabbed interface
        tab_widget = QTabWidget()
        tab_widget.setFixedWidth(350)
        parent.addWidget(tab_widget)
        
        # Parameters tab
        parameters_tab = QWidget()
        tab_widget.addTab(parameters_tab, "Parameters")
        self.setup_parameters_panel(parameters_tab)
        
        # Mission tab
        mission_tab = QWidget()
        tab_widget.addTab(mission_tab, "Mission")
        self.setup_mission_tab_panel(mission_tab)
    
    def setup_map_controls_panel(self, parent):
        # Map controls panel (now under tabbed panels)
        controls_frame = QFrame()
        controls_frame.setFixedWidth(350)
        controls_frame.setMaximumHeight(200)
        parent.addWidget(controls_frame)
        
        controls_layout = QVBoxLayout(controls_frame)
        controls_layout.setContentsMargins(10, 10, 10, 10)
        
        # Controls title
        controls_title = QLabel("Map Controls")
        controls_title.setFont(QFont("Arial", 10, QFont.Bold))
        controls_title.setAlignment(Qt.AlignCenter)
        controls_layout.addWidget(controls_title)
        
        # Control buttons
        clear_btn = QPushButton("Clear Waypoints")
        clear_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['warning']}; }}")
        clear_btn.clicked.connect(self.clear_waypoints)
        controls_layout.addWidget(clear_btn)
        
        center_btn = QPushButton("Center on Drone")
        center_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['accent']}; }}")
        center_btn.clicked.connect(self.center_on_drone)
        controls_layout.addWidget(center_btn)
        
        # Waypoint list
        waypoint_label = QLabel("Waypoints:")
        waypoint_label.setFont(QFont("Arial", 9, QFont.Bold))
        controls_layout.addWidget(waypoint_label)
        
        self.waypoint_listbox = QListWidget()
        self.waypoint_listbox.setMaximumHeight(80)
        controls_layout.addWidget(self.waypoint_listbox)
        
        controls_layout.addStretch()
        
    def setup_telemetry_panel(self, parent):
        layout = QVBoxLayout(parent)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Title
        title = QLabel("Real-Time Telemetry")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Telemetry data display
        data_widget = QWidget()
        data_layout = QGridLayout(data_widget)
        data_layout.setSpacing(8)
        
        # Create telemetry labels
        self.telem_labels = {}
        telem_items = [
            ('Latitude:', 'lat'), ('Longitude:', 'lon'), ('Altitude:', 'alt'),
            ('Ground Speed:', 'ground_speed'), ('Battery:', 'battery'),
            ('Armed:', 'armed'), ('Mode:', 'mode'), ('GPS Status:', 'gps_status')
        ]
        
        for i, (label_text, key) in enumerate(telem_items):
            label = QLabel(label_text)
            label.setStyleSheet(f"color: {self.colors['secondary_text']};")
            data_layout.addWidget(label, i, 0, Qt.AlignLeft)
            
            self.telem_labels[key] = QLabel("--")
            self.telem_labels[key].setStyleSheet(f"color: {self.colors['success']}; font-weight: bold;")
            data_layout.addWidget(self.telem_labels[key], i, 1, Qt.AlignLeft)
        
        layout.addWidget(data_widget)
        
        # System status
        status_title = QLabel("System Console")
        status_title.setFont(QFont("Arial", 11, QFont.Bold))
        status_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(status_title)
        
        self.system_status = QTextEdit()
        self.system_status.setFont(QFont("Courier", 9))
        self.system_status.setStyleSheet(f"""
            QTextEdit {{
                background-color: {self.colors['background']};
                color: {self.colors['primary_text']};
                border: 1px solid {self.colors['divider']};
            }}
        """)

   
    def setup_parameters_panel(self, parent):
        layout = QVBoxLayout(parent)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Title
        title = QLabel("Flight Parameters")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Parameter inputs grid
        params_widget = QWidget()
        params_layout = QGridLayout(params_widget)
        params_layout.setSpacing(10)
        
        # Takeoff altitude
        takeoff_label = QLabel("Takeoff Alt (m):")
        takeoff_label.setStyleSheet(f"color: {self.colors['secondary_text']};")
        params_layout.addWidget(takeoff_label, 0, 0)
        
        self.takeoff_alt_entry = QLineEdit()
        self.takeoff_alt_entry.setText("5")  # Default 5m altitude
        self.takeoff_alt_entry.setFixedWidth(80)
        params_layout.addWidget(self.takeoff_alt_entry, 0, 1)
        
        # Mission speed
        speed_label = QLabel("Speed (m/s):")
        speed_label.setStyleSheet(f"color: {self.colors['secondary_text']};")
        params_layout.addWidget(speed_label, 1, 0)
        
        self.speed_entry = QLineEdit()
        self.speed_entry.setText("5.0")
        self.speed_entry.setFixedWidth(80)
        params_layout.addWidget(self.speed_entry, 1, 1)
        
        # Mission altitude
        mission_alt_label = QLabel("Mission Alt (m):")
        mission_alt_label.setStyleSheet(f"color: {self.colors['secondary_text']};")
        params_layout.addWidget(mission_alt_label, 2, 0)
        
        self.mission_alt_entry = QLineEdit()
        self.mission_alt_entry.setText("10.0")
        self.mission_alt_entry.setFixedWidth(80)
        params_layout.addWidget(self.mission_alt_entry, 2, 1)
        
        layout.addWidget(params_widget)
        
        # Live Video Section
        video_title = QLabel("Camera Control")
        video_title.setFont(QFont("Arial", 11, QFont.Bold))
        video_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(video_title)
        
        # Video controls
        video_controls = QHBoxLayout()
        
        self.camera_btn = QPushButton("Open Camera")
        self.camera_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['info']}; min-width: 100px; }}")
        self.camera_btn.clicked.connect(self.open_camera_window)
        video_controls.addWidget(self.camera_btn)
        
        self.record_btn = QPushButton("Record")
        self.record_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['warning']}; min-width: 80px; }}")
        video_controls.addWidget(self.record_btn)
        
        layout.addLayout(video_controls)
        
        # Apply button
        apply_btn = QPushButton("Apply Parameters")
        apply_btn.setStyleSheet(f"QPushButton {{ background-color: #8e44ad; min-width: 150px; }}")
        apply_btn.clicked.connect(self.apply_parameters)
        layout.addWidget(apply_btn)
        
        layout.addStretch()
    
    def setup_mission_tab_panel(self, parent):
        layout = QVBoxLayout(parent)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Title
        title = QLabel("Mission Planning")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Mission controls
        mission_controls = QVBoxLayout()
        
        # Execute mission button (combined upload and start)
        self.execute_mission_btn = QPushButton("Execute Mission")
        self.execute_mission_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['success']}; min-height: 30px; }}")
        self.execute_mission_btn.clicked.connect(self.execute_mission)
        mission_controls.addWidget(self.execute_mission_btn)
        
        # Test hardcoded mission button
        test_mission_btn = QPushButton("Test 5m Square Mission")
        test_mission_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['success']}; min-height: 30px; }}")
        test_mission_btn.clicked.connect(self.test_hardcoded_mission)
        mission_controls.addWidget(test_mission_btn)
        
        # Debug button
        debug_btn = QPushButton("Debug Mission System")
        debug_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['info']}; min-height: 25px; font-size: 8pt; }}")
        debug_btn.clicked.connect(self.debug_mission_button)
        mission_controls.addWidget(debug_btn)
        
        # Mission info
        mission_info = QLabel("Mission Status:")
        mission_info.setFont(QFont("Arial", 10, QFont.Bold))
        mission_controls.addWidget(mission_info)
        
        self.mission_status = QLabel("No mission loaded")
        self.mission_status.setStyleSheet(f"color: {self.colors['secondary_text']};")
        mission_controls.addWidget(self.mission_status)
        
        layout.addLayout(mission_controls)
        
        # Add system console to mission tab
        console_title = QLabel("System Console")
        console_title.setFont(QFont("Arial", 10, QFont.Bold))
        console_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(console_title)
        
        self.system_status = QTextEdit()
        self.system_status.setFont(QFont("Courier", 8))
        self.system_status.setStyleSheet(f"""
            QTextEdit {{
                background-color: {self.colors['background']};
                color: {self.colors['primary_text']};
                border: 1px solid {self.colors['divider']};
            }}
        """)
        self.system_status.setMaximumHeight(150)
        layout.addWidget(self.system_status)
        
        layout.addStretch()

    def connect_mavlink(self):
        try:
            # Default connection for PX4 SITL
            mavsdk_address = "udp://:14540"
            
            # Start connection in thread
            self.telemetry_thread = threading.Thread(
                target=self.run_connection_async, 
                args=(mavsdk_address,), 
                daemon=True
            )
            self.telemetry_thread.start()
            
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect: {str(e)}")
            
    def disconnect_mavlink(self):
        self.connected = False
        self.running = False
        
        # Update UI
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet(f"color: {self.colors['warning']};")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        
        self.log_message("MAVSDK connection closed")
        
    def run_connection_async(self, address):
        """Run the async connection in a separate thread"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.event_loop = loop
        loop.run_until_complete(self.connect_and_monitor(address))
        
    async def connect_and_monitor(self, address):
        """Connect to drone and start monitoring telemetry"""
        try:
            # Connect to drone
            await self.drone.connect(system_address=address)
            self.log_message(f"Connecting to drone at {address}...")
            
            # Wait for connection
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.connected = True
                    self.running = True
                    
                    # Update UI in main thread
                    QTimer.singleShot(0, self.update_connection_ui)
                    self.log_message("MAVSDK connection established")
                    
                    # Start telemetry monitoring
                    await self.monitor_telemetry()
                    break
                    
        except Exception as e:
            self.log_message(f"Connection error: {str(e)}")
            QTimer.singleShot(0, lambda: QMessageBox.critical(
                self, "Connection Error", f"Failed to connect: {str(e)}"
            ))
            
    def update_connection_ui(self):
        """Update UI when connection is established"""
        self.status_label.setText("Connected")
        self.status_label.setStyleSheet(f"color: {self.colors['success']};")
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        
    async def monitor_telemetry(self):
        """Monitor telemetry data from the drone"""
        # Start all telemetry monitoring tasks concurrently
        await asyncio.gather(
            self.monitor_position(),
            self.monitor_flight_mode(),
            self.monitor_battery(),
            self.monitor_gps(),
            self.monitor_armed_state(),
            self.monitor_velocity()
        )
        
    async def monitor_position(self):
        """Monitor drone position"""
        async for position in self.drone.telemetry.position():
            self.telemetry_data['lat'] = position.latitude_deg
            self.telemetry_data['lon'] = position.longitude_deg
            self.telemetry_data['alt'] = position.relative_altitude_m
            
            self.drone_position['lat'] = position.latitude_deg
            self.drone_position['lon'] = position.longitude_deg
            self.drone_position['alt'] = position.relative_altitude_m
            
            # Update UI
            QTimer.singleShot(0, self.update_telemetry_display)
            QTimer.singleShot(0, self.update_map)
            
    async def monitor_flight_mode(self):
        """Monitor flight mode"""
        async for flight_mode in self.drone.telemetry.flight_mode():
            self.telemetry_data['mode'] = flight_mode.name
            QTimer.singleShot(0, self.update_telemetry_display)
            
    async def monitor_battery(self):
        """Monitor battery status"""
        async for battery in self.drone.telemetry.battery():
            self.telemetry_data['battery'] = battery.remaining_percent * 100
            QTimer.singleShot(0, self.update_telemetry_display)
            
    async def monitor_gps(self):
        """Monitor GPS status"""
        async for gps_info in self.drone.telemetry.gps_info():
            fix_types = {
                0: 'No GPS',
                1: 'No Fix',
                2: '2D Fix',
                3: '3D Fix',
                4: 'DGPS',
                5: 'RTK Float',
                6: 'RTK Fixed'
            }
            fix_type = fix_types.get(gps_info.fix_type, f'Unknown({gps_info.fix_type})')
            self.telemetry_data['gps_status'] = f"{fix_type} ({gps_info.num_satellites} sats)"
            QTimer.singleShot(0, self.update_telemetry_display)
            
    async def monitor_armed_state(self):
        """Monitor armed state"""
        async for armed in self.drone.telemetry.armed():
            self.telemetry_data['armed'] = armed
            QTimer.singleShot(0, self.update_telemetry_display)
            
    async def monitor_velocity(self):
        """Monitor ground speed"""
        async for velocity in self.drone.telemetry.velocity_ned():
            # Calculate ground speed from north and east components
            ground_speed = math.sqrt(velocity.north_m_s**2 + velocity.east_m_s**2)
            self.telemetry_data['ground_speed'] = ground_speed
            QTimer.singleShot(0, self.update_telemetry_display)
        
    def update_telemetry_display(self):
        self.telem_labels['lat'].setText(f"{self.telemetry_data['lat']:.6f}°")
        self.telem_labels['lon'].setText(f"{self.telemetry_data['lon']:.6f}°")
        self.telem_labels['alt'].setText(f"{self.telemetry_data['alt']:.1f} m")
        self.telem_labels['ground_speed'].setText(f"{self.telemetry_data['ground_speed']:.1f} m/s")
        self.telem_labels['armed'].setText("YES" if self.telemetry_data['armed'] else "NO")
        self.telem_labels['mode'].setText(self.telemetry_data['mode'])
        
    def draw_map(self):
        if not hasattr(self, 'map_scene'):
            return
            
        self.map_scene.clear()
        
        # Draw grid
        width = 800  # Updated to match new map size
        height = 600  # Updated to match new map size  
        
        # Set scene rect
        self.map_scene.setSceneRect(0, 0, width, height)
        
        # Add background image if available
        try:
            from PyQt5.QtGui import QPixmap
            from PyQt5.QtWidgets import QGraphicsPixmapItem
            
            # Try to load background image from root folder
            current_dir = os.path.dirname(os.path.abspath(__file__))
            bg_path = os.path.join(current_dir, "back.png")
            
            if os.path.exists(bg_path):
                pixmap = QPixmap(bg_path)
                # Scale the image to 2.5x the map area for panning capability
                zoom_factor = 2.5
                scaled_width = int(width * zoom_factor)
                scaled_height = int(height * zoom_factor)
                scaled_pixmap = pixmap.scaled(scaled_width, scaled_height, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
                bg_item = QGraphicsPixmapItem(scaled_pixmap)
                # Center the zoomed image so panning can reveal different areas
                bg_item.setPos(-scaled_width//4, -scaled_height//4)  # Offset to center the larger image
                bg_item.setZValue(-1)  # Put background behind everything else
                self.map_scene.addItem(bg_item)
                
                # Expand scene rect to accommodate the larger background for panning
                self.map_scene.setSceneRect(-scaled_width//4, -scaled_height//4, scaled_width, scaled_height)
        except Exception as e:
            # If background loading fails, continue without it
            pass
        
        # Grid and scale labels removed for cleaner map view
            
        # Draw waypoints
        for i, wp in enumerate(self.waypoints):
            x, y = self.lat_lon_to_pixels(wp['lat'], wp['lon'])
            
            # Waypoint circle
            waypoint_item = QGraphicsEllipseItem(x-5, y-5, 10, 10)
            waypoint_item.setBrush(QBrush(QColor(self.colors['warning'])))
            waypoint_item.setPen(QPen(QColor("#c0392b"), 2))
            self.map_scene.addItem(waypoint_item)
            
            # Waypoint label
            label_item = QGraphicsTextItem(f"WP{i+1}")
            label_item.setPos(x+15, y-20)
            label_item.setDefaultTextColor(QColor(self.colors['primary_text']))
            label_item.setFont(QFont("Arial", 8))
            self.map_scene.addItem(label_item)
            
        # Draw drone position
        if self.drone_position['lat'] != 0 or self.drone_position['lon'] != 0:
            x, y = self.lat_lon_to_pixels(self.drone_position['lat'], self.drone_position['lon'])
            
            # Drone circle
            drone_item = QGraphicsEllipseItem(x-8, y-8, 16, 16)
            drone_item.setBrush(QBrush(QColor(self.colors['accent'])))
            drone_item.setPen(QPen(QColor("#2980b9"), 3))
            self.map_scene.addItem(drone_item)
            
            # Drone label
            drone_label = QGraphicsTextItem("D")
            drone_label.setPos(x-4, y-8)
            drone_label.setDefaultTextColor(QColor("white"))
            drone_label.setFont(QFont("Arial", 10, QFont.Bold))
            self.map_scene.addItem(drone_label)
            
    def lat_lon_to_pixels(self, lat, lon):
        # Convert GPS coordinates to pixels with 20m grid squares
        # Each 50-pixel grid square represents 20m x 20m in real world
        width = 800  # Updated to match new map size
        height = 600  # Updated to match new map size
        
        # Simplified conversion: 1 degree ≈ 111,320 meters
        # For small distances, we can use approximate conversion
        # At latitude ~47°: 1 degree lat ≈ 111,320m, 1 degree lon ≈ 75,000m
        
        # Convert GPS differences to approximate meters
        lat_diff_m = (lat - self.map_center['lat']) * 111320  # meters per degree latitude
        lon_diff_m = (lon - self.map_center['lon']) * 111320 * math.cos(math.radians(47.4))  # ~75,000m per degree
        
        # Scale: 2.5 pixels per meter (50 pixels = 20 meters)
        pixels_per_meter = 2.5
        x = width/2 + lon_diff_m * pixels_per_meter
        y = height/2 - lat_diff_m * pixels_per_meter  # Negative because screen Y increases downward
        
        return int(x), int(y)
        
    def pixels_to_lat_lon(self, x, y):
        # Convert pixels back to GPS coordinates  
        # For simulation, use much smaller GPS coordinate changes
        width = 800  # Updated to match new map size
        height = 600  # Updated to match new map size
        
        # Calculate pixel differences from center
        pixel_x_diff = x - width/2
        pixel_y_diff = height/2 - y  # Negative because screen Y increases downward
        
        # Convert pixels to meters (2.5 pixels per meter)
        pixels_per_meter = 2.5
        meter_x_diff = pixel_x_diff / pixels_per_meter
        meter_y_diff = pixel_y_diff / pixels_per_meter
        
        # Convert meters to GPS coordinates - use much smaller scale for simulation
        # 1 degree ≈ 111320m, so 1m ≈ 0.000009 degrees latitude
        # At lat 47.4°, 1m ≈ 0.000013 degrees longitude
        degrees_per_meter_lat = 1.0 / 111320  # ~0.000009
        degrees_per_meter_lon = 1.0 / (111320 * math.cos(math.radians(47.4)))  # ~0.000013
        
        lat = self.map_center['lat'] + meter_y_diff * degrees_per_meter_lat
        lon = self.map_center['lon'] + meter_x_diff * degrees_per_meter_lon
        
        return lat, lon
    
    def map_mouse_press(self, event):
        if event.button() == Qt.LeftButton:
            # Convert view coordinates to scene coordinates
            scene_pos = self.map_view.mapToScene(event.pos())
            self.add_waypoint_at_pos(scene_pos.x(), scene_pos.y())
        elif event.button() == Qt.RightButton:
            # Start panning
            self.is_panning = True
            self.last_pan_point = event.pos()
            self.map_view.setCursor(Qt.ClosedHandCursor)
    
    def map_mouse_move(self, event):
        if self.is_panning and self.last_pan_point is not None:
            # Calculate pan delta
            delta = event.pos() - self.last_pan_point
            self.last_pan_point = event.pos()
            
            # Get current scrollbar values
            h_scroll = self.map_view.horizontalScrollBar()
            v_scroll = self.map_view.verticalScrollBar()
            
            # Update scrollbar positions (negative delta for natural panning)
            h_scroll.setValue(h_scroll.value() - delta.x())
            v_scroll.setValue(v_scroll.value() - delta.y())
    
    def map_mouse_release(self, event):
        if event.button() == Qt.RightButton:
            # Stop panning
            self.is_panning = False
            self.last_pan_point = None
            self.map_view.setCursor(Qt.ArrowCursor)
        
    def add_waypoint_at_pos(self, x, y):
        lat, lon = self.pixels_to_lat_lon(x, y)
        alt = float(self.mission_alt_entry.text())
        
        waypoint = {'lat': lat, 'lon': lon, 'alt': alt}
        self.waypoints.append(waypoint)
        
        # Update waypoint list (if it exists)
        if hasattr(self, 'waypoint_listbox'):
            self.waypoint_listbox.addItem(f"WP{len(self.waypoints)}: {lat:.6f}, {lon:.6f}, {alt}m")
        
        # Update waypoint count in top bar
        if hasattr(self, 'waypoint_count_label'):
            self.waypoint_count_label.setText(f"Waypoints: {len(self.waypoints)}")
        
        self.draw_map()
        self.log_message(f"Added waypoint: {lat:.6f}, {lon:.6f}")
        
    def clear_waypoints(self):
        self.waypoints.clear()
        if hasattr(self, 'waypoint_listbox'):
            self.waypoint_listbox.clear()
        
        # Update waypoint count in top bar
        if hasattr(self, 'waypoint_count_label'):
            self.waypoint_count_label.setText("Waypoints: 0")
            
        self.draw_map()
        self.log_message("Cleared all waypoints")
        
    def center_on_drone(self):
        if self.drone_position['lat'] != 0 or self.drone_position['lon'] != 0:
            self.map_center['lat'] = self.drone_position['lat']
            self.map_center['lon'] = self.drone_position['lon']
            self.draw_map()
            
    def update_map(self):
        self.draw_map()
        
    def arm_drone(self):
        if self.connected:
            asyncio.run_coroutine_threadsafe(self._arm_drone_async(), self.event_loop)
        else:
            QMessageBox.warning(self, "Warning", "Not connected to drone!")
            
    async def _arm_drone_async(self):
        try:
            await self.drone.action.arm()
            self.log_message("ARM command sent via MAVSDK")
        except Exception as e:
            self.log_message(f"ARM failed: {str(e)}")
            
    def disarm_drone(self):
        if self.connected:
            asyncio.run_coroutine_threadsafe(self._disarm_drone_async(), self.event_loop)
        else:
            QMessageBox.warning(self, "Warning", "Not connected to drone!")
            
    async def _disarm_drone_async(self):
        try:
            await self.drone.action.disarm()
            self.log_message("DISARM command sent via MAVSDK")
        except Exception as e:
            self.log_message(f"DISARM failed: {str(e)}")
            
    def takeoff_drone(self):
        if not self.connected:
            QMessageBox.warning(self, "Warning", "Not connected to drone!")
            return
            
        # Check if drone is armed first
        if not self.telemetry_data.get('armed', False):
            QMessageBox.warning(self, "Warning", "Drone must be armed before takeoff!")
            return
        
        alt = float(self.takeoff_alt_entry.text())
        asyncio.run_coroutine_threadsafe(self._takeoff_drone_async(alt), self.event_loop)
            
    async def _takeoff_drone_async(self, altitude):
        try:
            self.log_message(f"Initiating takeoff to {altitude}m altitude")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            self.log_message(f"MAVSDK takeoff command sent to {altitude}m")
        except Exception as e:
            self.log_message(f"Takeoff failed: {str(e)}")
            
    def land_drone(self):
        if self.connected:
            asyncio.run_coroutine_threadsafe(self._land_drone_async(), self.event_loop)
        else:
            QMessageBox.warning(self, "Warning", "Not connected to drone!")
            
    async def _land_drone_async(self):
        try:
            await self.drone.action.land()
            self.log_message("MAVSDK land command sent")
        except Exception as e:
            self.log_message(f"Land failed: {str(e)}")
            
    def execute_mission(self):
        """Execute mission - combined upload and start with automatic interruption handling"""
        if not self.waypoints:
            QMessageBox.warning(self, "Warning", "Add waypoints before executing mission!")
            return
            
        # Disconnect dashboard from drone if connected (to avoid conflicts)
        if self.connected:
            self.log_message("Disconnecting dashboard to avoid conflicts with mission executor...")
            self.disconnect_mavlink()
            
        # Store home coordinates from current drone position or use default
        if self.drone_position['lat'] != 0 or self.drone_position['lon'] != 0:
            self.home_coordinates = {
                'lat': self.drone_position['lat'],
                'lon': self.drone_position['lon'],
                'alt': 10.0  # Safe altitude above home
            }
        
        # Execute mission using separate process
        self.execute_mission_process()
    
    def execute_mission_process(self):
        """Execute mission using separate Python process"""
        try:
            # Prepare waypoints data as JSON
            waypoints_json = json.dumps(self.waypoints)
            speed = self.speed_entry.text()
            
            # Get the path to mission.py (same directory as dashboard.py)
            current_dir = os.path.dirname(os.path.abspath(__file__))
            mission_script = os.path.join(current_dir, "mission.py")
            
            if not os.path.exists(mission_script):
                QMessageBox.critical(self, "Error", f"Mission script not found: {mission_script}")
                return
                
            # Launch mission.py as separate process
            self.log_message("Launching mission executor process...")
            self.log_message(f"Waypoints: {len(self.waypoints)} points")
            self.log_message(f"Speed: {speed} m/s")
            
            # Update mission status
            self.mission_status.setText("Launching mission executor...")
            
            # Start the mission process
            cmd = ["python3", mission_script, waypoints_json, speed]
            self.mission_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Start monitoring the process
            self.monitor_mission_process()
            
        except Exception as e:
            error_msg = f"Failed to execute mission process: {str(e)}"
            self.log_message(error_msg)
            QMessageBox.critical(self, "Mission Error", error_msg)
            self.mission_status.setText("Mission launch failed!")
    
    def monitor_mission_process(self):
        """Monitor the mission process output"""
        try:
            if self.mission_process is None:
                return
                
            # Check if process is still running
            if self.mission_process.poll() is None:
                # Process is still running, schedule next check
                QTimer.singleShot(1000, self.monitor_mission_process)
                self.mission_status.setText("Mission executor running...")
            else:
                # Process has finished
                return_code = self.mission_process.returncode
                stdout, stderr = self.mission_process.communicate()
                
                if return_code == 0:
                    self.log_message("✓ Mission process completed (interrupted or finished)")
                    self.mission_status.setText("Mission completed - starting recovery...")
                else:
                    self.log_message(f"✗ Mission process failed with code {return_code}")
                    self.mission_status.setText("Mission failed - starting recovery...")
                    
                # Log any output and check for battery low signal
                if stdout.strip():
                    for line in stdout.strip().split('\n'):
                        self.log_message(f"Mission: {line}")
                        if "BATTERY_LOW_SIGNAL" in line:
                            self.log_message("🔋 Battery low - drone returning to home for precision landing")
                        
                if stderr.strip():
                    for line in stderr.strip().split('\n'):
                        self.log_message(f"Mission Error: {line}")
                        
                self.mission_process = None
                
                # Start recovery process: reconnect and return to home
                self.start_recovery_process()
                
        except Exception as e:
            self.log_message(f"Error monitoring mission process: {str(e)}")
            self.mission_status.setText("Mission monitoring failed!")
            # Still try recovery
            self.start_recovery_process()
    
    def start_recovery_process(self):
        """Start the recovery process after mission completion/interruption"""
        self.log_message("🏠 Starting recovery process...")
        self.mission_status.setText("Reconnecting for recovery...")
        self.reconnection_attempts = 0
        
        # Start reconnection timer
        QTimer.singleShot(2000, self.attempt_reconnection)  # Wait 2 seconds before reconnecting
    
    def attempt_reconnection(self):
        """Attempt to reconnect to drone for recovery"""
        if self.reconnection_attempts >= self.max_reconnection_attempts:
            self.log_message("✗ Failed to reconnect after maximum attempts")
            self.mission_status.setText("Recovery failed - manual intervention required")
            QMessageBox.critical(
                self, 
                "Recovery Failed", 
                "Failed to reconnect to drone after mission completion.\n"
                "Please manually connect and check drone status."
            )
            return
            
        self.reconnection_attempts += 1
        self.log_message(f"Attempting reconnection {self.reconnection_attempts}/{self.max_reconnection_attempts}...")
        
        try:
            # Attempt reconnection
            self.connect_mavlink()
            
            # Schedule check for successful connection
            QTimer.singleShot(5000, self.check_reconnection_success)
            
        except Exception as e:
            self.log_message(f"Reconnection attempt {self.reconnection_attempts} failed: {e}")
            # Retry after delay
            QTimer.singleShot(3000, self.attempt_reconnection)
    
    def check_reconnection_success(self):
        """Check if reconnection was successful and proceed with recovery"""
        if self.connected:
            self.log_message("✓ Reconnection successful, starting return to home...")
            self.mission_status.setText("Connected - returning to home...")
            self.return_to_home_and_land()
        else:
            self.log_message(f"⚠ Reconnection attempt {self.reconnection_attempts} failed")
            # Retry
            QTimer.singleShot(1000, self.attempt_reconnection)
    
    def return_to_home_and_land(self):
        """Send drone to home position and start landing sequence"""
        if not self.connected or not self.event_loop:
            self.log_message("✗ Cannot return to home - not connected")
            self.mission_status.setText("Recovery failed - not connected")
            return
            
        self.log_message(f"🏠 Commanding drone to return to home position...")
        self.log_message(f"Home coordinates: {self.home_coordinates['lat']:.6f}, {self.home_coordinates['lon']:.6f}, {self.home_coordinates['alt']}m")
        
        # Send goto command to home position
        asyncio.run_coroutine_threadsafe(self._goto_home_async(), self.event_loop)
    
    async def _goto_home_async(self):
        """Send drone to home position using goto command"""
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                self.log_message(f"Goto home attempt {attempt + 1}/{max_retries}")
                
                # Use goto_location method with individual parameters
                await self.drone.action.goto_location(
                    self.home_coordinates['lat'],
                    self.home_coordinates['lon'], 
                    self.home_coordinates['alt'],
                    0  # yaw angle
                )
                
                self.log_message("✓ Goto home command sent")
                
                # Wait a moment for drone to start moving
                await asyncio.sleep(3)
                
                # Monitor arrival at home position
                await self.monitor_home_arrival()
                return
                
            except Exception as e:
                self.log_message(f"✗ Goto home attempt {attempt + 1} failed: {e}")
                
                if attempt < max_retries - 1:
                    await asyncio.sleep(2)
                    continue
                
                # All goto attempts failed, try RTL as fallback
                self.log_message("Attempting RTL as fallback...")
                for rtl_attempt in range(max_retries):
                    try:
                        await self.drone.action.return_to_launch()
                        self.log_message("✓ RTL command sent as fallback")
                        await asyncio.sleep(5)  # Wait for RTL
                        QTimer.singleShot(0, self.launch_precision_landing)
                        return
                    except Exception as rtl_error:
                        self.log_message(f"✗ RTL attempt {rtl_attempt + 1} failed: {rtl_error}")
                        if rtl_attempt < max_retries - 1:
                            await asyncio.sleep(2)
                
                # Both goto and RTL failed
                QTimer.singleShot(0, lambda: self.show_manual_intervention_popup("Both Goto and RTL failed after retries"))
    
    async def monitor_home_arrival(self):
        """Monitor drone arrival at home position"""
        try:
            self.log_message("Monitoring arrival at home position...")
            arrival_timeout = 60  # 60 seconds timeout
            start_time = time.time()
            
            while time.time() - start_time < arrival_timeout:
                # Get current position
                async for position in self.drone.telemetry.position():
                    current_lat = position.latitude_deg
                    current_lon = position.longitude_deg
                    current_alt = position.relative_altitude_m
                    
                    # Calculate distance to home
                    lat_diff = abs(current_lat - self.home_coordinates['lat'])
                    lon_diff = abs(current_lon - self.home_coordinates['lon'])
                    alt_diff = abs(current_alt - self.home_coordinates['alt'])
                    
                    # Check if close to home (within 1 meter horizontally, ignore altitude)
                    # Convert lat/lon differences to approximate meters
                    lat_meters = lat_diff * 111320
                    lon_meters = lon_diff * 111320 * math.cos(math.radians(47.4))
                    horizontal_distance = math.sqrt(lat_meters**2 + lon_meters**2)
                    
                    if horizontal_distance < 1.0:  # Within 1 meter of home position
                        self.log_message(f"✓ Drone arrived within 1m of home position (distance: {horizontal_distance:.1f}m)")
                        QTimer.singleShot(0, self.launch_precision_landing)
                        return
                    
                    break
                
                await asyncio.sleep(2)  # Check every 2 seconds
            
            # Timeout reached
            self.log_message("⚠ Timeout waiting for home arrival, proceeding with landing anyway")
            QTimer.singleShot(0, self.launch_precision_landing)
            
        except Exception as e:
            self.log_message(f"Error monitoring home arrival: {e}")
            QTimer.singleShot(0, self.launch_precision_landing)
    
    def launch_precision_landing(self):
        """Launch the precision landing script"""
        try:
            self.log_message("🎯 Starting precision landing sequence...")
            self.mission_status.setText("Starting precision landing...")
            
            # Get path to lander.py
            current_dir = os.path.dirname(os.path.abspath(__file__))
            lander_script = os.path.join(current_dir, "lander.py")
            
            if not os.path.exists(lander_script):
                self.log_message("⚠ lander.py not found, attempting basic landing...")
                # Fallback to basic landing
                if self.connected and self.event_loop:
                    asyncio.run_coroutine_threadsafe(self.drone.action.land(), self.event_loop)
                    self.mission_status.setText("Basic landing initiated")
                return
            
            # Launch lander.py
            cmd = ["python3", lander_script]
            lander_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.log_message("✓ Precision landing script launched")
            self.mission_status.setText("Precision landing in progress...")
            
            # Monitor lander process (optional)
            QTimer.singleShot(1000, lambda: self.monitor_lander_process(lander_process))
            
        except Exception as e:
            self.log_message(f"✗ Failed to launch precision landing: {e}")
            self.show_manual_intervention_popup("Precision landing launch failed")
    
    def monitor_lander_process(self, lander_process):
        """Monitor the lander process"""
        try:
            if lander_process.poll() is None:
                # Still running
                QTimer.singleShot(2000, lambda: self.monitor_lander_process(lander_process))
            else:
                # Process finished
                return_code = lander_process.returncode
                stdout, stderr = lander_process.communicate()
                
                if return_code == 0:
                    self.log_message("✓ Precision landing completed successfully")
                    self.mission_status.setText("Mission and landing completed!")
                else:
                    self.log_message(f"⚠ Precision landing process returned code {return_code}")
                    self.mission_status.setText("Landing completed with warnings")
                
                # Log lander output
                if stdout.strip():
                    for line in stdout.strip().split('\n'):
                        self.log_message(f"Lander: {line}")
                        
        except Exception as e:
            self.log_message(f"Error monitoring lander process: {e}")
    
    def show_manual_intervention_popup(self, error_message):
        """Show popup requiring manual intervention"""
        QMessageBox.critical(
            self,
            "Manual Intervention Required",
            f"Automatic recovery failed: {error_message}\n\n"
            "Please manually:\n"
            "1. Check drone status\n"
            "2. Land the drone safely\n"
            "3. Reconnect to dashboard if needed"
        )
        self.mission_status.setText("Manual intervention required")

            
    def return_to_launch(self):
        if self.connected:
            asyncio.run_coroutine_threadsafe(self._return_to_launch_async(), self.event_loop)
        else:
            QMessageBox.warning(self, "Warning", "Not connected to drone!")
            
    async def _return_to_launch_async(self):
        try:
            await self.drone.action.return_to_launch()
            self.log_message("RTL command sent via MAVSDK")
        except Exception as e:
            self.log_message(f"RTL failed: {str(e)}")
            
    def open_camera_window(self):
        """Open camera feed in a separate window"""
        self.camera_window = CameraWindow(self)
        self.camera_window.show()
        self.log_message("Camera window opened")
        
    def apply_parameters(self):
        self.log_message("Parameters applied")
        
    def test_hardcoded_mission(self):
        """Test with a hardcoded 5m square mission"""
        if not self.connected:
            QMessageBox.warning(self, "Warning", "Not connected to drone!")
            return
        if not self.event_loop:
            QMessageBox.critical(self, "Error", "Event loop not running. Connect to the drone first.")
            return
            
        self.log_message("Testing hardcoded 5m square mission...")
        if hasattr(self, 'mission_status'):
            self.mission_status.setText("Uploading test mission...")
        asyncio.run_coroutine_threadsafe(self._test_hardcoded_mission_async(), self.event_loop)
        
    async def _test_hardcoded_mission_async(self):
        """Upload a hardcoded 5m square mission"""
        try:
            from mavsdk.mission import MissionItem, MissionPlan
            
            self.log_message("=== Starting Hardcoded Mission Test ===")
            
            # Use PX4 SITL default location  
            home_lat = 47.3977
            home_lon = 8.5456
            home_alt = 10.0
            
            self.log_message(f"Using default position: {home_lat:.6f}, {home_lon:.6f}")
            
            # Clear existing mission
            await self.drone.mission.clear_mission()
            self.log_message("✓ Cleared existing mission")
            
            # Create 5m square waypoints (approximately 0.000045 degrees = ~5m)
            offset = 0.000045
            
            waypoints = [
                {'lat': home_lat, 'lon': home_lon, 'alt': home_alt, 'name': 'Start'},
                {'lat': home_lat + offset, 'lon': home_lon, 'alt': home_alt, 'name': 'North'},
                {'lat': home_lat + offset, 'lon': home_lon + offset, 'alt': home_alt, 'name': 'NE'}, 
                {'lat': home_lat, 'lon': home_lon + offset, 'alt': home_alt, 'name': 'East'},
                {'lat': home_lat, 'lon': home_lon, 'alt': home_alt, 'name': 'Home'}
            ]
            
            # Create mission items
            mission_items = []
            
            for i, wp in enumerate(waypoints):
                mission_item = MissionItem(
                    wp['lat'],                          # latitude_deg
                    wp['lon'],                          # longitude_deg  
                    wp['alt'],                          # relative_altitude_m
                    3.0,                                # speed_m_s (slow for testing)
                    True,                               # is_fly_through
                    float('nan'),                       # gimbal_pitch_deg
                    float('nan'),                       # gimbal_yaw_deg
                    MissionItem.CameraAction.NONE,     # camera_action
                    float('nan'),                       # loiter_time_s
                    float('nan'),                       # camera_photo_interval_s
                    1.0,                                # acceptance_radius_m (tight for testing)
                    float('nan'),                       # yaw_deg
                    float('nan'),                       # camera_photo_distance_m
                    MissionItem.VehicleAction.NONE      # vehicle_action (required parameter)
                )
                mission_items.append(mission_item)
                self.log_message(f"✓ Created waypoint {i+1} ({wp['name']}): {wp['lat']:.6f}, {wp['lon']:.6f}, {wp['alt']}m")
            
            # Create and upload mission plan
            mission_plan = MissionPlan(mission_items)
            self.log_message(f"✓ Created mission plan with {len(mission_items)} waypoints")
            
            # Upload to drone
            self.log_message("Uploading hardcoded mission to drone...")
            await self.drone.mission.upload_mission(mission_plan)
            self.log_message("✓ Hardcoded mission uploaded successfully!")
            
            # Verify upload
            try:
                downloaded_mission = await self.drone.mission.download_mission()
                self.log_message(f"✓ Mission verification: {len(downloaded_mission.mission_items)} items on drone")
            except Exception as e:
                self.log_message(f"⚠️ Mission verification failed: {str(e)}")
            
            # Update UI
            if hasattr(self, 'mission_status'):
                QTimer.singleShot(0, lambda: self.mission_status.setText("Test mission uploaded! (5m square)"))
            self.log_message("=== Hardcoded Mission Test Complete ===")
            
        except Exception as e:
            error_msg = f"Hardcoded mission test failed: {str(e)}"
            self.log_message(error_msg)
            if hasattr(self, 'mission_status'):
                QTimer.singleShot(0, lambda: self.mission_status.setText("Test mission failed!"))
                QTimer.singleShot(0, lambda: QMessageBox.critical(
                    self, "Test Mission Error", error_msg
                ))

    def debug_mission_button(self):
        """Debug mission system - callable from UI"""
        if self.connected and self.event_loop:
            asyncio.run_coroutine_threadsafe(self.debug_mission_system(), self.event_loop)
        else:
            self.log_message("Not connected - cannot debug mission system")
            
    async def debug_mission_system(self):
        """Debug mission system and connection status"""
        try:
            self.log_message("=== Mission System Debug ===")
            
            # Check connection
            connection_state = await self.drone.core.connection_state().__anext__()
            self.log_message(f"Connection state: {connection_state.is_connected}")
            
            # Check if mission system is available
            try:
                current_mission = await self.drone.mission.download_mission()
                self.log_message(f"Current mission on drone: {len(current_mission.mission_items)} items")
            except Exception as e:
                self.log_message(f"Mission system error: {str(e)}")
            
            self.log_message("=== Debug Complete ===")
            
        except Exception as e:
            self.log_message(f"Debug failed: {str(e)}")

    def log_message(self, message):
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        # Use QTimer to ensure thread-safe UI updates
        def update_log():
            if hasattr(self, 'system_status'):
                self.system_status.append(log_entry)
                
                # Keep only last 100 lines
                text = self.system_status.toPlainText()
                lines = text.split('\n')
                if len(lines) > 100:
                    self.system_status.setPlainText('\n'.join(lines[-100:]))
        
        # If we're in the main thread, update directly, otherwise use QTimer
        if hasattr(self, 'system_status'):
            QTimer.singleShot(0, update_log)
        else:
            print(f"LOG: {log_entry}")
            
    def closeEvent(self, event):
        self.running = False
        if self.connected:
            self.disconnect_mavlink()
        event.accept()


class CameraWindow(QMainWindow):
    def __init__(self, parent_dashboard):
        super().__init__()
        self.parent_dashboard = parent_dashboard
        self.setWindowTitle("Live Camera Feed")
        self.setGeometry(200, 200, 800, 600)
        
        # Apply same color scheme as main dashboard
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {parent_dashboard.colors['background']};
                color: {parent_dashboard.colors['primary_text']};
            }}
            QWidget {{
                background-color: {parent_dashboard.colors['background']};
                color: {parent_dashboard.colors['primary_text']};
                font-family: 'Segoe UI', Arial, sans-serif;
            }}
            QPushButton {{
                background-color: {parent_dashboard.colors['accent']};
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px 16px;
                font-weight: bold;
                min-height: 20px;
            }}
            QPushButton:hover {{
                background-color: {parent_dashboard.colors['hover']};
            }}
            QLabel {{
                color: {parent_dashboard.colors['primary_text']};
            }}
        """)
        
        self.setup_camera_ui()
        
    def setup_camera_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Title
        title = QLabel("FPV Camera Feed")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Video display area (large)
        self.video_view = QGraphicsView()
        self.video_view.setMinimumSize(760, 480)
        self.video_scene = QGraphicsScene()
        self.video_view.setScene(self.video_scene)
        self.video_view.setStyleSheet(f"""
            QGraphicsView {{
                background-color: black;
                border: 2px solid {self.parent_dashboard.colors['divider']};
                border-radius: 8px;
            }}
        """)
        
        # Video placeholder text
        placeholder_text = QGraphicsTextItem("Live FPV Video Feed\n\nCamera not connected\nConnect camera source to view feed")
        placeholder_text.setDefaultTextColor(QColor("white"))
        placeholder_text.setFont(QFont("Arial", 16))
        placeholder_text.setPos(200, 200)
        self.video_scene.addItem(placeholder_text)
        
        layout.addWidget(self.video_view)
        
        # Controls
        controls_layout = QHBoxLayout()
        
        record_btn = QPushButton("Start Recording")
        record_btn.setStyleSheet(f"QPushButton {{ background-color: {self.parent_dashboard.colors['warning']}; }}")
        controls_layout.addWidget(record_btn)
        
        snapshot_btn = QPushButton("Take Snapshot")
        snapshot_btn.setStyleSheet(f"QPushButton {{ background-color: {self.parent_dashboard.colors['info']}; }}")
        controls_layout.addWidget(snapshot_btn)
        
        controls_layout.addStretch()
        
        close_btn = QPushButton("Close")
        close_btn.setStyleSheet(f"QPushButton {{ background-color: {self.parent_dashboard.colors['warning']}; }}")
        close_btn.clicked.connect(self.close)
        controls_layout.addWidget(close_btn)
        
        layout.addLayout(controls_layout)


def main():
    app = QApplication(sys.argv)
    window = DroneGCSDashboard()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()