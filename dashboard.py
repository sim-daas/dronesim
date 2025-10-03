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
        
        # Top control bar (compact)
        top_layout = QHBoxLayout()
        self.setup_connection_panel(top_layout)
        self.setup_mission_control_panel(top_layout)
        top_layout.addStretch()  # Push everything to the left
        main_layout.addLayout(top_layout)
        
        # Main content area
        content_layout = QHBoxLayout()
        content_layout.setSpacing(5)
        
        # Left side: Large map
        self.setup_map_panel(content_layout)
        
        # Right side: Tabbed interface
        self.setup_tabbed_panels(content_layout)
        
        main_layout.addLayout(content_layout)
        
    def setup_connection_panel(self, parent):
        conn_frame = QFrame()
        conn_frame.setFixedWidth(200)
        parent.addWidget(conn_frame)
        
        layout = QVBoxLayout(conn_frame)
        layout.setContentsMargins(8, 5, 8, 5)
        
        # Title
        title = QLabel("Connection")
        title.setFont(QFont("Arial", 10, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Connection buttons (horizontal)
        button_layout = QHBoxLayout()
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['success']}; min-width: 60px; }}")
        self.connect_btn.clicked.connect(self.connect_mavlink)
        button_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['warning']}; min-width: 60px; }}")
        self.disconnect_btn.clicked.connect(self.disconnect_mavlink)
        self.disconnect_btn.setEnabled(False)
        button_layout.addWidget(self.disconnect_btn)
        
        layout.addLayout(button_layout)
        
        # Connection status (smaller)
        self.status_label = QLabel("Disconnected")
        self.status_label.setFont(QFont("Arial", 8))
        self.status_label.setStyleSheet(f"color: {self.colors['warning']};")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
    def setup_mission_control_panel(self, parent):
        mission_frame = QFrame()
        mission_frame.setFixedWidth(280)
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
        
    def setup_map_panel(self, parent):
        map_frame = QFrame()
        parent.addWidget(map_frame)
        
        layout = QVBoxLayout(map_frame)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Title
        title = QLabel("Interactive Mission Map")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Map graphics view
        self.map_view = QGraphicsView()
        self.map_view.setMinimumSize(600, 400)
        self.map_scene = QGraphicsScene()
        self.map_view.setScene(self.map_scene)
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.mousePressEvent = self.map_mouse_press
        layout.addWidget(self.map_view)
        
        # Map controls
        controls_layout = QHBoxLayout()
        
        clear_btn = QPushButton("Clear Waypoints")
        clear_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['warning']}; }}")
        clear_btn.clicked.connect(self.clear_waypoints)
        controls_layout.addWidget(clear_btn)
        
        center_btn = QPushButton("Center on Drone")
        center_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['accent']}; }}")
        center_btn.clicked.connect(self.center_on_drone)
        controls_layout.addWidget(center_btn)
        
        controls_layout.addStretch()
        layout.addLayout(controls_layout)
        
        # Waypoint list
        self.waypoint_listbox = QListWidget()
        self.waypoint_listbox.setMaximumHeight(80)
        layout.addWidget(self.waypoint_listbox)
        
        self.draw_map()
        
    def setup_tabbed_panels(self, parent):
        # Right side tabbed interface
        tab_widget = QTabWidget()
        tab_widget.setFixedWidth(350)
        parent.addWidget(tab_widget)
        
        # Telemetry tab
        telemetry_tab = QWidget()
        tab_widget.addTab(telemetry_tab, "Telemetry")
        self.setup_telemetry_panel(telemetry_tab)
        
        # Parameters tab
        parameters_tab = QWidget()
        tab_widget.addTab(parameters_tab, "Parameters")
        self.setup_parameters_panel(parameters_tab)
        
        # Mission tab
        mission_tab = QWidget()
        tab_widget.addTab(mission_tab, "Mission")
        self.setup_mission_tab_panel(mission_tab)
        
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
        layout.addWidget(self.system_status)
   
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
        
        # Upload mission button
        self.upload_mission_btn = QPushButton("Upload Mission")
        self.upload_mission_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['info']}; min-height: 30px; }}")
        self.upload_mission_btn.clicked.connect(self.upload_mission)
        mission_controls.addWidget(self.upload_mission_btn)
        
        # Start mission button  
        self.start_mission_btn = QPushButton("Start Mission")
        self.start_mission_btn.setStyleSheet(f"QPushButton {{ background-color: {self.colors['success']}; min-height: 30px; }}")
        self.start_mission_btn.clicked.connect(self.start_mission)
        mission_controls.addWidget(self.start_mission_btn)
        
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
        self.telem_labels['battery'].setText(f"{self.telemetry_data['battery']:.0f}%")
        self.telem_labels['armed'].setText("YES" if self.telemetry_data['armed'] else "NO")
        self.telem_labels['mode'].setText(self.telemetry_data['mode'])
        self.telem_labels['gps_status'].setText(self.telemetry_data.get('gps_status', '--'))
        
    def draw_map(self):
        if not hasattr(self, 'map_scene'):
            return
            
        self.map_scene.clear()
        
        # Draw grid
        width = 600
        height = 400
        
        # Set scene rect
        self.map_scene.setSceneRect(0, 0, width, height)
        
        # Draw grid lines
        grid_pen = QPen(QColor(self.colors['divider']), 1)
        for i in range(0, width, 50):
            self.map_scene.addLine(i, 0, i, height, grid_pen)
        for i in range(0, height, 50):
            self.map_scene.addLine(0, i, width, i, grid_pen)
            
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
        # Simple conversion for demonstration
        width = 600
        height = 400
        
        x = width/2 + (lon - self.map_center['lon']) * 10000
        y = height/2 - (lat - self.map_center['lat']) * 10000
        
        return int(x), int(y)
        
    def pixels_to_lat_lon(self, x, y):
        width = 600
        height = 400
        
        lon = self.map_center['lon'] + (x - width/2) / 10000
        lat = self.map_center['lat'] - (y - height/2) / 10000
        
        return lat, lon
    
    def map_mouse_press(self, event):
        if event.button() == Qt.LeftButton:
            # Convert view coordinates to scene coordinates
            scene_pos = self.map_view.mapToScene(event.pos())
            self.add_waypoint_at_pos(scene_pos.x(), scene_pos.y())
        
    def add_waypoint_at_pos(self, x, y):
        lat, lon = self.pixels_to_lat_lon(x, y)
        alt = float(self.mission_alt_entry.text())
        
        waypoint = {'lat': lat, 'lon': lon, 'alt': alt}
        self.waypoints.append(waypoint)
        
        # Update waypoint list
        self.waypoint_listbox.addItem(f"WP{len(self.waypoints)}: {lat:.6f}, {lon:.6f}, {alt}m")
        
        self.draw_map()
        self.log_message(f"Added waypoint: {lat:.6f}, {lon:.6f}")
        
    def clear_waypoints(self):
        self.waypoints.clear()
        self.waypoint_listbox.clear()
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
            
    def upload_mission(self):
        if not self.connected:
            QMessageBox.warning(self, "Warning", "Not connected to drone!")
            return
        if not self.waypoints:
            QMessageBox.warning(self, "Warning", "Add waypoints before uploading mission!")
            return
        if not self.event_loop:
            QMessageBox.critical(self, "Error", "Event loop not running. Connect to the drone first.")
            return
            
        self.log_message("Uploading mission...")
        self.mission_status.setText("Uploading mission...")
        asyncio.run_coroutine_threadsafe(self._upload_mission_async(), self.event_loop)
            
    async def _upload_mission_async(self):
        try:
            from mavsdk.mission import MissionItem, MissionPlan
            
            # Clear any existing mission first
            await self.drone.mission.clear_mission()
            self.log_message("Cleared existing mission")
            
            # Create mission items
            mission_items = []
            speed = float(self.speed_entry.text())
            
            for i, wp in enumerate(self.waypoints):
                # Create waypoint mission item
                mission_item = MissionItem(
                    wp['lat'],                          # latitude_deg
                    wp['lon'],                          # longitude_deg  
                    wp['alt'],                          # relative_altitude_m
                    speed,                              # speed_m_s
                    True,                               # is_fly_through
                    float('nan'),                       # gimbal_pitch_deg
                    float('nan'),                       # gimbal_yaw_deg
                    MissionItem.CameraAction.NONE,     # camera_action
                    float('nan'),                       # loiter_time_s
                    float('nan'),                       # camera_photo_interval_s
                    2.0,                                # acceptance_radius_m (important!)
                    float('nan'),                       # yaw_deg
                    float('nan')                        # camera_photo_distance_m
                )
                mission_items.append(mission_item)
                self.log_message(f"Created waypoint {i+1}: {wp['lat']:.6f}, {wp['lon']:.6f}, {wp['alt']}m")
            
            # Create mission plan
            mission_plan = MissionPlan(mission_items)
            self.log_message(f"Created mission plan with {len(mission_items)} items")
            
            # Upload mission to drone
            self.log_message("Uploading mission to drone...")
            await self.drone.mission.upload_mission(mission_plan)
            self.log_message(f"✓ Mission uploaded successfully: {len(self.waypoints)} waypoints")
            
            # Verify upload by downloading mission
            downloaded_mission = await self.drone.mission.download_mission()
            self.log_message(f"✓ Mission verification: {len(downloaded_mission.mission_items)} items on drone")
            
            # Update UI in main thread
            QTimer.singleShot(0, lambda: self.mission_status.setText(f"Mission uploaded! ({len(self.waypoints)} waypoints)"))
            
        except Exception as e:
            error_msg = f"Mission upload failed: {str(e)}"
            self.log_message(error_msg)
            QTimer.singleShot(0, lambda: self.mission_status.setText("Upload failed!"))
            QTimer.singleShot(0, lambda: QMessageBox.critical(
                self, "Upload Error", error_msg
            ))
            
    def start_mission(self):
        if not self.connected:
            QMessageBox.warning(self, "Warning", "Not connected to drone!")
            return
        if not self.event_loop:
            QMessageBox.critical(self, "Error", "Event loop not running. Connect to the drone first.")
            return
            
        self.log_message("Starting mission...")
        self.mission_status.setText("Starting mission...")
        asyncio.run_coroutine_threadsafe(self._start_mission_async(), self.event_loop)
            
    async def _start_mission_async(self):
        try:
            # Check if drone is armed and ready
            if not self.telemetry_data.get('armed', False):
                self.log_message("⚠️ Drone must be armed before starting mission")
                QTimer.singleShot(0, lambda: QMessageBox.warning(
                    self, "Mission Warning", "Drone must be armed before starting mission!"
                ))
                return
            
            # Check flight mode - should be in a mode that supports missions
            current_mode = self.telemetry_data.get('mode', 'UNKNOWN')
            self.log_message(f"Current flight mode: {current_mode}")
            
            # Start the mission
            await self.drone.mission.start_mission()
            self.log_message("✓ Mission started successfully!")
            
            # Start monitoring mission progress
            asyncio.create_task(self.monitor_mission_progress())
            
            # Update UI in main thread
            QTimer.singleShot(0, lambda: self.mission_status.setText("Mission running!"))
            
        except Exception as e:
            error_msg = f"Mission start failed: {str(e)}"
            self.log_message(error_msg)
            QTimer.singleShot(0, lambda: self.mission_status.setText("Mission start failed!"))
            QTimer.singleShot(0, lambda: QMessageBox.critical(
                self, "Mission Error", error_msg
            ))
            
    async def monitor_mission_progress(self):
        """Monitor mission progress and update status"""
        try:
            async for progress in self.drone.mission.mission_progress():
                current = progress.current
                total = progress.total
                progress_text = f"Mission: {current}/{total}"
                
                self.log_message(f"Mission progress: waypoint {current}/{total}")
                QTimer.singleShot(0, lambda: self.mission_status.setText(progress_text))
                
                # Mission completed
                if current >= total and total > 0:
                    self.log_message("✓ Mission completed!")
                    QTimer.singleShot(0, lambda: self.mission_status.setText("Mission completed!"))
                    break
                    
        except Exception as e:
            self.log_message(f"Mission progress monitoring error: {str(e)}")
            
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
                    float('nan')                        # camera_photo_distance_m
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