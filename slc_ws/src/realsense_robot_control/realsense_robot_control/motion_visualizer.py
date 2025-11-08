#!/usr/bin/env python3

import sys
import numpy as np
import json
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                            QLabel, QTextEdit, QGroupBox, QSlider)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush, QFont
import math

class Motion3DViewer(QWidget):
    """3D Motion path visualization widget"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.motion_data = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.update_animation)
        self.animation_progress = 0.0
        self.is_animating = False
        
        # Visualization parameters
        self.scale = 500  # pixels per meter
        self.origin_x = 400
        self.origin_y = 300
        self.rotation_x = 30  # degrees
        self.rotation_z = 45  # degrees
        
        self.setMinimumSize(800, 600)
        self.setStyleSheet("background-color: #f0f0f0;")
        
    def load_motion_data(self, motion_data):
        """Load motion data for visualization"""
        self.motion_data = motion_data
        self.waypoints = motion_data.get('waypoints', [])
        self.current_waypoint_index = 0
        self.animation_progress = 0.0
        self.update()
        
    def load_motion_from_file(self, file_path):
        """Load motion data from JSON file"""
        try:
            with open(file_path, 'r') as f:
                motion_data = json.load(f)
                self.load_motion_data(motion_data)
                return True
        except Exception as e:
            print(f"Error loading motion file: {e}")
            return False
    
    def project_3d_to_2d(self, point_3d):
        """Project 3D point to 2D screen coordinates"""
        x, y, z = point_3d
        
        # Apply rotations
        # Rotation around X axis
        rad_x = math.radians(self.rotation_x)
        y_rot = y * math.cos(rad_x) - z * math.sin(rad_x)
        z_rot = y * math.sin(rad_x) + z * math.cos(rad_x)
        
        # Rotation around Z axis
        rad_z = math.radians(self.rotation_z)
        x_rot = x * math.cos(rad_z) - y_rot * math.sin(rad_z)
        y_final = x * math.sin(rad_z) + y_rot * math.cos(rad_z)
        
        # Project to 2D (simple orthographic projection)
        screen_x = self.origin_x + x_rot * self.scale
        screen_y = self.origin_y - z_rot * self.scale  # Flip Y axis
        
        return int(screen_x), int(screen_y)
    
    def paintEvent(self, event):
        """Paint the motion path"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw coordinate axes
        self.draw_axes(painter)
        
        if not self.waypoints:
            # Draw placeholder text
            painter.setPen(QPen(QColor(100, 100, 100), 2))
            painter.setFont(QFont("Arial", 14))
            painter.drawText(self.rect(), Qt.AlignCenter, "No motion data loaded")
            return
        
        # Draw the motion path
        self.draw_motion_path(painter)
        
        # Draw waypoints
        self.draw_waypoints(painter)
        
        # Draw current position during animation
        if self.is_animating:
            self.draw_current_position(painter)
        
        # Draw information
        self.draw_info(painter)
    
    def draw_axes(self, painter):
        """Draw 3D coordinate axes"""
        # Origin
        origin_2d = self.project_3d_to_2d([0, 0, 0])
        
        # X axis (red)
        x_axis_2d = self.project_3d_to_2d([0.2, 0, 0])
        painter.setPen(QPen(QColor(255, 0, 0), 3))
        painter.drawLine(origin_2d[0], origin_2d[1], x_axis_2d[0], x_axis_2d[1])
        painter.drawText(x_axis_2d[0] + 5, x_axis_2d[1], "X")
        
        # Y axis (green)
        y_axis_2d = self.project_3d_to_2d([0, 0.2, 0])
        painter.setPen(QPen(QColor(0, 255, 0), 3))
        painter.drawLine(origin_2d[0], origin_2d[1], y_axis_2d[0], y_axis_2d[1])
        painter.drawText(y_axis_2d[0] + 5, y_axis_2d[1], "Y")
        
        # Z axis (blue)
        z_axis_2d = self.project_3d_to_2d([0, 0, 0.2])
        painter.setPen(QPen(QColor(0, 0, 255), 3))
        painter.drawLine(origin_2d[0], origin_2d[1], z_axis_2d[0], z_axis_2d[1])
        painter.drawText(z_axis_2d[0] + 5, z_axis_2d[1], "Z")
    
    def draw_motion_path(self, painter):
        """Draw the motion path lines"""
        if len(self.waypoints) < 2:
            return
        
        # Draw path lines
        painter.setPen(QPen(QColor(50, 50, 200), 2, Qt.DashLine))
        
        for i in range(len(self.waypoints) - 1):
            pos1 = self.waypoints[i]['position']
            pos2 = self.waypoints[i + 1]['position']
            
            p1_2d = self.project_3d_to_2d(pos1)
            p2_2d = self.project_3d_to_2d(pos2)
            
            painter.drawLine(p1_2d[0], p1_2d[1], p2_2d[0], p2_2d[1])
    
    def draw_waypoints(self, painter):
        """Draw waypoint markers"""
        for i, wp in enumerate(self.waypoints):
            pos = wp['position']
            p_2d = self.project_3d_to_2d(pos)
            
            # Different colors for different waypoints
            if i == 0:  # Start
                color = QColor(0, 200, 0)
            elif i == len(self.waypoints) - 1:  # End
                color = QColor(200, 0, 0)
            else:  # Middle waypoints
                color = QColor(100, 100, 255)
            
            # Draw waypoint marker
            painter.setPen(QPen(color, 2))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(p_2d[0] - 6, p_2d[1] - 6, 12, 12)
            
            # Draw waypoint label
            painter.setPen(QPen(QColor(0, 0, 0), 1))
            painter.setFont(QFont("Arial", 10))
            painter.drawText(p_2d[0] + 10, p_2d[1] - 5, wp['name'])
    
    def draw_current_position(self, painter):
        """Draw current position during animation"""
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            return
        
        # Interpolate between waypoints
        wp1 = self.waypoints[self.current_waypoint_index]
        wp2 = self.waypoints[self.current_waypoint_index + 1]
        
        pos1 = np.array(wp1['position'])
        pos2 = np.array(wp2['position'])
        
        # Linear interpolation
        current_pos = pos1 + (pos2 - pos1) * self.animation_progress
        p_2d = self.project_3d_to_2d(current_pos)
        
        # Draw current position marker
        painter.setPen(QPen(QColor(255, 165, 0), 3))
        painter.setBrush(QBrush(QColor(255, 165, 0)))
        painter.drawEllipse(p_2d[0] - 8, p_2d[1] - 8, 16, 16)
        
        # Draw tool orientation (simplified)
        painter.setPen(QPen(QColor(255, 100, 0), 2))
        painter.drawLine(p_2d[0], p_2d[1], p_2d[0], p_2d[1] - 20)
    
    def draw_info(self, painter):
        """Draw information text"""
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.setFont(QFont("Arial", 12))
        
        info_text = []
        if self.motion_data:
            info_text.append(f"Motion Type: {self.motion_data.get('motion_type', 'Unknown')}")
            info_text.append(f"Total Waypoints: {len(self.waypoints)}")
            
            if self.is_animating:
                info_text.append(f"Current: {self.waypoints[self.current_waypoint_index]['name']}")
                info_text.append(f"Progress: {self.animation_progress * 100:.1f}%")
        
        y_offset = 20
        for text in info_text:
            painter.drawText(10, y_offset, text)
            y_offset += 20
    
    def start_animation(self):
        """Start motion animation"""
        if not self.waypoints:
            return
        
        self.is_animating = True
        self.current_waypoint_index = 0
        self.animation_progress = 0.0
        self.animation_timer.start(50)  # 20 FPS
    
    def stop_animation(self):
        """Stop motion animation"""
        self.is_animating = False
        self.animation_timer.stop()
        self.update()
    
    def update_animation(self):
        """Update animation state"""
        if not self.is_animating:
            return
        
        # Update progress
        self.animation_progress += 0.05
        
        if self.animation_progress >= 1.0:
            self.animation_progress = 0.0
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoints) - 1:
                # Animation complete
                self.stop_animation()
                return
        
        self.update()
    
    def set_view_angle(self, rotation_x, rotation_z):
        """Set viewing angle"""
        self.rotation_x = rotation_x
        self.rotation_z = rotation_z
        self.update()
    
    def set_scale(self, scale):
        """Set visualization scale"""
        self.scale = scale
        self.update()


class MotionVisualizerWidget(QWidget):
    """Complete motion visualizer widget with controls"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
        
    def init_ui(self):
        """Initialize the UI"""
        layout = QVBoxLayout()
        
        # Motion viewer
        self.motion_viewer = Motion3DViewer()
        layout.addWidget(self.motion_viewer)
        
        # Control panel
        control_group = QGroupBox("Visualization Controls")
        control_layout = QVBoxLayout()
        
        # View angle controls
        angle_layout = QHBoxLayout()
        
        angle_layout.addWidget(QLabel("X Rotation:"))
        self.x_rotation_slider = QSlider(Qt.Horizontal)
        self.x_rotation_slider.setRange(-90, 90)
        self.x_rotation_slider.setValue(30)
        self.x_rotation_slider.valueChanged.connect(self.update_view_angle)
        angle_layout.addWidget(self.x_rotation_slider)
        
        angle_layout.addWidget(QLabel("Z Rotation:"))
        self.z_rotation_slider = QSlider(Qt.Horizontal)
        self.z_rotation_slider.setRange(0, 360)
        self.z_rotation_slider.setValue(45)
        self.z_rotation_slider.valueChanged.connect(self.update_view_angle)
        angle_layout.addWidget(self.z_rotation_slider)
        
        control_layout.addLayout(angle_layout)
        
        # Scale control
        scale_layout = QHBoxLayout()
        scale_layout.addWidget(QLabel("Scale:"))
        self.scale_slider = QSlider(Qt.Horizontal)
        self.scale_slider.setRange(100, 1000)
        self.scale_slider.setValue(500)
        self.scale_slider.valueChanged.connect(self.update_scale)
        scale_layout.addWidget(self.scale_slider)
        control_layout.addLayout(scale_layout)
        
        # Animation controls
        anim_layout = QHBoxLayout()
        self.play_btn = QPushButton("Play Animation")
        self.play_btn.clicked.connect(self.toggle_animation)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.motion_viewer.stop_animation)
        
        anim_layout.addWidget(self.play_btn)
        anim_layout.addWidget(self.stop_btn)
        control_layout.addLayout(anim_layout)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Motion details
        self.details_text = QTextEdit()
        self.details_text.setReadOnly(True)
        self.details_text.setMaximumHeight(150)
        layout.addWidget(self.details_text)
        
        self.setLayout(layout)
    
    def load_motion(self, motion_data):
        """Load motion data"""
        self.motion_viewer.load_motion_data(motion_data)
        self.update_details(motion_data)
    
    def load_motion_from_file(self, file_path):
        """Load motion from file"""
        if self.motion_viewer.load_motion_from_file(file_path):
            self.update_details(self.motion_viewer.motion_data)
    
    def update_details(self, motion_data):
        """Update motion details display"""
        if not motion_data:
            return
        
        details = []
        details.append(f"Motion Type: {motion_data.get('motion_type', 'Unknown')}")
        details.append(f"Timestamp: {motion_data.get('timestamp', 'Unknown')}")
        details.append(f"Coordinate Frame: {motion_data.get('coordinate_frame', 'Unknown')}")
        details.append(f"Total Waypoints: {motion_data.get('total_waypoints', 0)}")
        
        params = motion_data.get('motion_parameters', {})
        details.append(f"\nMotion Parameters:")
        details.append(f"  Linear Speed: {params.get('linear_speed', 0)} mm/s")
        details.append(f"  Acceleration: {params.get('acceleration', 0)} mm/s²")
        details.append(f"  Path Width: {params.get('path_width', 0)*1000} mm")
        details.append(f"  Path Depth: {params.get('path_depth', 0)*1000} mm")
        
        self.details_text.setText('\n'.join(details))
    
    def toggle_animation(self):
        """Toggle animation play/pause"""
        if self.motion_viewer.is_animating:
            self.motion_viewer.stop_animation()
            self.play_btn.setText("Play Animation")
        else:
            self.motion_viewer.start_animation()
            self.play_btn.setText("Pause")
    
    def update_view_angle(self):
        """Update viewing angle"""
        x_rot = self.x_rotation_slider.value()
        z_rot = self.z_rotation_slider.value()
        self.motion_viewer.set_view_angle(x_rot, z_rot)
    
    def update_scale(self):
        """Update visualization scale"""
        scale = self.scale_slider.value()
        self.motion_viewer.set_scale(scale)