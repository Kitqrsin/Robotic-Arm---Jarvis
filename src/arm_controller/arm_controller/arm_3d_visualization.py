#!/usr/bin/env python3
"""
3D Visualization Canvas with Draggable IK Target Point
This module provides a 3D visualization of the robot arm with an interactive
target point that can be dragged to control the arm using inverse kinematics.
"""

import tkinter as tk
from tkinter import ttk
import math
import numpy as np


class Arm3DVisualization:
    """
    3D visualization of the robot arm with a draggable IK target point.
    Uses a simple orthographic projection to render the arm and target.
    """
    
    def __init__(self, parent_frame, ik_solver, on_target_move_callback=None):
        """
        Initialize the 3D visualization.
        
        Args:
            parent_frame: Tkinter parent frame to embed the canvas
            ik_solver: IK solver instance for forward/inverse kinematics
            on_target_move_callback: Callback function when target is moved
                                     Called with (x, y, z) in meters
        """
        self.parent = parent_frame
        self.ik_solver = ik_solver
        self.on_target_move = on_target_move_callback
        
        # Canvas dimensions
        self.canvas_width = 400
        self.canvas_height = 350
        
        # View parameters
        self.view_azimuth = 45  # Degrees around vertical axis
        self.view_elevation = 25  # Degrees above horizontal
        self.zoom = 800  # Pixels per meter
        self.center_offset_x = self.canvas_width // 2
        self.center_offset_y = self.canvas_height - 50
        
        # Target position (meters)
        self.target_x = 0.2
        self.target_y = 0.0
        self.target_z = 0.15
        
        # Current joint angles (degrees)
        self.joint_angles = [90.0] * 6
        
        # Link lengths (from IK solver)
        self.L1 = self.ik_solver.L1
        self.L2 = self.ik_solver.L2
        self.L3 = self.ik_solver.L3
        self.L4 = self.ik_solver.L4
        
        # Dragging state
        self.dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.rotating_view = False
        
        # Create UI
        self.setup_ui()
        
        # Initial draw
        self.draw_scene()
    
    def setup_ui(self):
        """Setup the visualization UI"""
        
        # Main frame
        self.frame = tk.LabelFrame(
            self.parent,
            text="3D View - Drag Orange Point for IK",
            font=('Arial', 10, 'bold'),
            bg='#3b3b3b',
            fg='white',
            bd=2
        )
        self.frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Canvas for 3D rendering
        self.canvas = tk.Canvas(
            self.frame,
            width=self.canvas_width,
            height=self.canvas_height,
            bg='#1a1a2e',
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Bind mouse events
        self.canvas.bind('<Button-1>', self.on_mouse_down)
        self.canvas.bind('<B1-Motion>', self.on_mouse_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_mouse_up)
        self.canvas.bind('<Button-3>', self.on_right_mouse_down)
        self.canvas.bind('<B3-Motion>', self.on_right_mouse_drag)
        self.canvas.bind('<ButtonRelease-3>', self.on_right_mouse_up)
        self.canvas.bind('<MouseWheel>', self.on_mouse_wheel)
        self.canvas.bind('<Button-4>', self.on_mouse_wheel_up)
        self.canvas.bind('<Button-5>', self.on_mouse_wheel_down)
        
        # View controls frame
        controls_frame = tk.Frame(self.frame, bg='#3b3b3b')
        controls_frame.pack(fill=tk.X, padx=5, pady=2)
        
        # View preset buttons
        tk.Button(
            controls_frame,
            text="Front",
            font=('Arial', 8),
            bg='#4a4a4a',
            fg='white',
            command=lambda: self.set_view(0, 0)
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            controls_frame,
            text="Side",
            font=('Arial', 8),
            bg='#4a4a4a',
            fg='white',
            command=lambda: self.set_view(90, 0)
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            controls_frame,
            text="Top",
            font=('Arial', 8),
            bg='#4a4a4a',
            fg='white',
            command=lambda: self.set_view(0, 90)
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            controls_frame,
            text="Isometric",
            font=('Arial', 8),
            bg='#4a4a4a',
            fg='white',
            command=lambda: self.set_view(45, 25)
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            controls_frame,
            text="Reset Target",
            font=('Arial', 8),
            bg='#006600',
            fg='white',
            command=self.reset_target
        ).pack(side=tk.RIGHT, padx=2)
        
        # Position display
        self.pos_label = tk.Label(
            self.frame,
            text=f"Target: X={self.target_x*1000:.0f} Y={self.target_y*1000:.0f} Z={self.target_z*1000:.0f} mm",
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#00ffff'
        )
        self.pos_label.pack(pady=2)
        
        # Instructions
        tk.Label(
            self.frame,
            text="Left-click: drag target | Right-click: rotate view | Scroll: zoom",
            font=('Arial', 8),
            bg='#3b3b3b',
            fg='#888888'
        ).pack(pady=1)
    
    def project_3d_to_2d(self, x, y, z):
        """
        Project a 3D point to 2D screen coordinates using orthographic projection.
        
        Args:
            x, y, z: 3D coordinates in meters
            
        Returns:
            tuple: (screen_x, screen_y)
        """
        # Convert view angles to radians
        az = math.radians(self.view_azimuth)
        el = math.radians(self.view_elevation)
        
        # Rotate around vertical axis (azimuth)
        x_rot = x * math.cos(az) - y * math.sin(az)
        y_rot = x * math.sin(az) + y * math.cos(az)
        z_rot = z
        
        # Rotate around horizontal axis (elevation)
        y_final = y_rot * math.cos(el) - z_rot * math.sin(el)
        z_final = y_rot * math.sin(el) + z_rot * math.cos(el)
        
        # Project to screen (orthographic)
        screen_x = self.center_offset_x + x_rot * self.zoom
        screen_y = self.center_offset_y - z_final * self.zoom
        
        return screen_x, screen_y
    
    def draw_scene(self):
        """Draw the complete 3D scene"""
        self.canvas.delete('all')
        
        # Draw grid on XY plane
        self.draw_grid()
        
        # Draw coordinate axes
        self.draw_axes()
        
        # Draw the robot arm
        self.draw_arm()
        
        # Draw workspace boundary (sphere showing reachable area)
        self.draw_workspace()
        
        # Draw the target point (last, so it's on top)
        self.draw_target()
        
        # Draw actual end effector position marker
        self.draw_end_effector_marker()
    
    def draw_grid(self):
        """Draw a grid on the XY plane"""
        grid_size = 0.3  # 300mm
        grid_step = 0.05  # 50mm
        
        for i in np.arange(-grid_size, grid_size + grid_step, grid_step):
            # Lines parallel to X
            x1, y1 = self.project_3d_to_2d(i, -grid_size, 0)
            x2, y2 = self.project_3d_to_2d(i, grid_size, 0)
            self.canvas.create_line(x1, y1, x2, y2, fill='#333355', width=1)
            
            # Lines parallel to Y
            x1, y1 = self.project_3d_to_2d(-grid_size, i, 0)
            x2, y2 = self.project_3d_to_2d(grid_size, i, 0)
            self.canvas.create_line(x1, y1, x2, y2, fill='#333355', width=1)
    
    def draw_axes(self):
        """Draw coordinate axes"""
        origin = self.project_3d_to_2d(0, 0, 0)
        axis_length = 0.1  # 100mm
        
        # X-axis (red)
        x_end = self.project_3d_to_2d(axis_length, 0, 0)
        self.canvas.create_line(origin[0], origin[1], x_end[0], x_end[1], 
                               fill='#ff4444', width=2, arrow=tk.LAST)
        self.canvas.create_text(x_end[0] + 10, x_end[1], text='X', fill='#ff4444', font=('Arial', 9))
        
        # Y-axis (green)
        y_end = self.project_3d_to_2d(0, axis_length, 0)
        self.canvas.create_line(origin[0], origin[1], y_end[0], y_end[1], 
                               fill='#44ff44', width=2, arrow=tk.LAST)
        self.canvas.create_text(y_end[0] + 10, y_end[1], text='Y', fill='#44ff44', font=('Arial', 9))
        
        # Z-axis (blue)
        z_end = self.project_3d_to_2d(0, 0, axis_length)
        self.canvas.create_line(origin[0], origin[1], z_end[0], z_end[1], 
                               fill='#4444ff', width=2, arrow=tk.LAST)
        self.canvas.create_text(z_end[0] + 10, z_end[1], text='Z', fill='#4444ff', font=('Arial', 9))
    
    def draw_workspace(self):
        """Draw approximate reachable workspace boundary"""
        # Draw a few circles at different heights to show workspace
        max_reach = self.L2 + self.L3 + self.L4
        
        for z in [0.05, 0.15, 0.25]:
            # Calculate radius at this height
            z_from_shoulder = z - self.L1
            if abs(z_from_shoulder) < max_reach:
                radius = math.sqrt(max_reach**2 - z_from_shoulder**2) if max_reach > abs(z_from_shoulder) else 0
                if radius > 0:
                    # Draw circle using line segments
                    points = []
                    for angle in range(0, 361, 15):
                        rad = math.radians(angle)
                        px = radius * math.cos(rad)
                        py = radius * math.sin(rad)
                        sx, sy = self.project_3d_to_2d(px, py, z)
                        points.extend([sx, sy])
                    
                    if len(points) >= 4:
                        self.canvas.create_line(points, fill='#334433', width=1, smooth=True)
    
    def draw_arm(self):
        """Draw the robot arm based on current joint angles"""
        # Get joint positions using forward kinematics
        positions = self.calculate_arm_positions()
        
        # Draw links
        colors = ['#888888', '#666699', '#669966', '#996666', '#669999', '#999966']
        
        for i in range(len(positions) - 1):
            p1 = positions[i]
            p2 = positions[i + 1]
            
            x1, y1 = self.project_3d_to_2d(*p1)
            x2, y2 = self.project_3d_to_2d(*p2)
            
            # Draw link
            self.canvas.create_line(x1, y1, x2, y2, fill=colors[i % len(colors)], width=6)
            
            # Draw joint
            self.canvas.create_oval(x1 - 5, y1 - 5, x1 + 5, y1 + 5, 
                                   fill='#aaaaaa', outline='#ffffff', width=1)
        
        # Draw end effector
        end_pos = positions[-1]
        ex, ey = self.project_3d_to_2d(*end_pos)
        self.canvas.create_oval(ex - 6, ey - 6, ex + 6, ey + 6,
                               fill='#44ff44', outline='#ffffff', width=2)
    
    def calculate_arm_positions(self):
        """Calculate 3D positions of all arm joints using forward kinematics"""
        # Extract angles
        base = self.joint_angles[0]
        shoulder = self.joint_angles[1]
        elbow = self.joint_angles[2]
        forearm = self.joint_angles[3]
        wrist = self.joint_angles[4]
        
        # Get home positions from IK solver
        home = self.ik_solver.home_position
        
        # Convert to radians relative to home
        base_rad = math.radians(base - home['base'])
        shoulder_rad = math.radians(180 - shoulder)
        elbow_rad = math.radians(180 - elbow)
        forearm_rad = math.radians(forearm - home['forearm'])
        
        # Position calculations
        positions = []
        
        # Base (origin)
        positions.append((0, 0, 0))
        
        # Shoulder (top of base)
        positions.append((0, 0, self.L1))
        
        # Elbow
        elbow_x = self.L2 * math.cos(shoulder_rad)
        elbow_z = self.L1 + self.L2 * math.sin(shoulder_rad)
        positions.append((
            elbow_x * math.cos(base_rad),
            elbow_x * math.sin(base_rad),
            elbow_z
        ))
        
        # Wrist
        cumulative_angle = shoulder_rad - elbow_rad
        wrist_x = elbow_x + self.L3 * math.cos(cumulative_angle)
        wrist_z = elbow_z + self.L3 * math.sin(cumulative_angle)
        positions.append((
            wrist_x * math.cos(base_rad),
            wrist_x * math.sin(base_rad),
            wrist_z
        ))
        
        # End effector
        pitch = cumulative_angle + forearm_rad
        end_x = wrist_x + self.L4 * math.cos(pitch)
        end_z = wrist_z + self.L4 * math.sin(pitch)
        positions.append((
            end_x * math.cos(base_rad),
            end_x * math.sin(base_rad),
            end_z
        ))
        
        return positions
    
    def draw_target(self):
        """Draw the draggable target point"""
        tx, ty = self.project_3d_to_2d(self.target_x, self.target_y, self.target_z)
        
        # Draw target sphere (larger, orange)
        radius = 12
        self.canvas.create_oval(
            tx - radius, ty - radius, tx + radius, ty + radius,
            fill='#ff8800', outline='#ffffff', width=2,
            tags='target'
        )
        
        # Draw crosshair
        self.canvas.create_line(tx - radius - 5, ty, tx + radius + 5, ty, 
                               fill='#ffaa00', width=1, tags='target')
        self.canvas.create_line(tx, ty - radius - 5, tx, ty + radius + 5, 
                               fill='#ffaa00', width=1, tags='target')
        
        # Draw projection lines to ground
        ground_x, ground_y = self.project_3d_to_2d(self.target_x, self.target_y, 0)
        self.canvas.create_line(tx, ty, ground_x, ground_y, 
                               fill='#cc6600', width=1, dash=(3, 3), tags='target')
        self.canvas.create_oval(
            ground_x - 4, ground_y - 4, ground_x + 4, ground_y + 4,
            fill='#ff8800', outline='', tags='target'
        )
    
    def draw_end_effector_marker(self):
        """Draw marker showing actual end effector position from forward kinematics"""
        # Get actual end effector position from arm
        positions = self.calculate_arm_positions()
        if len(positions) > 0:
            end_pos = positions[-1]  # Last position is end effector
            ex, ey = self.project_3d_to_2d(*end_pos)
            
            # Draw small green circle to show actual gripper position
            radius = 8
            self.canvas.create_oval(
                ex - radius, ey - radius, ex + radius, ey + radius,
                fill='#00ff00', outline='#ffffff', width=2
            )
            
            # Label
            self.canvas.create_text(
                ex, ey - radius - 12,
                text='Gripper',
                fill='#00ff00',
                font=('Arial', 8, 'bold')
            )
    
    def set_view(self, azimuth, elevation):
        """Set the view angle"""
        self.view_azimuth = azimuth
        self.view_elevation = elevation
        self.draw_scene()
    
    def reset_target(self):
        """Reset target to default position"""
        self.target_x = 0.2
        self.target_y = 0.0
        self.target_z = 0.15
        self.update_position_label()
        self.draw_scene()
        
        if self.on_target_move:
            self.on_target_move(self.target_x, self.target_y, self.target_z)
    
    def update_position_label(self):
        """Update the position display label"""
        self.pos_label.config(
            text=f"Target: X={self.target_x*1000:.0f} Y={self.target_y*1000:.0f} Z={self.target_z*1000:.0f} mm"
        )
    
    def is_click_on_target(self, screen_x, screen_y):
        """Check if a click is on the target point"""
        tx, ty = self.project_3d_to_2d(self.target_x, self.target_y, self.target_z)
        distance = math.sqrt((screen_x - tx)**2 + (screen_y - ty)**2)
        return distance < 20  # 20 pixel tolerance
    
    def on_mouse_down(self, event):
        """Handle left mouse button down"""
        if self.is_click_on_target(event.x, event.y):
            self.dragging = True
            self.drag_start_x = event.x
            self.drag_start_y = event.y
    
    def on_mouse_drag(self, event):
        """Handle left mouse drag"""
        if self.dragging:
            # Calculate movement in screen space
            dx = event.x - self.drag_start_x
            dy = event.y - self.drag_start_y
            
            # Convert to 3D movement based on view angle
            az = math.radians(self.view_azimuth)
            el = math.radians(self.view_elevation)
            
            # Movement scale
            scale = 1.0 / self.zoom
            
            # Calculate movement in camera space
            # Horizontal screen movement affects X and Y based on azimuth
            # Right movement in screen = positive along camera's right vector
            camera_right_x = -math.sin(az)  # Perpendicular to view direction
            camera_right_y = math.cos(az)
            
            # Vertical screen movement affects Z and the depth direction
            # Up movement = positive Z minus some depth based on elevation
            vertical_z_component = math.cos(el)
            vertical_depth_component = math.sin(el)
            
            # Apply movements
            world_dx = dx * scale * camera_right_x
            world_dy = dx * scale * camera_right_y
            
            # For vertical: mostly Z, but also move toward/away from camera based on elevation
            world_dz = -dy * scale * vertical_z_component
            # Also move in XY plane based on elevation and azimuth
            world_dx += dy * scale * vertical_depth_component * math.cos(az)
            world_dy += dy * scale * vertical_depth_component * math.sin(az)
            
            # Update target position
            self.target_x += world_dx
            self.target_y += world_dy
            self.target_z += world_dz
            
            # Clamp to reasonable workspace
            self.target_x = max(-0.35, min(0.35, self.target_x))
            self.target_y = max(-0.35, min(0.35, self.target_y))
            self.target_z = max(0.0, min(0.4, self.target_z))
            
            # Update drag start for next frame
            self.drag_start_x = event.x
            self.drag_start_y = event.y
            
            # Update display and callback
            self.update_position_label()
            self.draw_scene()
            
            if self.on_target_move:
                self.on_target_move(self.target_x, self.target_y, self.target_z)
    
    def on_mouse_up(self, event):
        """Handle left mouse button release"""
        self.dragging = False
    
    def on_right_mouse_down(self, event):
        """Handle right mouse button down (view rotation)"""
        self.rotating_view = True
        self.drag_start_x = event.x
        self.drag_start_y = event.y
    
    def on_right_mouse_drag(self, event):
        """Handle right mouse drag (view rotation)"""
        if self.rotating_view:
            dx = event.x - self.drag_start_x
            dy = event.y - self.drag_start_y
            
            # Update view angles
            self.view_azimuth += dx * 0.5
            self.view_elevation += dy * 0.5
            
            # Clamp elevation
            self.view_elevation = max(-89, min(89, self.view_elevation))
            
            # Normalize azimuth
            self.view_azimuth = self.view_azimuth % 360
            
            self.drag_start_x = event.x
            self.drag_start_y = event.y
            
            self.draw_scene()
    
    def on_right_mouse_up(self, event):
        """Handle right mouse button release"""
        self.rotating_view = False
    
    def on_mouse_wheel(self, event):
        """Handle mouse wheel (zoom)"""
        if event.delta > 0:
            self.zoom *= 1.1
        else:
            self.zoom /= 1.1
        
        self.zoom = max(200, min(2000, self.zoom))
        self.draw_scene()
    
    def on_mouse_wheel_up(self, event):
        """Handle mouse wheel up (Linux)"""
        self.zoom *= 1.1
        self.zoom = min(2000, self.zoom)
        self.draw_scene()
    
    def on_mouse_wheel_down(self, event):
        """Handle mouse wheel down (Linux)"""
        self.zoom /= 1.1
        self.zoom = max(200, self.zoom)
        self.draw_scene()
    
    def set_target(self, x, y, z):
        """Set target position programmatically"""
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.update_position_label()
        self.draw_scene()
    
    def set_joint_angles(self, angles):
        """Update joint angles for arm visualization"""
        self.joint_angles = list(angles)
        self.draw_scene()
    
    def update_from_ik_solution(self, joint_angles_dict):
        """Update arm visualization from IK solution dictionary"""
        ik_to_idx = {
            'base': 0,
            'shoulder': 1,
            'elbow': 2,
            'forearm': 3,
            'wrist': 4,
            'gripper': 5
        }
        
        for joint, idx in ik_to_idx.items():
            if joint in joint_angles_dict:
                self.joint_angles[idx] = joint_angles_dict[joint]
        
        self.draw_scene()
