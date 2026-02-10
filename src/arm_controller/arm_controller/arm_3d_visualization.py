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
        self.zoom = 650  # Pixels per meter (reduced from 800 for larger workspace)
        self.center_offset_x = self.canvas_width // 2
        self.center_offset_y = self.canvas_height - 50
        
        # Target position (meters)
        # Increased for 5-joint arm with extended reach
        self.target_x = 0.25
        self.target_y = 0.0
        self.target_z = 0.22
        
        # Current joint angles (degrees)
        self.joint_angles = [90.0] * 6
        
        # Actual gripper position from FK/TF (meters) - separate from computed position
        self.actual_gripper_x = 0.0
        self.actual_gripper_y = 0.0
        self.actual_gripper_z = 0.15
        
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
        
        # Auto-follow mode: gripper tracks target in real-time during drag
        self.auto_follow = False
        self._drag_ik_pending = False
        
        # Callback for when user wants to move arm to target
        self.on_move_to_target = None
        
        # Create UI
        self.setup_ui()
        
        # Initial draw - set target to a reasonable default position
        self.sync_target_to_gripper()
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
        
        # Instructions (updated when auto_follow changes)
        self.instructions_label = tk.Label(
            self.frame,
            text="Drag orange dot to set target | Press Start to enable auto-follow",
            font=('Arial', 8),
            bg='#3b3b3b',
            fg='#888888'
        )
        self.instructions_label.pack(pady=1)
    
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
        
        # Draw the current gripper position (green)
        self.draw_gripper_marker()
        
        # Draw the target point (orange - where user wants to move)
        self.draw_target()
    
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
        # Increased by 1.5x to accommodate new 5th joint
        max_reach = (self.L2 + self.L3 + self.L4) * 1.5
        
        for z in [0.05, 0.15, 0.25, 0.35]:
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
        """Draw the robot arm based on current joint angles.
        
        Draws 5 motor joints:
        1. Base (gray) - ground level
        2. Shoulder (blue) - top of base
        3. Elbow (green) - end of upper arm
        4. Wrist (red) - end of forearm
        5. Gripper (cyan) - where gripper attaches
        """
        # Get joint positions using forward kinematics
        positions = self.calculate_arm_positions()
        
        # Joint colors matching the robot (5 joints)
        joint_colors = ['#888888', '#6666aa', '#66aa66', '#aa6666', '#66aaaa']
        joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist', 'Gripper']
        
        # Link colors
        link_colors = ['#888888', '#666699', '#669966', '#996666']
        
        # Draw links first (behind joints)
        for i in range(len(positions) - 1):
            p1 = positions[i]
            p2 = positions[i + 1]
            
            x1, y1 = self.project_3d_to_2d(*p1)
            x2, y2 = self.project_3d_to_2d(*p2)
            
            # Draw link
            self.canvas.create_line(x1, y1, x2, y2, fill=link_colors[i % len(link_colors)], width=6)
        
        # Draw all 5 joints
        for i, pos in enumerate(positions):
            jx, jy = self.project_3d_to_2d(*pos)
            color = joint_colors[i] if i < len(joint_colors) else '#aaaaaa'
            
            # Draw joint circle (larger for gripper)
            radius = 7 if i == 4 else 5
            self.canvas.create_oval(
                jx - radius, jy - radius, jx + radius, jy + radius,
                fill=color, outline='#ffffff', width=1
            )
    
    def calculate_arm_positions(self):
        """Calculate 3D positions of arm joints for schematic visualization.
        
        This matches the RViz model EXACTLY using URDF joint origins.
        5 motor joints: base, shoulder, elbow, forearm (wrist), gripper
        
        Uses the same angle conversions as send_command() in arm_gui.py:
        - Shoulder and Elbow: INVERTED (90.0 - pos) to match RViz
        - Others: Normal (pos - 90.0)
        
        At 90° (home position), the arm should point straight up.
        """
        # Get joint angles in degrees (GUI convention)
        base = self.joint_angles[0]
        shoulder = self.joint_angles[1]
        elbow = self.joint_angles[2]
        forearm = self.joint_angles[3]
        wrist_rotate = self.joint_angles[4] if len(self.joint_angles) > 4 else 90.0
        
        # Convert to radians for Tkinter visualization
        # MUST MATCH RViz conversions in arm_gui.py send_command()
        
        # Base: 90° = pointing along +X axis (normal conversion)
        base_rad = math.radians(base - 90.0)
        
        # Shoulder: INVERTED to match RViz (90.0 - pos)
        shoulder_rad = math.radians(90.0 - shoulder)
        
        # Elbow: INVERTED to match RViz (90.0 - pos)
        elbow_rad = math.radians(90.0 - elbow)
        
        # Forearm/Wrist pitch - INVERTED for display to match RViz
        # Hardware uses normal convention, but RViz display inverts it
        # So Tkinter must also invert for visual consistency
        forearm_rad = -math.radians(forearm - 90.0)
        
        # Link lengths from URDF joint origin xyz values (in meters)
        # Measured from robot_fixed.urdf joint definitions:
        # base joint: xyz="0.0284017 -0.00137038 0.0592971" -> z=0.059
        # shoulder joint: xyz="-0.0281584 0.0201485 0.0113428" -> ~0.036
        # elbow joint: xyz="-0.000476914 0.0841198 -0.00387217" -> y=0.084
        # wrist joint: xyz="0.0565214 0.0648797 -0.000297974" -> dist=0.086
        # wrist_rotate joint: xyz="0.0475547 0.0385425 0.027714" -> dist=0.061
        L_base = 0.059       # Base to shoulder height (base joint z)
        L_upper = 0.084      # Shoulder to elbow (elbow joint y - main link!)
        L_forearm = 0.086    # Elbow to wrist (wrist joint distance)
        L_wrist = 0.061      # Wrist to gripper (wrist_rotate joint distance)
        
        positions = []
        
        # Joint 1: Base motor (origin at ground level)
        positions.append((0, 0, 0))
        
        # Joint 2: Shoulder motor (top of base pedestal)
        positions.append((0, 0, L_base))
        
        # Joint 3: Elbow motor
        # Shoulder rotates the upper arm in vertical plane
        # At shoulder_rad=0 (GUI 90°), arm points straight up
        elbow_x = L_upper * math.sin(shoulder_rad)
        elbow_z = L_base + L_upper * math.cos(shoulder_rad)
        positions.append((
            elbow_x * math.cos(base_rad),
            elbow_x * math.sin(base_rad),
            elbow_z
        ))
        
        # Joint 4: Forearm/Wrist motor
        # Cumulative angle for forearm
        arm_angle = shoulder_rad + elbow_rad
        wrist_x = elbow_x + L_forearm * math.sin(arm_angle)
        wrist_z = elbow_z + L_forearm * math.cos(arm_angle)
        positions.append((
            wrist_x * math.cos(base_rad),
            wrist_x * math.sin(base_rad),
            wrist_z
        ))
        
        # Joint 5: Gripper motor (wrist rotate / gripper base)
        # This is where the gripper attaches - add forearm pitch
        full_angle = arm_angle + forearm_rad
        gripper_x = wrist_x + L_wrist * math.sin(full_angle)
        gripper_z = wrist_z + L_wrist * math.cos(full_angle)
        positions.append((
            gripper_x * math.cos(base_rad),
            gripper_x * math.sin(base_rad),
            gripper_z
        ))
        
        return positions
    
    def draw_target(self):
        """Draw the draggable target point (orange ball - where user wants arm to go)"""
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
        
        # Label
        self.canvas.create_text(
            tx, ty - radius - 12,
            text='Target',
            fill='#ff8800',
            font=('Arial', 8, 'bold')
        )
    
    def draw_gripper_marker(self):
        """Draw the current gripper position (green ball) - at the 5th joint (gripper base)"""
        # Get the 5th joint position directly from calculated arm positions
        # This ensures the green dot is exactly on the last joint
        positions = self.calculate_arm_positions()
        if len(positions) >= 5:
            gripper_pos = positions[4]  # 5th joint (index 4)
            self.actual_gripper_x = gripper_pos[0]
            self.actual_gripper_y = gripper_pos[1]
            self.actual_gripper_z = gripper_pos[2]
        
        gx, gy = self.project_3d_to_2d(self.actual_gripper_x, self.actual_gripper_y, self.actual_gripper_z)
        
        # Draw gripper marker (green, slightly smaller)
        radius = 8
        self.canvas.create_oval(
            gx - radius, gy - radius, gx + radius, gy + radius,
            fill='#00ff00', outline='#ffffff', width=2
        )
        
        # Label
        self.canvas.create_text(
            gx, gy + radius + 10,
            text='Gripper',
            fill='#00ff00',
            font=('Arial', 8, 'bold')
        )
        
        # Update the position label to show both
        self.update_position_label()
    
    def sync_target_to_gripper(self):
        """
        Sync the target position to the current gripper position
        calculated from the arm's forward kinematics.
        """
        # Use calculate_arm_positions to get actual gripper position
        # This ensures consistency with how the arm is drawn
        positions = self.calculate_arm_positions()
        if len(positions) > 0:
            end_pos = positions[-1]
            self.target_x = end_pos[0]
            self.target_y = end_pos[1]
            self.target_z = end_pos[2]
            self.update_position_label()
    
    def trigger_move_to_target(self):
        """Trigger the callback to move arm to target position"""
        print(f"Tkinter trigger_move_to_target: X={self.target_x:.3f}, Y={self.target_y:.3f}, Z={self.target_z:.3f}")
        if self.on_target_move:
            self.on_target_move(self.target_x, self.target_y, self.target_z)
    
    def _fire_drag_ik(self):
        """Fire the IK callback after a short throttle delay."""
        self._drag_ik_pending = False
        if self.auto_follow and self.on_target_move and self.dragging:
            self.on_target_move(self.target_x, self.target_y, self.target_z)

    def set_auto_follow(self, enabled):
        """Enable or disable automatic gripper-follows-target mode."""
        self.auto_follow = enabled
        if hasattr(self, 'instructions_label') and self.instructions_label:
            if enabled:
                self.instructions_label.config(
                    text="⚡ AUTO-FOLLOW ACTIVE — Drag target, arm follows in real-time",
                    fg='#ffaa00'
                )
            else:
                self.instructions_label.config(
                    text="Drag orange dot to set target | Press Start to enable auto-follow",
                    fg='#888888'
                )

    def set_move_callback(self, callback):
        """Set the callback for when user clicks Move to Target"""
        self.on_move_to_target = callback
    
    def set_view(self, azimuth, elevation):
        """Set the view angle"""
        self.view_azimuth = azimuth
        self.view_elevation = elevation
        self.draw_scene()
    
    def reset_target(self):
        """Reset target to current gripper position"""
        self.sync_target_to_gripper()
        self.draw_scene()
    
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
            
            # The projection uses:
            # screen_x = center + x_rot * zoom, where x_rot = x*cos(az) - y*sin(az)
            # screen_y = center - z_final * zoom, where z_final = y*sin(el) + z*cos(el)
            #
            # So to invert: moving right on screen (positive dx) should increase x_rot
            # And moving up on screen (negative dy) should increase z_final
            
            # For horizontal screen movement (dx):
            # This maps to movement in the x_rot direction
            # x_rot = x*cos(az) - y*sin(az), so we need to move in world coords
            # To increase x_rot: increase x (proportional to cos(az)) or decrease y (proportional to sin(az))
            world_dx_from_horizontal = dx * scale * math.cos(az)
            world_dy_from_horizontal = -dx * scale * math.sin(az)
            
            # For vertical screen movement (dy):
            # screen_y decreases when z_final increases (note the minus sign in projection)
            # z_final = y_rot*sin(el) + z*cos(el)
            # y_rot = x*sin(az) + y*cos(az)
            # Moving up on screen (dy < 0) should increase z_final
            # So we use -dy to get the z increase
            world_dz_from_vertical = -dy * scale * math.cos(el)
            # Also affects y_rot component
            world_dx_from_vertical = -dy * scale * math.sin(el) * math.sin(az)
            world_dy_from_vertical = -dy * scale * math.sin(el) * math.cos(az)
            
            # Combine movements
            world_dx = world_dx_from_horizontal + world_dx_from_vertical
            world_dy = world_dy_from_horizontal + world_dy_from_vertical
            world_dz = world_dz_from_vertical
            
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
            
            # Update display
            self.update_position_label()
            self.draw_scene()
            
            # If auto-follow is active, trigger IK in real-time as user drags
            if self.auto_follow and self.on_target_move and not self._drag_ik_pending:
                self._drag_ik_pending = True
                # Use after_idle to batch rapid drag events
                self.canvas.after(80, self._fire_drag_ik)
    
    def on_mouse_up(self, event):
        """Handle left mouse button release"""
        if self.dragging and self.auto_follow and self.on_target_move:
            # Fire IK one final time at the release position
            self.on_target_move(self.target_x, self.target_y, self.target_z)
        self.dragging = False
        self._drag_ik_pending = False
    
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
        
        # After IK solution, target should already be at desired position
        # (the target was dragged there), so don't sync back
        self.draw_scene()
    
    def set_actual_gripper_position(self, x, y, z):
        """Set the actual gripper position from FK/TF (for accurate green marker)"""
        print(f"Tkinter set_actual_gripper_position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
        self.actual_gripper_x = x
        self.actual_gripper_y = y
        self.actual_gripper_z = z
        self.draw_scene()
