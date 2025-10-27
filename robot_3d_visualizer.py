"""
3D Robot Arm Visualizer for Thor Robot
Displays robot skeleton and end effector trajectory in interactive 3D view
OPTIMIZED: Uses caching and dirty flags to minimize redraws
"""

import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import logging
import threading
import time

import forward_kinematics as fk

logger = logging.getLogger(__name__)


class Robot3DCanvas(FigureCanvas):
    """
    Matplotlib 3D canvas for visualizing robot arm and trajectory
    """

    def __init__(self, parent=None, width=8, height=7, dpi=100):
        """
        Initialize 3D robot visualization canvas with performance optimizations

        Args:
            parent: Parent Qt widget
            width: Figure width in inches
            height: Figure height in inches
            dpi: Dots per inch for rendering
        """
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super().__init__(self.fig)
        self.setParent(parent)

        # Create 3D axes
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Visualization state
        self.current_joint_positions = None
        self.trajectory_points = []
        self.trajectory_timestamps = []

        # Display options
        self.show_robot = True
        self.show_trajectory = True
        self.show_base_frame = True
        self.show_workspace = False
        self.show_grid = True
        self.show_labels = False
        self.auto_rotate = False
        self.rotation_angle = 45  # Current azimuth for auto-rotate

        # Color schemes
        self.colors_active = {
            'link': '#00CED1',      # Cyan
            'joint': '#FFD700',     # Gold
            'tcp': '#FF0000',       # Red
            'base': '#2F4F4F'       # Dark slate gray
        }
        self.colors_inactive = {
            'link': '#CCCCCC',      # Light gray
            'joint': '#666666',     # Dark gray
            'tcp': '#999999',       # Medium gray
            'base': '#2F4F4F'       # Dark slate gray
        }

        # PERFORMANCE OPTIMIZATION: Dirty flag pattern
        self._is_dirty = True  # Needs redraw
        self._last_joint_angles = None  # Cache last angles to detect changes
        self._last_trajectory_length = 0  # Cache trajectory length
        self._render_lock = threading.Lock()  # Thread-safe rendering
        self._pending_update = False  # Debounce rapid updates

        # FK calculation cache (avoid redundant calculations)
        self._fk_cache = {}
        self._fk_cache_lock = threading.Lock()

        # Setup initial view
        self.setup_3d_axes()

        # Show home position immediately
        self.show_home_position()

        logger.info("3D robot canvas initialized with performance optimizations")

    def setup_3d_axes(self):
        """Setup 3D axes with labels, limits, and view angle"""
        self.ax.clear()

        # Set labels
        self.ax.set_xlabel('X (mm)', fontsize=9)
        self.ax.set_ylabel('Y (mm)', fontsize=9)
        self.ax.set_zlabel('Z (mm)', fontsize=9)

        # Set title
        self.ax.set_title('Thor Robot - 3D Visualization', fontsize=11, fontweight='bold')

        # Get workspace envelope for setting limits
        workspace = fk.compute_workspace_envelope()
        max_reach = workspace['radius']

        # Set axis limits (equal aspect ratio for all axes)
        # Reduced margin for better zoom - 0.85x gives tighter view
        limit = max_reach * 0.85
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([0, workspace['z_max'] * 0.85])

        # Set isometric view (default)
        self.ax.view_init(elev=30, azim=45)

        # Grid
        if self.show_grid:
            self.ax.grid(True, alpha=0.3)

        # Equal aspect ratio
        self.ax.set_box_aspect([1, 1, 1])

    def show_home_position(self):
        """Display robot at ready position (extended arm configuration)"""
        self.setup_3d_axes()

        # Use a "ready" position instead of home (all zeros) for better visualization
        # q1=0°, q2=45° (shoulder up), q3=-90° (elbow bent), q4=0°, q5=45° (wrist), q6=0°
        # This creates a clearly visible arm shape
        ready_positions = fk.compute_all_joint_positions(0, 45, -90, 0, 45, 0)
        self.current_joint_positions = ready_positions

        # Draw robot in inactive colors
        self.draw_robot_arm(ready_positions, active=False)

        # Draw base frame if enabled
        if self.show_base_frame:
            self.draw_base_frame()

        # Add text overlay
        self.add_text_overlay("Robot not connected - Showing ready position", color='gray')

        self.draw()
        logger.info("Showing home position")

    def draw_robot_arm(self, joint_positions, active=True):
        """
        Draw robot arm as connected line segments

        Args:
            joint_positions: List of 8 tuples [(x,y,z), ...] for joints
            active: If True, use active colors; if False, use inactive (gray)
        """
        if joint_positions is None or len(joint_positions) < 2:
            return

        # Select color scheme
        colors = self.colors_active if active else self.colors_inactive
        alpha = 1.0 if active else 0.6

        # Extract coordinates
        xs = [p[0] for p in joint_positions]
        ys = [p[1] for p in joint_positions]
        zs = [p[2] for p in joint_positions]

        # Draw links as thick lines
        self.ax.plot(xs, ys, zs,
                    color=colors['link'],
                    linewidth=4 if active else 3,
                    alpha=alpha,
                    marker='o',
                    markersize=8 if active else 6,
                    markerfacecolor=colors['joint'],
                    markeredgecolor=colors['joint'],
                    markeredgewidth=2,
                    label='Robot Arm' if active else 'Robot Arm (Inactive)')

        # Highlight TCP (last point) with larger marker
        tcp = joint_positions[-1]
        self.ax.scatter([tcp[0]], [tcp[1]], [tcp[2]],
                       color=colors['tcp'],
                       s=200 if active else 150,
                       alpha=alpha,
                       marker='o',
                       edgecolors='black',
                       linewidths=2,
                       label='TCP')

        # Highlight base (first point)
        base = joint_positions[0]
        self.ax.scatter([base[0]], [base[1]], [base[2]],
                       color=colors['base'],
                       s=150,
                       alpha=alpha,
                       marker='s',  # Square marker for base
                       edgecolors='black',
                       linewidths=2,
                       label='Base')

        # Draw joint labels if enabled
        if self.show_labels:
            joint_names = fk.get_joint_names()
            for i, (name, pos) in enumerate(zip(joint_names, joint_positions)):
                self.ax.text(pos[0], pos[1], pos[2], f'  {name}',
                           fontsize=7, color='black' if active else 'gray')

    def draw_tcp_trajectory(self, tcp_points, timestamps):
        """
        Draw TCP trajectory as gradient colored line

        Args:
            tcp_points: List of (x, y, z) tuples
            timestamps: List of timestamps for each point
        """
        if len(tcp_points) < 2:
            return

        # Extract coordinates
        xs = [p[0] for p in tcp_points]
        ys = [p[1] for p in tcp_points]
        zs = [p[2] for p in tcp_points]

        # Create line segments
        points = np.array([xs, ys, zs]).T.reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Create gradient colors (blue to red based on time)
        n_segments = len(segments)
        colors_traj = plt.cm.jet(np.linspace(0, 1, n_segments))

        # Create Line3DCollection with gradient
        lc = Line3DCollection(segments, colors=colors_traj, linewidths=2, alpha=0.8)
        self.ax.add_collection3d(lc)

        # Add label for legend
        # Use dummy line for legend entry
        self.ax.plot([], [], [], color='red', linewidth=2, label='TCP Trajectory', alpha=0.8)

    def draw_base_frame(self, length=100):
        """
        Draw coordinate frame at base (XYZ axes)

        Args:
            length: Length of each axis arrow in mm
        """
        origin = [0, 0, 0]

        # X axis - Red
        self.ax.quiver(origin[0], origin[1], origin[2],
                      length, 0, 0,
                      color='red', arrow_length_ratio=0.15, linewidth=2.5, alpha=0.8)
        self.ax.text(length * 1.1, 0, 0, 'X', color='red', fontsize=10, fontweight='bold')

        # Y axis - Green
        self.ax.quiver(origin[0], origin[1], origin[2],
                      0, length, 0,
                      color='green', arrow_length_ratio=0.15, linewidth=2.5, alpha=0.8)
        self.ax.text(0, length * 1.1, 0, 'Y', color='green', fontsize=10, fontweight='bold')

        # Z axis - Blue
        self.ax.quiver(origin[0], origin[1], origin[2],
                      0, 0, length,
                      color='blue', arrow_length_ratio=0.15, linewidth=2.5, alpha=0.8)
        self.ax.text(0, 0, length * 1.1, 'Z', color='blue', fontsize=10, fontweight='bold')

    def draw_workspace_limits(self):
        """Draw workspace envelope as semi-transparent cylinder"""
        workspace = fk.compute_workspace_envelope()

        # Create cylinder mesh
        radius = workspace['radius']
        z_min = workspace['z_min']
        z_max = workspace['z_max']

        # Cylinder parameters
        theta = np.linspace(0, 2*np.pi, 30)
        z = np.linspace(z_min, z_max, 20)
        Theta, Z = np.meshgrid(theta, z)
        X = radius * np.cos(Theta)
        Y = radius * np.sin(Theta)

        # Plot surface
        self.ax.plot_surface(X, Y, Z,
                            alpha=0.08,
                            color='blue',
                            edgecolor='blue',
                            linewidth=0.5,
                            linestyle=':')

        # Add circle at top and bottom
        circle_theta = np.linspace(0, 2*np.pi, 100)
        circle_x = radius * np.cos(circle_theta)
        circle_y = radius * np.sin(circle_theta)

        self.ax.plot(circle_x, circle_y, z_max, 'b:', alpha=0.3, linewidth=1)
        self.ax.plot(circle_x, circle_y, z_min, 'b:', alpha=0.3, linewidth=1)

    def add_text_overlay(self, text, color='black'):
        """
        Add text overlay to the plot

        Args:
            text: Text to display
            color: Text color
        """
        self.ax.text2D(0.5, 0.95, text,
                      transform=self.ax.transAxes,
                      fontsize=10,
                      color=color,
                      ha='center',
                      va='top',
                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

    def _compute_fk_cached(self, q1, q2, q3, q4, q5, q6):
        """
        Compute forward kinematics with caching to avoid redundant calculations

        Args:
            q1-q6: Joint angles in degrees

        Returns:
            List of joint positions
        """
        # Create cache key (round to 0.1 degree precision)
        cache_key = (round(q1, 1), round(q2, 1), round(q3, 1),
                     round(q4, 1), round(q5, 1), round(q6, 1))

        with self._fk_cache_lock:
            if cache_key in self._fk_cache:
                return self._fk_cache[cache_key]

            # Compute FK if not cached
            positions = fk.compute_all_joint_positions(q1, q2, q3, q4, q5, q6)

            # Cache result (limit cache size to prevent memory bloat)
            if len(self._fk_cache) > 100:
                # Remove oldest entry
                self._fk_cache.pop(next(iter(self._fk_cache)))

            self._fk_cache[cache_key] = positions
            return positions

    def _has_data_changed(self, current_angles, trajectory_length):
        """
        Check if visualization data has actually changed

        Args:
            current_angles: Dict of current joint angles
            trajectory_length: Number of trajectory points

        Returns:
            True if data changed, False otherwise
        """
        if self._last_joint_angles is None:
            return True

        # Check if joint angles changed significantly (>0.5 degrees)
        for key in ['art1', 'art2', 'art3', 'art4', 'art5', 'art6']:
            old_val = self._last_joint_angles.get(key, 0)
            new_val = current_angles.get(key, 0)
            if abs(new_val - old_val) > 0.5:
                return True

        # Check if trajectory changed
        if trajectory_length != self._last_trajectory_length:
            return True

        return False

    def update_visualization(self, position_history, window_size=60, options=None):
        """
        Update the 3D visualization with current robot state and trajectory
        OPTIMIZED: Only redraws if data actually changed

        Args:
            position_history: PositionHistory object
            window_size: Time window in seconds
            options: Dictionary of display options
        """
        # Prevent concurrent updates (thread-safe)
        if not self._render_lock.acquire(blocking=False):
            logger.debug("Skipping update - render in progress")
            return

        try:
            if options is None:
                options = {}

            # Update display options
            self.show_robot = options.get('show_robot', True)
            self.show_trajectory = options.get('show_trajectory', True)
            self.show_base_frame = options.get('show_base_frame', True)
            self.show_workspace = options.get('show_workspace', False)
            self.show_grid = options.get('show_grid', True)
            self.show_labels = options.get('show_labels', False)
            self.auto_rotate = options.get('auto_rotate', False)

            # If no data, show home position (only once)
            if len(position_history) == 0:
                if self._is_dirty:
                    self.show_home_position()
                    self._is_dirty = False
                return

            # Get current joint angles (most recent)
            current_angles = position_history.get_current_joint_angles()
            if current_angles is None:
                if self._is_dirty:
                    self.show_home_position()
                    self._is_dirty = False
                return

            # OPTIMIZATION: Check if data actually changed
            tcp_trajectory = position_history.get_tcp_trajectory(window_size) if self.show_trajectory else []
            if not self._has_data_changed(current_angles, len(tcp_trajectory)) and not self.auto_rotate:
                logger.debug("No data change - skipping redraw")
                return

            # Data changed - update cache
            self._last_joint_angles = current_angles.copy()
            self._last_trajectory_length = len(tcp_trajectory)

            # Clear and setup axes (only when necessary)
            self.setup_3d_axes()

            # OPTIMIZATION: Use cached FK computation
            current_positions = self._compute_fk_cached(
                current_angles.get('art1', 0),
                current_angles.get('art2', 0),
                current_angles.get('art3', 0),
                current_angles.get('art4', 0),
                current_angles.get('art5', 0),
                current_angles.get('art6', 0)
            )
            self.current_joint_positions = current_positions

            # Draw robot arm if enabled
            if self.show_robot:
                self.draw_robot_arm(current_positions, active=True)

            # Draw TCP trajectory if enabled
            if self.show_trajectory and len(tcp_trajectory) > 1:
                tcp_points = [(x, y, z) for x, y, z, _ in tcp_trajectory]
                timestamps = [t for _, _, _, t in tcp_trajectory]
                self.draw_tcp_trajectory(tcp_points, timestamps)

            # Draw base frame if enabled
            if self.show_base_frame:
                self.draw_base_frame()

            # Draw workspace limits if enabled
            if self.show_workspace:
                self.draw_workspace_limits()

            # Auto-rotate if enabled
            if self.auto_rotate:
                self.rotation_angle += 0.5
                if self.rotation_angle >= 360:
                    self.rotation_angle = 0
                self.ax.view_init(elev=30, azim=self.rotation_angle)

            # Add legend (small, in corner)
            self.ax.legend(loc='upper left', fontsize=8, framealpha=0.8)

            # Redraw
            self.draw()

            logger.debug("3D visualization updated (data changed)")

        finally:
            self._render_lock.release()

    def reset_view(self):
        """Reset view to default isometric angle"""
        self.rotation_angle = 45
        self.ax.view_init(elev=30, azim=45)
        self.draw()
        logger.info("View reset to isometric")


# Example usage and testing
if __name__ == '__main__':
    import sys
    from PyQt5 import QtWidgets

    logging.basicConfig(level=logging.DEBUG)

    print("Thor Robot 3D Visualizer Test")
    print("=" * 50)

    app = QtWidgets.QApplication(sys.argv)

    # Create main window
    window = QtWidgets.QMainWindow()
    window.setWindowTitle("Thor Robot 3D Visualizer Test")
    window.setGeometry(100, 100, 900, 700)

    # Create central widget
    central_widget = QtWidgets.QWidget()
    window.setCentralWidget(central_widget)

    # Create layout
    layout = QtWidgets.QVBoxLayout(central_widget)

    # Create 3D canvas
    canvas = Robot3DCanvas(central_widget, width=8, height=6)
    layout.addWidget(canvas)

    # Show window
    window.show()

    print("\nShowing home position by default")
    print("Close window to exit")

    sys.exit(app.exec_())
