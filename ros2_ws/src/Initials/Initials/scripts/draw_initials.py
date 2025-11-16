#!/usr/bin/env python3
"""
Midterm Project
Muhammad Ahmed Nazir Shaikh

This node makes the Kinova Gen3 Lite write the initials "MAN" on a virtual board.
- Letter strokes are defined in 2D and mapped to 3D poses in base_link.
- The pen faces the board with a fixed orientation.
- Waypoints are executed with small Cartesian steps; joint-space fallback improves robustness.
- RViz LINE_STRIP markers visualize the path in real time for debugging.
"""

import time
import threading
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
from pymoveit2 import MoveIt2

# Sentinel indicating a stroke break (lift pen / "pen up")
PEN_UP = None

# ----------------------------- Letter stroke data (2D) -------------------------
# Each tuple is (px, py) in meters in a small "letter" coordinate frame
LETTER_M = [
    (0.00, 0.20), (0.00, 0.00),
    (0.05, 0.10),
    (0.10, 0.00),
    (0.10, 0.20)
]

LETTER_A = [
    (0.15, 0.20), (0.20, 0.00),
    (0.25, 0.20),
    PEN_UP,
    (0.175, 0.12), (0.225, 0.12)
]

LETTER_N = [
    (0.30, 0.20), (0.30, 0.00),
    (0.40, 0.20),
    (0.40, 0.00)
]

# Concatenate the full path: M, (pen up), A, (pen up), N
ALL_LETTERS_PATH = LETTER_M + [PEN_UP] + LETTER_A + [PEN_UP] + LETTER_N

def make_pose(x, y, z, orientation) -> Pose:
    """Build and return a geometry_msgs/Pose at (x, y, z) with quaternion 'orientation'."""
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = orientation
    return p

class WritingNode(Node):
    """ROS 2 node that:
    - Plans/executes motions via MoveIt2 to trace the letters "MAN".
    - Publishes RViz LINE_STRIP markers so the stroke path is visible during execution.
    - Uses a small "tap-in" at the very first point to establish contact cleanly.
    """

    def __init__(self):
        super().__init__("writing_node")

        # MoveIt2 interface: arm group, base and tool frames
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3",
                         "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        # RViz publisher for stroke visualization
        self.marker_publisher = self.create_publisher(Marker, "/visualization_marker", 10)
        self.marker_id_counter = 0
        self.line_marker = self.create_line_strip_marker()

        # Fixed tool orientation (pen points toward the board / +X)
        self.write_orientation = (0.7071, 0.0, 0.7071, 0.0)

        # Named joint presets
        self.j_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.j_write = [0.0, 0.4363, 2.007, -1.5708, 0.2094, 1.5708]

    def create_line_strip_marker(self):
        """Create a green LINE_STRIP marker (in base_link) for RViz stroke visualization."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "drawing_path"
        marker.id = self.marker_id_counter
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005     # line width in meters
        marker.color.g = 1.0       # green
        marker.color.a = 1.0       # fully opaque
        return marker

    def move_to_joints(self, joint_positions) -> bool:
        """Plan → execute a joint-space move and block until completion.
        Returns True on success, False otherwise.
        """
        try:
            self.moveit2.move_to_configuration(joint_positions=joint_positions)
            self.moveit2.wait_until_executed()     # Synchronize to avoid segment overlap on hardware
            time.sleep(0.03)                       # Tiny settle time
            return True
        except Exception as e:
            self.get_logger().warn(f"move_to_joints failed: {e}")
            return False

    def move_to_pose(self, pose: Pose, cartesian: bool = False,
                     max_step: float = 0.005, frac_thresh: float = 0.30) -> bool:
        """Plan - execute to end-effector 'pose'.
        - If cartesian=True: plan small Cartesian segments (max_step) with a cartesian
          path completeness threshold (frac_thresh).
        - If cartesian=False: standard (joint-space) planning.
        Always blocks until executed. Returns True on success, False otherwise.
        """
        try:
            if cartesian:
                # Cartesian plan with user-tuned step size and fraction threshold
                traj = self.moveit2.plan(
                    pose=pose,
                    cartesian=True,
                    max_step=max_step,
                    cartesian_fraction_threshold=frac_thresh,
                )
            else:
                # Joint-space plan as a robust fallback
                traj = self.moveit2.plan(pose=pose, cartesian=False)

            if traj is None:
                self.get_logger().warn("Planning failed: traj=None")
                return False

            # Execute and block for deterministic behavior
            self.moveit2.execute(traj)
            self.moveit2.wait_until_executed()
            time.sleep(0.02)  # small settle
            return True

        except Exception as e:
            self.get_logger().warn(f"move_to_pose exception: {e}")
            return False

    def task_write_initials(self):
        """Main routine:
        1) Move to HOME → WRITING STANCE,
        2) Iterate MAN waypoints:
           - At PEN_UP: publish the current stroke, start a new strip, and do a 2 cm retract along -X,
           - Else: map 2D point to the writing plane and execute (Cartesian first, then joint fallback),
           - Special case (first point of M): do a 1 cm "tap-in" pre-contact.
        3) Final retract and return to HOME.
        """
        self.get_logger().info("--- Starting MAN Writing Task ---")

        # Give MoveIt services time to come up (practical startup guard)
        time.sleep(3.0)

        # Move to safe HOME posture
        self.get_logger().info("-- Moving to HOME position.")
        if not self.move_to_joints(self.j_home):
            self.get_logger().error("Failed to reach HOME. Aborting.")
            return

        # Move to WRITING STANCE to set comfortable wrist/elbow geometry
        self.get_logger().info("-- Moving to WRITING STANCE.")
        if not self.move_to_joints(self.j_write):
            self.get_logger().error("Failed to reach WRITING STANCE. Aborting.")
            return

        # ---------------- Mapping parameters (frame = base_link) ----------------
        drawing_depth_x = 0.33   # Fixed X depth (board plane)
        start_y = 0.0            # Y offset for mapping px
        start_z = 0.4            # Z offset for mapping py
        scale = 0.5              # Scale factor applied to the 2D letter coordinates

        self.get_logger().info("-- Starting the drawing loop...")

        # Book-keeping for special behaviors (first point of first letter)
        letter_index = 0
        stroke_index_within_letter = 0
        approach_dx = 0.01       # 1 cm pre-contact offset along -X for the initial "tap-in"

        # --------------------------- Draw loop begins ---------------------------
        for point in ALL_LETTERS_PATH:
            # Conditional: stroke separator → finalize current strip and do a short retract
            if point is PEN_UP:
                # Publish the stroke drawn so far and start a new LINE_STRIP
                self.marker_publisher.publish(self.line_marker)
                self.marker_id_counter += 1
                self.line_marker = self.create_line_strip_marker()

                # Reset per-stroke counter; advance letter index for the next segment
                stroke_index_within_letter = 0
                letter_index += 1

                # Compute a small -X retract (pen up) to avoid smearing during transitions
                fk_result = self.moveit2.compute_fk()
                current_pose = fk_result.pose
                retract = deepcopy(current_pose)
                retract.position.x -= 0.02  # 2 cm lift-off
                _ = self.move_to_pose(retract, cartesian=True)
                continue  # Move on to the next waypoint/letter

            # Map 2D (px, py) → world (y, z); x is fixed to the board depth
            target_y = start_y - (point[0] * scale)
            target_z = start_z - (point[1] * scale)
            target_pose = make_pose(drawing_depth_x, target_y, target_z, self.write_orientation)

            # Conditional: only at the first stroke point of the very first letter (M)
            if letter_index == 0 and stroke_index_within_letter == 0:
                # Step 1: approach 1 cm before the board in joint space to avoid scraping
                pre_contact = make_pose(drawing_depth_x - approach_dx, target_y, target_z, self.write_orientation)
                if not self.move_to_pose(pre_contact, cartesian=False):
                    self.get_logger().warn("M approach (joint-space) failed.")

                # Step 2: Cartesian "tap-in" onto the board with tighter parameters
                contact = make_pose(drawing_depth_x, target_y, target_z, self.write_orientation)
                if not self.move_to_pose(contact, cartesian=True, max_step=0.003, frac_thresh=0.10):
                    # Fallback: force joint-space if Cartesian contact cannot be planned fully
                    self.get_logger().warn("Tap-in failed; forcing joint-space.")
                    _ = self.move_to_pose(contact, cartesian=False)

            # Append the target to the live RViz path so we can see the stroke evolve
            self.line_marker.points.append(target_pose.position)
            self.marker_publisher.publish(self.line_marker)

            # Try Cartesian motion first (tiny steps for smooth surface tracing)
            ok = self.move_to_pose(target_pose, cartesian=True, max_step=0.003, frac_thresh=0.30)

            # Conditional fallback: if Cartesian planning under-achieves, try joint-space
            if not ok:
                self.get_logger().warn("Cartesian failed; falling back to joint-space.")
                ok = self.move_to_pose(target_pose, cartesian=False)

            # Conditional: as a last resort, skip this waypoint to keep execution robust
            if not ok:
                self.get_logger().warn(
                    f"Skipping point (y={target_y:.3f}, z={target_z:.3f}) due to planning failure."
                )

            # Increment per-stroke index for the next waypoint
            stroke_index_within_letter += 1

        # ---------------------------- Draw loop ends ----------------------------

        self.get_logger().info("-- Finished drawing! Returning home.")

        # Final small retract off the board for safety
        fk_result = self.moveit2.compute_fk()
        final_pose = fk_result.pose
        final_pose.position.x -= 0.02
        _ = self.move_to_pose(final_pose, cartesian=True)

        # Return to HOME posture
        _ = self.move_to_joints(self.j_home)

def main(args=None):
    """Spin the WritingNode with a multithreaded executor so planning/logging/markers
    remain responsive while the drawing task runs in a separate thread.
    """
    rclpy.init(args=args)
    node = WritingNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    # Run the main task in a thread so the executor can keep spinning callbacks
    thread = threading.Thread(target=node.task_write_initials)
    thread.start()

    executor.spin()
    thread.join()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

