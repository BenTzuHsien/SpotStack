import time
from SpotStack.power.power_manager import PowerManager

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.lease import Error as LeaseBaseError

class MotionController:
    """
    A controller class for commanding Spot's body motion using velocity and displacement inputs.

    This class initializes required clients, manages robot power state, and provides utility
    methods to send velocity-based or displacement-based movement commands.

    Coordinate Frame Convention:
    - +X: Forward (robot head direction)
    - +Y: Left (from robot's perspective)
    - +Z: Upward
    """

    def __init__(self, robot):
        """
        Initialize the MotionController class.

        Parameters
        ----------
        robot : bosdyn.client.robot.Robot
            The Spot robot instance.
        """
        self._state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        self._power_manager = PowerManager(robot)
        self._power_manager.toggle_power(should_power_on=True)

        # Setup
        blocking_stand(self._command_client)

    def on_quit(self):
        """
        Cleanup method to run when terminating the program.

        If the robot was powered on by this instance (not externally), it will be powered off safely.
        """
        # Sit the robot down + power off after the navigation command is complete.
        if self._power_manager.check_is_powered_on():
            self._power_manager.toggle_power(should_power_on=False)

    def send_velocity_command(self, v_x, v_y, v_yaw, duration):
        """
        Send a velocity command to the robot in the body frame for a specified duration.
        
        You should add a time.sleep() call after this function to allow execution to complete.

        Parameters
        ----------
        v_x : float
            Linear velocity in the X direction (forward, positive toward robot head), in meters per second.
        v_y : float
            Linear velocity in the Y direction (left), in meters per second.
        v_yaw : float
            Angular velocity around the Z-axis (positive is counter-clockwise), in radians per second.
        duration : float
            Duration in seconds to apply the velocity command.
        """
        velocity_command = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_yaw)

        try:
            self._command_client.robot_command(command=velocity_command, end_time_secs=time.time() + duration)
        
        except (ResponseError, RpcError, LeaseBaseError) as err:
            print(f"Velocity Command Failed: {err}")

    def send_displacement_command(self, d_x, d_y, d_yaw):
        """
        Send a displacement command to move the robot relative to its current body pose, using the odom frame as reference.

        The robot will move approximately d_x meters forward, d_y meters left, and rotate d_yaw radians
        counter-clockwise. The command is transformed into an absolute goal in the odom frame and sent 
        as a trajectory point.

        Displacement must be at least ~0.2 meters to trigger meaningful motion, and results may vary slightly 
        due to locomotion noise. You should follow this command with time.sleep() to allow execution.

        Parameters
        ----------
        d_x : float
            Forward displacement in meters (positive toward robot head).
        d_y : float
            Leftward displacement in meters.
        d_yaw : float
            Rotation in radians (positive is counter-clockwise).
        """
        current_state = self._state_client.get_robot_state()
        current_pose = get_odom_tform_body(current_state.kinematic_state.transforms_snapshot)   #T_odom_current: current body frame in odom frame
        displacement = SE3Pose(d_x, d_y, 0, Quat.from_yaw(d_yaw))   #T_current_next: transformation from next to current(displacement)

        goal_pose = current_pose.mult(displacement)   #T_odom_next: next body frame in odom frame

        displacement_command = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_pose.x, goal_pose.y, goal_pose.rotation.to_yaw(), frame_name='odom')

        try:
            self._command_client.robot_command(command=displacement_command, end_time_secs=time.time() + 5)
        
        except (ResponseError, RpcError, LeaseBaseError) as err:
            print(f"Displacement Command Failed: {err}")

# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, sys
    from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('MotionController')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    try:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            try:
                motion_controller = MotionController(robot)
                
                # Velocity
                motion_controller.send_velocity_command(1, 0, 0, duration=1)
                time.sleep(2)

                motion_controller.send_velocity_command(0, 0, 0, duration=5)
                time.sleep(2)

                # Displacement
                motion_controller.send_displacement_command(0.2, 0, 0)
                time.sleep(2)

                motion_controller.on_quit()

            except Exception as exc:  # pylint: disable=broad-except
                print("MotionController threw an error.")
                print(exc)

    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )