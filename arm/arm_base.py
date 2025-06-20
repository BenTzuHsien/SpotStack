from SpotStack.power.power_manager import PowerManager
from SpotStack.image.image_fetcher import ImageFetcher

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, block_until_arm_arrives
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.image import ImageClient, build_image_request

class ArmBase:
    source = 'hand_color_image'

    def __init__(self, robot, pixel_format='PIXEL_FORMAT_RGB_U8'):
        self._state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._image_request = [build_image_request(self.source, quality_percent=100, pixel_format=pixel_format)]

        self._power_manager = PowerManager(robot)
        self._power_manager.toggle_power(should_power_on=True)

    def rest_arm(self):

        reset_command = RobotCommandBuilder.arm_stow_command()

        cmd_id = command_client.robot_command(reset_command)
        block_until_arm_arrives(self._command_client, cmd_id)

    def move_to_pose(self, pose, gripper_open_fraction=0.0):

        # Get current transformation
        current_state = self._state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(current_state.kinematic_state.transforms_snapshot, ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        # Arm command
        odom_T_hand = odom_T_flat_body * SE3Pose.from_obj(pose)
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
            odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds=2)

        # Gripper command
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(gripper_open_fraction)

        arm_gripper_command = RobotCommandBuilder.build_synchro_command(arm_command, gripper_command)
        cmd_id = self._command_client.robot_command(command=arm_gripper_command)
        block_until_arm_arrives(self._command_client, cmd_id)

    def get_image(self, data_transform=None):

        image_response = self._image_client.get_image(self._image_request)
        _, image = ImageFetcher._decrypt_image(image_response[0], auto_rotate=False, data_transform=data_transform)

        return image
    
# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, sys, os, time
    from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
    from bosdyn.client.robot_command import blocking_stand
    from bosdyn.api import geometry_pb2
    from bosdyn.client.math_helpers import Quat
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('ArmBase')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    try:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            try:
                arm_base = ArmBase(robot)
                blocking_stand(command_client)

                hand_ewrt_flat_body = geometry_pb2.Vec3(x=0.6, y=0.0, z=0.65)
                flat_body_Q_hand = Quat.from_pitch(0.5).to_proto()
                flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)

                arm_base.move_to_pose(flat_body_T_hand, 1)
                time.sleep(1)
                
                arm_image = arm_base.get_image()
                time.sleep(1)

                image_save_path = os.path.join(os.path.dirname(__file__), 'arm_image.jpg')
                arm_image.save(image_save_path)

                arm_base.rest_arm()

            except Exception as exc:  # pylint: disable=broad-except
                print("ArmBase threw an error.")
                print(exc)

    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )