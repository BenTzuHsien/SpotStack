from SpotStack.power.power_manager import PowerManager
from SpotStack.image.image_fetcher import ImageFetcher

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, block_until_arm_arrives
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.gripper_camera_param import GripperCameraParamClient
from bosdyn.api import header_pb2
from bosdyn.api.gripper_camera_param_pb2 import GripperCameraParams, GripperCameraParamRequest

class ArmCore:
    """
    A controller class for manipulating the Spot robot's arm and capturing hand camera images.

    This class abstracts arm motion commands, gripper control, and camera image acquisition, simplifying coordinated arm and perception tasks.

    Attributes
    ----------
    sources : str
        The image source identifier for the hand camera.
    """
    source = 'hand_color_image'

    def __init__(self, robot, resolution='1920x1080'):
        """
        Initialize the ArmCore.

        Parameters
        ----------
        robot : bosdyn.client.robot.Robot
            The Spot robot instance.
        resolution : str, optional
            The resolution for the gripper camera (default is '1920x1080').
        """
        self._state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        # Gripper Camera Setup
        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._image_request = [build_image_request(self.source, quality_percent=100, pixel_format='PIXEL_FORMAT_RGB_U8')]
        self._gripper_camera_param_client = robot.ensure_client(GripperCameraParamClient.default_service_name)

        # Setup the gripper camera resolution
        params = {}
        if resolution == '640x480':
            params['camera_mode'] = GripperCameraParams.MODE_640_480_120FPS_UYVY
        elif resolution == '1280x720':
            params['camera_mode'] = GripperCameraParams.MODE_1280_720_60FPS_UYVY
        elif resolution == '1920x1080':
            params['camera_mode'] = GripperCameraParams.MODE_1920_1080_60FPS_MJPG
        elif resolution == '3840x2160':
            params['camera_mode'] = GripperCameraParams.MODE_3840_2160_30FPS_MJPG
        elif resolution == '4096x2160':
            params['camera_mode'] = GripperCameraParams.MODE_4096_2160_30FPS_MJPG
        elif resolution == '4208x3120':
            params['camera_mode'] = GripperCameraParams.MODE_4208_3120_20FPS_MJPG
        self.set_camera_param(params)

        self._power_manager = PowerManager(robot)
        self._power_manager.toggle_power(should_power_on=True)

    def on_quit(self):
        """
        Cleanup method to run when terminating the program.

        If the robot was powered on by this instance (not externally), it will be powered off safely.
        """
        # Sit the robot down + power off after the navigation command is complete.
        if self._power_manager.check_is_powered_on():
            self._power_manager.toggle_power(should_power_on=False)

    # Motion Control
    def rest_arm(self):
        """
        Reset the arm and closes the gripper.
        This moves the arm to the default rest position, which is typically safe for navigation.
        """
        reset_command = RobotCommandBuilder.build_synchro_command(RobotCommandBuilder.arm_stow_command(), RobotCommandBuilder.claw_gripper_close_command())

        cmd_id = self._command_client.robot_command(reset_command)
        block_until_arm_arrives(self._command_client, cmd_id)

    def freeze_arm(self):
        """
        Commands the arm to hold its current pose.
        This method enables impedance-based control to maintain the arm's joint position in space even if the robot base moves.
        """
        arm_joint_freeze_command = RobotCommandBuilder.arm_joint_freeze_command()
        cmd_id = self._command_client.robot_command(arm_joint_freeze_command)
        block_until_arm_arrives(self._command_client, cmd_id)

    def move_to_pose(self, pose, gripper_open_fraction=0.0, use_world_frame=False):
        """
        Moves the robot's arm to a specified 6-DoF pose. 
        When using the world frame as reference, the arm maintains its position relative to the world, regardless of base movement.

        Parameters
        ----------
        pose : bosdyn.client.math_helpers.SE3Pose
            The desired end-effector pose relative to body frame.
        gripper_open_fraction : float
            Fraction [0.0, 1.0] to open the gripper. closed=0.0, fully open=1.0. (default is closed)
        use_world_frame : bool, optional
            If True, interprets the target pose in the world frame (Odom). Otherwise, interprets it in the robot's body frame. Default is False.
        """
        # Arm command
        if use_world_frame:
            # Get current transformation
            current_state = self._state_client.get_robot_state()
            odom_T_flat_body = get_a_tform_b(current_state.kinematic_state.transforms_snapshot, ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

            odom_T_hand = odom_T_flat_body * pose
            arm_command = RobotCommandBuilder.arm_pose_command(
                odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
                odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds=2)
        else:
            arm_command = RobotCommandBuilder.arm_pose_command(pose.x, pose.y, pose.z, pose.rot.w, pose.rot.x,pose.rot.y, pose.rot.z, GRAV_ALIGNED_BODY_FRAME_NAME, seconds=2)
        
        # Gripper command
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(gripper_open_fraction)

        arm_gripper_command = RobotCommandBuilder.build_synchro_command(arm_command, gripper_command)
        cmd_id = self._command_client.robot_command(command=arm_gripper_command)
        block_until_arm_arrives(self._command_client, cmd_id)

    # Camera
    def set_camera_param(self, param_dict):
        """
        Sets parameters for the gripper (hand) camera.
        
        For the full list of configurable parameters, refer to:
        https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html#grippercameraparams

        Parameters
        ----------
        param_dict : dict
            A dictionary mapping field names in `GripperCameraParams` to their desired values.
            Only valid fields will be set. Invalid field names will be ignored with a warning.
        """
        params = GripperCameraParams()
        valid_fields = {f.name for f in GripperCameraParams.DESCRIPTOR.fields}

        for field_name, value in param_dict.items():
            if field_name not in valid_fields:
                print(f'{field_name} is not a valid field in GripperCameraParams')
                continue
            try:
                setattr(params, field_name, value)
            except (TypeError, ValueError) as e:
                print(f"Invalid value for {field_name}: {value} ({e})")

        request = GripperCameraParamRequest(params=params)
        response = self._gripper_camera_param_client.set_camera_params(request)

        if response.header.error and response.header.error.code != header_pb2.CommonError.CODE_OK:
            print(f'Setting gripper camera params get an error: {response.header.error}')
    
    def get_image(self, data_transform=None):
        """
        Fetches a single image from the hand color camera.

        Parameters
        ----------
        data_transform : Callable, optional
            Optional transform (e.g., torchvision transform) applied to the PIL image.

        Returns
        -------
        PIL.Image or transformed object
            Decoded and optionally transformed image.
        """
        image_response = self._image_client.get_image(self._image_request)
        _, image = ImageFetcher._decrypt_image(image_response[0], auto_rotate=False, data_transform=data_transform)

        return image
    
# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, sys, os, time
    from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
    from bosdyn.client.math_helpers import Quat, SE3Pose

    from SpotStack.motion.motion_controller import MotionController
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('ArmCore')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    try:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            try:
                arm_core = ArmCore(robot, resolution='4208x3120')
                motion_controller = MotionController(robot)

                flat_body_Q_hand = Quat.from_pitch(1.2)
                flat_body_T_hand = SE3Pose(x=0.6, y=0.0, z=0.45, rot=flat_body_Q_hand)

                arm_core.move_to_pose(flat_body_T_hand, 1)
                time.sleep(2)
                
                arm_image = arm_core.get_image()

                image_save_path = os.path.join(os.path.dirname(__file__), 'arm_image.jpg')
                arm_image.save(image_save_path)

                arm_core.freeze_arm()
                motion_controller.send_displacement_command(1, 0, 0,)
                time.sleep(3)

                arm_core.rest_arm()
                time.sleep(1)

            except Exception as exc:  # pylint: disable=broad-except
                print("ArmCore threw an error.")
                print(exc)
            finally:
                arm_core.on_quit()

    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )