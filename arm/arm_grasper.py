import time
from SpotStack.arm.arm_core import ArmCore
from SpotStack.image.image_fetcher import ImageFetcher

from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.api import geometry_pb2, manipulation_api_pb2
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.math_helpers import Quat

class ArmGrasper(ArmCore):
    """
    A high-level arm control module that enables object grasping using Spot's hand camera and built-in manipulation API.
    Other camera sources may also be used if configured accordingly.
    """

    def __init__(self, robot, resolution='1920x1080'):
        """
        Initialize the ArmGrasper.

        Parameters
        ----------
        robot : bosdyn.client.robot.Robot
            The Spot robot instance.
        resolution : str, optional
            The resolution for the gripper camera (default is '1920x1080').
        """
        super().__init__(robot, resolution)
        self._manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)

    def _set_constraint(self, grasp_config, constraint):
        """
        Set the grasp orientation constraint in the grasp config.

        Parameters
        ----------
        grasp_config : bosdyn.api.manipulation_api_pb2.PickObjectInImage
            The grasp configuration proto to be updated with constraints.
        constraint : str or tuple or list
            The constraint to apply. Supported values:
            - 'squeeze': use built-in squeeze grasp.
            - 'top_down': align gripper x-axis with -Z in vision frame.
            - 'horizontal': align gripper y-axis with +Z in vision frame.
            - ('angle', pitch): specify a rotation constraint using a pitch angle in radians.

        Raises
        ------
        ValueError
            If the constraint name is invalid or unsupported.
        TypeError
            If the constraint type is not a string or a list/tuple.
        """
        grasp_config.grasp_params.grasp_params_frame_name = VISION_FRAME_NAME

        if isinstance(constraint, str):
            if constraint == 'squeeze':
                constraint_proto = grasp_config.grasp_params.allowable_orientation.add()
                constraint_proto.squeeze_grasp.SetInParent()
                return
            
            elif constraint == 'top_down':
                # The axis on the gripper is the x-axis.
                axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

                # The axis in the vision frame is the negative z-axis
                axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)
            
            elif constraint == 'horizontal':
                # The axis on the gripper is the y-axis.
                axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)

                # The axis in the vision frame is the positive z-axis
                axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=1)

            else:
                raise ValueError(f"ArmGrasper: Invalid constraint name: '{constraint}'.")
            
            # Add the vector constraint to our proto.
            constraint_proto = grasp_config.grasp_params.allowable_orientation.add()
            constraint_proto.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
                axis_on_gripper_ewrt_gripper)
            constraint_proto.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
                axis_to_align_with_ewrt_vo)

            # We'll take anything within about 10 degrees for top-down or horizontal grasps.
            constraint_proto.vector_alignment_with_tolerance.threshold_radians = 0.17

        elif isinstance(constraint, (tuple, list)):

            if constraint[0] == 'angle':
                # First, get the robot's position in the world.
                robot_state = self._state_client.get_robot_state()
                vision_T_body = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

                # Rotation from the body to our desired grasp.
                body_Q_grasp = Quat.from_pitch(constraint[1])  # 45 degrees
                vision_Q_grasp = vision_T_body.rotation * body_Q_grasp

                # Turn into a proto
                constraint_proto = grasp_config.grasp_params.allowable_orientation.add()
                constraint_proto.rotation_with_tolerance.rotation_ewrt_frame.CopyFrom(vision_Q_grasp.to_proto())

                # We'll accept anything within +/- 10 degrees
                constraint_proto.rotation_with_tolerance.threshold_radians = 0.17

            else:
                raise ValueError(f"ArmGrasper: Invalid constraint name: '{constraint}'.")

        else:
            raise TypeError("ArmGrasper: constraint must be a string or tuple/list")

    def grasp(self, object_pixel, image_response, constraint):
        """
        Execute a grasp command targeting a specific pixel corresponding to the target object in the camera image.

        Parameters
        ----------
        object_pixel : tuple or list
            The (x, y) pixel coordinates in the image that correspond to the target object.
            This point should lie within the visible region of the object to be grasped.
        image_response : bosdyn.api.image_pb2.ImageResponse
            The full image response from the camera, containing image metadata and transform snapshot.
        constraint : str or tuple or list
            Orientation constraint for the grasp. Can be:
                - 'squeeze': Enable squeeze grasp mode.
                - 'top_down': Align gripper x-axis with -Z in the vision frame.
                - 'horizontal': Align gripper y-axis with +Z in the vision frame.
                - ('angle', pitch): Specify a pitch angle (in radians) relative to the robot's body frame.

        Returns
        -------
        bool
            True if the grasp succeeds, False if it fails.
        """
        object_pixel = geometry_pb2.Vec2(x=object_pixel[0], y=object_pixel[1])
        
        grasp_config = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=object_pixel, transforms_snapshot_for_camera=image_response.shot.transforms_snapshot,
            frame_name_image_sensor=image_response.shot.frame_name_image_sensor,
            camera_model=image_response.source.pinhole)
        
        self._set_constraint(grasp_config, constraint)
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp_config)
        cmd_response = self._manipulation_api_client.manipulation_api_command(manipulation_api_request=grasp_request)

        # Wait until command finished
        while True:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(manipulation_cmd_id=cmd_response.manipulation_cmd_id)
            response = self._manipulation_api_client.manipulation_api_feedback_command(manipulation_api_feedback_request=feedback_request)

            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
                return True
            
            elif response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                print('ArmGrasper: Fail to grasp')
                return False

            time.sleep(0.25)

    def get_image_and_response(self, data_transform=None):
        """
        Returns a single image from the hand color camera along with the image response.

        Parameters
        ----------
        data_transform : Callable, optional
            Optional transform (e.g., torchvision transform) applied to the PIL image.

        Returns
        -------
        PIL.Image or transformed object
            Decoded and optionally transformed image.
        bosdyn.api.image_pb2.ImageResponse
            The full image response from the robot camera.
        """
        image_response = self._image_client.get_image(self._image_request)
        _, image = ImageFetcher._decrypt_image(image_response[0], auto_rotate=False, data_transform=data_transform)

        return image, image_response[0]

# Example Usage
if __name__ == '__main__':

    import cv2, numpy
    KEYBOARD_LOOKUP = {
        'w': (0.5, 0.0, 0.0),   # forward in x
        's': (-0.5, 0.0, 0.0),  # backward in x
        'a': (0.0, 0.5, 0.0),   # left in y
        'd': (0.0, -0.5, 0.0),  # right in y
        'q': (0.0, 0.0, 0.5),   # rotate left (yaw)
        'e': (0.0, 0.0, -0.5)   # rotate right (yaw)
    }
    def pil_to_cv(image_pil):
        image_np = numpy.array(image_pil)
        return cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)

    def cv_mouse_callback(event, x, y, flags, param):
        clone = param['image'].copy()
        if event == cv2.EVENT_LBUTTONUP:
            param['click'] = (x, y)
        else:
            # Draw auxiliary lines on the image.
            color = (30, 30, 30)
            thickness = 2
            image_title = 'Click to grasp'
            height = clone.shape[0]
            width = clone.shape[1]
            cv2.line(clone, (0, y), (width, y), color, thickness)
            cv2.line(clone, (x, 0), (x, height), color, thickness)
            cv2.imshow(image_title, clone)

    import argparse, bosdyn.client.util, sys
    from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
    from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
    from bosdyn.client.math_helpers import SE3Pose
    from SpotStack.motion.motion_controller import MotionController
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('ArmGrasper')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    try:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            try:
                arm_grasper = ArmGrasper(robot)
                motion_controller = MotionController(robot)
                blocking_stand(command_client)

                flat_body_Q_hand = Quat.from_pitch(1.2)
                flat_body_T_hand = SE3Pose(x=0.6, y=0.0, z=0.45, rot=flat_body_Q_hand)

                # Pre grasp
                arm_grasper.move_to_pose(flat_body_T_hand, 1)
                time.sleep(2)
                
                image, image_response = arm_grasper.get_image_and_response(data_transform=pil_to_cv)

                # Click to grasp
                image_title = 'Click to grasp'
                param = {'image': image, 'click': None}
                cv2.namedWindow(image_title)
                cv2.setMouseCallback(image_title, cv_mouse_callback, param)
                cv2.imshow(image_title, image)

                while param['click'] is None:
                    image, image_response = arm_grasper.get_image_and_response(data_transform=pil_to_cv)
                    param['image'] = image
                    cv2.imshow(image_title, image)
                    key = cv2.waitKey(1) & 0xFF
                    
                    if key == ord('x'):
                        motion_controller.send_velocity_command(0, 0, 0, 0.6)
                        print('"x" pressed, exiting.')
                        exit(0)
                    
                    try:
                        action = KEYBOARD_LOOKUP.get(chr(key), (0, 0, 0))
                        motion_controller.send_velocity_command(action[0], action[1], action[2], 0.6)

                    except ValueError:
                        continue
                        
                cv2.destroyWindow(image_title)

                # Grasping
                if not arm_grasper.grasp(param['click'], image_response, 'horizontal'):
                    raise RuntimeError("Grasp failed")

                # Carrying
                arm_grasper.move_to_pose(flat_body_T_hand, 0)
                time.sleep(5)

                # Releasing
                arm_grasper.move_to_pose(flat_body_T_hand, 1)
                time.sleep(1)

                arm_grasper.rest_arm()
                time.sleep(1)

            except Exception as exc:  # pylint: disable=broad-except
                print("ArmGrasper threw an error.")
                print(exc)

    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )