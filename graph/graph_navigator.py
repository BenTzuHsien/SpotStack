import time
from SpotStack.power.power_manager import PowerManager
from SpotStack.graph.graph_base import GraphBase, graph_nav_util

from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.graph_nav import graph_nav_pb2
from bosdyn.client.exceptions import ResponseError

class GraphNavigator(GraphBase):
    """
    Class for navigating GraphNav maps with Spot.

    This class uploads a saved graph, initializes localization via fiducials,
    and provides navigation commands to move to waypoints in the map.
    """

    def __init__(self, robot, graph_path):
        """
        Initialize the GraphNavigator.

        Parameters
        ----------
        robot : bosdyn.client.robot.Robot
            The Spot robot instance.
        graph_path : str
            Path to load graph and snapshot files.
        """
        super().__init__(robot, graph_path)

        # Create robot clients
        command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        
        self._power_manager = PowerManager(robot)
        self._power_manager.toggle_power(should_power_on=True)

        # Setup
        blocking_stand(command_client)
        self.load_graph()

    def on_quit(self):
        """
        Cleanup method to run when terminating the program.

        If the robot was powered on by this instance (not externally), it will be powered off safely.
        """
        # Sit the robot down + power off after the navigation command is complete.
        if self._power_manager.check_is_powered_on():
            self._power_manager.toggle_power(should_power_on=False)
    
    # GraphNav Action Section
    def check_success(self, command_id=-1):
        """
        Use a navigation command id to get feedback from the robot.

        Parameters
        ----------
        command_id : int
            Command ID returned from GraphNav navigate_to. Defaults to -1.

        Returns
        -------
        bool
            True if the navigation goal is reached or failed definitively (e.g., stuck or lost).
        """
        if command_id == -1:
            # No command, so we have no status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print("Robot got lost when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print("Robot got stuck when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            print("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False
    
    def navigate_to(self, waypoint_name, relative_pose=None):
        """
        Navigate the robot to a specific waypoint.

        Parameters
        ----------
        waypoint_name : str
            The name for the target waypoint.
        relative_pose : bosdyn.api.geometry_pb2.SE2Pose, optional
            A pose offset relative to the destination waypoint.
        """
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            waypoint_name, self._current_graph, self._current_annotation_name_to_wp_id)
        if not destination_waypoint:
            # Failed to find the appropriate unique waypoint id for the navigation command.
            return
        if not self._power_manager.toggle_power(should_power_on=True):
            print("Failed to power on the robot, and cannot complete navigate to request.")
            return

        nav_to_cmd_id = None
        is_finished = False
        # Navigate to the destination waypoint.
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            try:
                nav_to_cmd_id = self._graph_nav_client.navigate_to(destination_waypoint, 1.0,
                                                                   command_id=nav_to_cmd_id,
                                                                   destination_waypoint_tform_body_goal=relative_pose)

            except ResponseError as e:
                print("Error while navigating {}".format(e))
                break
            time.sleep(.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self.check_success(nav_to_cmd_id)

# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, os, sys
    from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--graph-path',
                        help='Full filepath to graph and snapshots to be uploaded.',
                        default=os.getcwd())
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('GraphNavigator')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    try:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            try:
                graph_navigator = GraphNavigator(robot, options.graph_path)
                num_waypoints = graph_navigator.get_waypoint_count()
                print(f'Nuber of Waypoint: {num_waypoints}')

                while True:
                    input_str = input(f"Which waypoint to go to? To end, type q:")
                    while not input_str.isdigit() and input_str != 'q':
                        print('wrong input')
                        input_str = input(f"Which waypoint to go to? To end, type q:")
                    
                    if input_str == 'q':
                        break
                
                    print(f'Going to Waypoint_{input_str}')
                    graph_navigator.navigate_to(f'Waypoint_{input_str}')

                # Showing how to use relative pose
                input_str = input(f"Try Relative Pose?(y):")
                if input_str == 'y':
                    from bosdyn.api.geometry_pb2 import SE2Pose, Vec2
                    relative_pose = SE2Pose(position=Vec2(x=1, y=1), angle=0.5)
                    graph_navigator.navigate_to(f'Waypoint_0', relative_pose)

                graph_navigator.on_quit()

            except Exception as exc:  # pylint: disable=broad-except
                print("GraphNavigator threw an error.")
                print(exc)

    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )