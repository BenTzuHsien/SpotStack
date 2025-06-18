from SpotStack.graph.utils import graph_nav_util

from bosdyn.api.graph_nav import map_pb2
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.graph_nav import GraphNavClient, nav_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.math_helpers import Quat, SE3Pose

class GraphBase:
    """
    Base class for managing Spot's navigation graph.

    This class provides shared functionality for uploading, clearing, and interacting
    with the GraphNav map and its associated waypoint and edge snapshots.
    """

    def __init__(self, robot, graph_path):
        """
        Initialize the GraphBase class.

        Parameters
        ----------
        robot : bosdyn.client.robot.Robot
            The Spot robot instance.
        graph_path : str
            Path to the local directory where the graph and snapshots are stored.
        """
        self._robot = robot
        self._graph_path = graph_path

        # Force trigger timesync
        self._robot.time_sync.wait_for_sync()

        # Create robot clients
        self._state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)

        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()

        # Setup
        self._clear_graph()

    def _clear_graph(self):
        """Clear the state of the map on the robot, removing all waypoints and edges."""  
        return self._graph_nav_client.clear_graph()
    
    def _set_initial_localization_fiducial(self):
        """Trigger localization when near a fiducial."""
        robot_state = self._state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body)
        print("Localization based on fiducials completed!")

    def _upload_graph_and_snapshots(self):
        """Upload the graph and snapshots to the robot."""
        print("Loading the graph from disk into local storage...")
        with open(self._graph_path + "/graph", "rb") as graph_file:
            
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            print("Loaded graph has {} waypoints and {} edges".format(
                len(self._current_graph.waypoints), len(self._current_graph.edges)))
        for waypoint in self._current_graph.waypoints:
            
            # Load the waypoint snapshots from disk.
            with open(self._graph_path + "/waypoint_snapshots/{}".format(waypoint.snapshot_id),
                      "rb") as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
        for edge in self._current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            
            # Load the edge snapshots from disk.
            with open(self._graph_path + "/edge_snapshots/{}".format(edge.snapshot_id),
                      "rb") as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        
        # Upload the graph to the robot.
        print("Uploading the graph and snapshots to the robot...")
        true_if_empty = not len(self._current_graph.anchoring.anchors)
        response = self._graph_nav_client.upload_graph(graph=self._current_graph,
                                                       generate_new_anchoring=true_if_empty)
        # Upload the snapshots to the robot.
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = self._current_waypoint_snapshots[snapshot_id]
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            print("Uploaded {}".format(waypoint_snapshot.id))
        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = self._current_edge_snapshots[snapshot_id]
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            print("Uploaded {}".format(edge_snapshot.id))

        # The upload is complete! Check that the robot is localized to the graph,
        # and if it is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            print("\n")
            print("Upload complete! The robot is currently not localized to the map")

    def _update_graph_waypoint_and_edge_ids(self, do_print=False):
        """
        Update internal dictionaries of waypoints and edges from the current robot map.

        Parameters
        ----------
        do_print : bool, optional
            Whether to print the waypoint and edge ID mappings to stdout.
        """
        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print("Empty graph.")
            return
        self._current_graph = graph

        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id

        # Update and print waypoints and edges
        self._current_annotation_name_to_wp_id, self._current_edges = graph_nav_util.update_waypoints_and_edges(
            graph, localization_id, do_print)

    def load_graph(self):
        """
        Load and upload the graph from disk, then localize the robot if needed.
        """
        self._upload_graph_and_snapshots()

        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            self._set_initial_localization_fiducial()
            self._update_graph_waypoint_and_edge_ids()

    def _get_waypoint(self, id):
        """
        Get waypoint from graph (return None if waypoint not found)

        Parameters
        ----------
        id : str
            The ID of the waypoint to retrieve.

        Returns
        -------
        map_pb2.Waypoint or None
            The corresponding waypoint object, or None if not found.
        """
        if self._current_graph is None:
            self._current_graph = self._graph_nav_client.download_graph()

        for waypoint in self._current_graph.waypoints:
            if waypoint.id == id:
                return waypoint

        print('ERROR: Waypoint {} not found in graph.'.format(id))
        return None
    
    def _get_transform(self, from_wp, to_wp):
        """
        Get transform from from-waypoint to to-waypoint.

        Parameters
        ----------
        from_wp : bosdyn.api.graph_nav.map_pb2.Waypoint
            The source waypoint.
        to_wp : bosdyn.api.graph_nav.map_pb2.Waypoint
            The destination waypoint.

        Returns
        -------
        bosdyn.api.geometry.SE3Pose
            The transform from `from_wp` to `to_wp`, in proto format.
        """
        from_se3 = from_wp.waypoint_tform_ko
        from_tf = SE3Pose(
            from_se3.position.x, from_se3.position.y, from_se3.position.z,
            Quat(w=from_se3.rotation.w, x=from_se3.rotation.x, y=from_se3.rotation.y,
                 z=from_se3.rotation.z))

        to_se3 = to_wp.waypoint_tform_ko
        to_tf = SE3Pose(
            to_se3.position.x, to_se3.position.y, to_se3.position.z,
            Quat(w=to_se3.rotation.w, x=to_se3.rotation.x, y=to_se3.rotation.y,
                 z=to_se3.rotation.z))

        from_T_to = from_tf.mult(to_tf.inverse())
        return from_T_to.to_proto()
    
    def get_waypoint_count(self, prefix=None):
        """
        Return the number of waypoints currently in the graph that match a given prefix.

        Parameters
        ----------
        prefix : str, optional
            The prefix to filter waypoint names. If None, defaults to "Waypoint_".

        Returns
        -------
        int
            The number of waypoints in the currently loaded graph.
        """
        if prefix is not None:
            num_points = len([key for key in self._current_annotation_name_to_wp_id.keys() if prefix in key])
        else:
            num_points = len([key for key in self._current_annotation_name_to_wp_id.keys() if 'Waypoint_' in key])

        return num_points
    
    def get_relative_pose_from_waypoint(self, waypoint_name):
        """
        Compute the robot's SE3 pose relative to a specified waypoint.

        Parameters
        ----------
        waypoint_name : str
            The name of the waypoint.

        Returns
        -------
        bosdyn.client.math_helpers.SE3Pose
            The robot's pose relative to the specified waypoint.
        """
        waypoint_id = graph_nav_util.find_unique_waypoint_id(waypoint_name, self._current_graph, self._current_annotation_name_to_wp_id)

        localization_state = self._graph_nav_client.get_localization_state()
        localize_waypoint_id = localization_state.localization.waypoint_id

        if waypoint_id == localize_waypoint_id:
            relative_pose = SE3Pose.from_proto(localization_state.localization.waypoint_tform_body)

        else:
            waypoint = self._get_waypoint(waypoint_id)
            localize_waypoint = self._get_waypoint(localize_waypoint_id)
            T_wp_locwp = SE3Pose.from_proto(self._get_transform(waypoint, localize_waypoint))
            waypoint_tform_body_se3 = SE3Pose.from_proto(localization_state.localization.waypoint_tform_body)
            relative_pose = T_wp_locwp.mult(waypoint_tform_body_se3)

        return relative_pose
    
# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, os, sys
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--graph-path',
                        help='Full filepath to graph.',
                        default=os.getcwd())
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('GraphBase')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    try:
        graph_base = GraphBase(robot, options.graph_path)
        graph_base.load_graph()

        num_waypoints = graph_base.get_waypoint_count()
        print(f'Nuber of Waypoint: {num_waypoints}')
        
        while True:
            input_str = input("Enter the index of the waypoint to view the robot's pose relative to it, to end type q:")
            while not input_str.isdigit() and input_str != 'q':
                print('wrong input')
                input_str = input("Enter the index of the waypoint to view the robot's pose relative to it, to end type q:")
            
            if input_str == 'q':
                break
            
            relative_pose = graph_base.get_relative_pose_from_waypoint(f'Waypoint_{input_str}')
            print(relative_pose)

    except Exception as exc:  # pylint: disable=broad-except
        print("GraphBase threw an error.")
        print(exc)