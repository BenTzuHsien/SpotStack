from . import graph_nav_util

from bosdyn.api.graph_nav import map_pb2
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.graph_nav import GraphNavClient, nav_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body

class GraphBase(object):
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