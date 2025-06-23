import os, time
from SpotStack.graph.graph_core import GraphCore, graph_nav_util

from bosdyn.api.graph_nav import map_pb2
from bosdyn.client.recording import GraphNavRecordingServiceClient, NotReadyYetError

class GraphRecorder(GraphCore):
    """
    Class for recording GraphNav maps with Spot.

    Inherits from GraphBase and provides methods to:
    - Start and stop recording maps.
    - Record waypoints and edges.
    - Save maps and snapshots locally.
    - Download and upload maps to and from the robot.
    """

    def __init__(self, robot, graph_path, is_new_map=True):
        """
        Initialize the GraphRecorder.

        Parameters
        ----------
        robot : bosdyn.client.robot.Robot
            The Spot robot instance.
        graph_path : str
            Path to save or load graph and snapshot files.
        is_new_map : bool, optional
            If False, an existing graph is uploaded and localization is attempted (default is True).
        """
        self._recording_client = robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)   
        self.stop_recording()

        super().__init__(robot, graph_path)

        if not is_new_map:
            self.load_graph()

    # Recording Action functions
    def start_recording(self):
        """Start recording a map."""
        try:
            status = self._recording_client.start_recording()
            print("GraphRecorder: Successfully started recording a map.")
        except Exception as err:
            print("GraphRecorder: Start recording failed: " + str(err))

    def stop_recording(self):
        """Stop or pause recording a map."""
        first_iter = True
        while True:
            try:
                status = self._recording_client.stop_recording()
                print("GraphRecorder: Successfully stopped recording a map.")
                break
            except NotReadyYetError as err:
                # It is possible that we are not finished recording yet due to
                # background processing. Try again every 1 second.
                if first_iter:
                    print("GraphRecorder: Cleaning up recording...")
                first_iter = False
                time.sleep(1.0)
                continue
            except Exception as err:
                print("GraphRecorder: Stop recording failed: " + str(err))
                break

    def get_recording_status(self):
        """Get the recording service's status."""
        status = self._recording_client.get_record_status()
        if status.is_recording:
            print("GraphRecorder: The recording service is on.")
        else:
            print("GraphRecorder: The recording service is off.")
        return status

    def record_waypoint(self, name):
        """
        Record current pose as a new waypoint with a given name.

        Parameters
        ----------
        name : str
            Name to assign to the waypoint.
        """
        self._recording_client.create_waypoint(waypoint_name=name)
        print(f'GraphRecorder: Waypoint {name} Recorded.')
    
    def create_new_edge(self, waypoint1, waypoint2):
        """
        Create a new edge between two existing waypoints using kinematic odometry.

        Parameters
        ----------
        waypoint1 : str
            Name or ID of the source waypoint.
        waypoint2 : str
            Name or ID of the destination waypoint.
        """
        self._update_graph_waypoint_and_edge_ids(do_print=False)

        from_id = graph_nav_util.find_unique_waypoint_id(waypoint1, self._current_graph,
                                                         self._current_annotation_name_to_wp_id)
        to_id = graph_nav_util.find_unique_waypoint_id(waypoint2, self._current_graph,
                                                       self._current_annotation_name_to_wp_id)

        print("GraphRecorder: Creating edge from {} to {}.".format(from_id, to_id))

        from_wp = self._get_waypoint(from_id)
        if from_wp is None:
            return

        to_wp = self._get_waypoint(to_id)
        if to_wp is None:
            return

        # Get edge transform based on kinematic odometry
        edge_transform = self._get_transform(from_wp, to_wp)

        # Define new edge
        new_edge = map_pb2.Edge()
        new_edge.id.from_waypoint = from_id
        new_edge.id.to_waypoint = to_id
        new_edge.from_tform_to.CopyFrom(edge_transform)

        # Send request to add edge to map
        try:
            self._recording_client.create_edge(edge=new_edge)
        except Exception as e:
            print(f'GraphRecorder: error creating an edge. {e}')

    # Download graph
    def _write_bytes(self, filepath, filename, data):
        """
        Write binary data to a file.

        Parameters
        ----------
        filepath : str
            The directory where the file will be written.
        filename : str
            The name of the file to write.
        data : bytes
            Serialized data to write.
        """
        os.makedirs(filepath, exist_ok=True)
        with open(filepath + filename, 'wb+') as f:
            f.write(data)
            f.close()

    def _write_full_graph(self, graph):
        """
        Download the graph protobuf from robot to the specified, local filepath location.

        Parameters
        ----------
        graph : bosdyn.api.graph_nav.map_pb2.Graph
            The graph to serialize and save.
        """
        graph_bytes = graph.SerializeToString()
        self._write_bytes(self._graph_path, '/graph', graph_bytes)

    def _download_and_write_waypoint_snapshots(self, waypoints):
        """
        Download the waypoint snapshots from robot to the specified, local filepath location.
        
        Parameters
        ----------
        waypoints : google._upb._message.RepeatedCompositeContainer
            List of waypoints whose snapshots will be downloaded.
        """
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id)
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                print("GraphRecorder: Failed to download waypoint snapshot: " + waypoint.snapshot_id)
                continue
            self._write_bytes(self._graph_path + '/waypoint_snapshots',
                              '/' + waypoint.snapshot_id, waypoint_snapshot.SerializeToString())
            num_waypoint_snapshots_downloaded += 1
            print("GraphRecorder: Downloaded {} of the total {} waypoint snapshots.".format(
                num_waypoint_snapshots_downloaded, len(waypoints)))

    def _download_and_write_edge_snapshots(self, edges):
        """
        Download the edge snapshots from robot to the specified, local filepath location.
        
        Parameters
        ----------
        edges : google._upb._message.RepeatedCompositeContainer
            List of edges whose snapshots will be downloaded.
        """
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                print("GraphRecorder: Failed to download edge snapshot: " + edge.snapshot_id)
                continue
            self._write_bytes(self._graph_path + '/edge_snapshots', '/' + edge.snapshot_id,
                              edge_snapshot.SerializeToString())
            num_edge_snapshots_downloaded += 1
            print("GraphRecorder: Downloaded {} of the total {} edge snapshots.".format(
                num_edge_snapshots_downloaded, num_to_download))

    def download_full_graph(self):
        """
        Download the current map and its snapshots from the robot.

        This includes:
        - The topological graph structure
        - All waypoint snapshots
        - All edge snapshots

        Saves them under the path specified at initialization.
        """
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print("GraphRecorder: Failed to download the graph.")
            return
        self._write_full_graph(graph)
        print("GraphRecorder: Graph downloaded with {} waypoints and {} edges".format(
            len(graph.waypoints), len(graph.edges)))
        # Download the waypoint and edge snapshots.
        self._download_and_write_waypoint_snapshots(graph.waypoints)
        self._download_and_write_edge_snapshots(graph.edges)

# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, sys
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--graph-path',
                        help='Full filepath for where to download graph and snapshots or filepath to old graph and snapshots to be uploaded.',
                        default=os.getcwd())
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('GraphRecorder')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    graph_recorder = GraphRecorder(robot, options.graph_path, is_new_map=True)

    try:
        graph_recorder.start_recording()
        waypoint_index = 0
        while True:
            input_str = input(f"Go to Waypoint_{waypoint_index}. When finished, type k, to end type q:")
            while input_str != 'k' and input_str != 'q':
                print('wrong input')
                input_str = input(f"Go to Waypoint_{waypoint_index}. When finished, type k, to end type q:")
            
            if input_str == 'q':
                break
            
            graph_recorder.record_waypoint(f'Waypoint_{waypoint_index}')
            waypoint_index += 1
        
        graph_recorder.stop_recording()
        graph_recorder.download_full_graph()
        print("Graph is downloaded !")

    except Exception as exc:  # pylint: disable=broad-except
        print("GraphRecorder threw an error.")
        print(exc)