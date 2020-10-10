import argparse
import time
from enum import Enum, auto

import numpy as np

from udacidrone.connection import CrazyflieConnection

from planning_utils import construct_road_map_crazyflie, find_start_goal, heuristic, a_star_graph, condense_waypoints_crazyflie, visualize_prob_road_map_crazyflie

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # initialize waypoints to navigate
        # self.all_waypoints = self.calculate_box()
        self.all_waypoints = self.plan_path_graph()

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.MANUAL:
            self.takeoff_transition()
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.15:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 0.1:
                        self.landing_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            if abs(self.local_position[2] < 0.01):
                self.manual_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_state == States.ARMING:
            # self.takeoff_transition()
            self.plan_path_graph()
        elif self.flight_state == States.PLANNING:
            self.takeoff_transition()
        if self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method

        1. Return waypoints to fly a box
            1--------2
            |        |
            |        |
            0--------3
            4
        """
        # add first visted point last since pop() is
        # LIFO -- Last In First Out
        cp = self.local_position
        cp[2] = 0
        local_waypoints = [cp + [0.75, 0.0, 0.5],
                           cp + [0.75, 0.75, 0.5],
                           cp + [0.0, 0.75, 0.5],
                           cp + [0.0, 0.0, 0.5]]
        local_waypoints.reverse()
        return local_waypoints

    def takeoff_transition(self):
        """TODO: Fill out this method

        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 0.3
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        # set appropriate state
        self.flight_state = States.TAKEOFF
        # while len(self.all_waypoints) == 0:
        #     time.sleep(1)

    def waypoint_transition(self):
        """TODO: Fill out this method

        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        if self.all_waypoints:
            waypoint = self.all_waypoints.pop(0)
            print(f'waypoint to navigate to: {waypoint}')
            north, east, altitude = waypoint
            print(north, east, altitude)
            self.target_position[0] = north
            self.target_position[1] = east
            self.target_position[2] = altitude
            self.cmd_position(north, east, altitude, 0)
            self.flight_state = States.WAYPOINT
        else:
            self.landing_transition()

    def landing_transition(self):
        """TODO: Fill out this method

        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def read_home(self, file_path):
        lat, lon = None, None
        with open(file_path) as f:
            first_line = f.readline()
            lat_lon_str = first_line.split(",")
            lat_lon_str = list(map(str.strip, lat_lon_str))
            lat = lat_lon_str[0].split(" ")[1]
            lon = lat_lon_str[1].split(" ")[1]

        return float(lat), float(lon)

    def plan_path_graph(self):
        """
        Construct a configuration space using a graph representation, set destination GPS position, find a path from start (current) position to destination, and minimize and set waypoints.
        """
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        data = np.loadtxt('map_obstacle_course.csv', delimiter=',',
                          dtype='Float64', skiprows=2)
        TARGET_ALTITUDE = 0.3 / 0.0254
        SAFETY_DISTANCE = 0.25 / 0.0254
        st = time.time()
        # grid, G = create_graph_and_edges_crazyflie(
        #     data, 130, 86, TARGET_ALTITUDE, SAFETY_DISTANCE)
        GRID_NORTH_SIZE = 130
        GRID_EAST_SIZE = 86
        NUMBER_OF_NODES = 200
        OUTDEGREE = 4
        grid, G = construct_road_map_crazyflie(
            data, GRID_NORTH_SIZE, GRID_EAST_SIZE, TARGET_ALTITUDE, SAFETY_DISTANCE, NUMBER_OF_NODES, OUTDEGREE)
        # visualize_prob_road_map_crazyflie(data, grid, G)
        time_taken = time.time() - st
        print(f'create_graph_and_edges() took: {time_taken} seconds')
        grid_start = (32, 40, 0)
        grid_goal = (110, 40, 0)
        # Find closest node on the graph
        g_start, g_goal = find_start_goal(G, grid_start, grid_goal)
        print("Start and Goal location:", grid_start, grid_goal)
        print("Start and Goal location on graph:", g_start, g_goal)
        path, _ = a_star_graph(G, heuristic, g_start, g_goal)
        path.append(grid_goal)
        new_path = []
        # new_path.append(grid_start)
        new_path.append(g_start)
        new_path.extend(path)
        print(new_path)
        unique_path = []
        for p in new_path:
            if p not in unique_path:
                unique_path.append(p)
        print(unique_path)
        # Reduce waypoints
        reduced_path = condense_waypoints_crazyflie(grid, unique_path)
        print(f'reduced_path: {reduced_path}')
        waypoints = []
        # modify path coordinates as offset from the takeoff position
        # i.e., we treat the firt position as origin
        for p in reduced_path:
            print(np.array(p) - np.array(grid_start))
            offset = (np.array(p) - np.array(grid_start)) * 0.0254
            offset[2] = TARGET_ALTITUDE * 0.0254
            waypoints.append(list(offset))
        print(f'All waypoints: {waypoints}')
        self.all_waypoints = waypoints

        visualize_prob_road_map_crazyflie(data, grid, G, grid_start,
                                          grid_goal, unique_path,
                                          all_nodes=False)
        visualize_prob_road_map_crazyflie(data, grid, G, grid_start,
                                          grid_goal, reduced_path,
                                          all_nodes=False)

        return self.all_waypoints

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = CrazyflieConnection('radio://0/80/2M')
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = MotionPlanning(conn)
    time.sleep(2)
    drone.start()
