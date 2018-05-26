import argparse
import time
import msgpack
import logging
import datetime
import csv

from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, path_pruning, find_goal, adjacent_point
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from skimage.morphology import medial_axis
from skimage.util import invert


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
        self.prefix = f'{datetime.datetime.now():%Y-%m-%d-%H-%M}'
        flight_fname = self.prefix + '_flight_log.txt'
        super().__init__(connection, tlog_name=flight_fname )

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        logger.info('Arming Transition')
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        logger.info('Takeoff Transition')
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        logger.info('Waypoint Transition')
        self.target_position = self.waypoints.pop(0)
        print('Target Position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        logger.info('Landing Transition')
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        logger.info('Disarm Transition')
        self.disarm()
        self.release_control()

    def manual_transition(self):
        logger.info('Manual Transition')
        distance_from_home = 100*np.linalg.norm(self.local_position)
        logger.info('Distance From Origin : {:3.2f} cm'.format(distance_from_home))
        print('Arrived At Goal', self.target_position)

    

        self.flight_state = States.MANUAL
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending Waypoints To Simulator... Waiting...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching For A Path... Waiting...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            origin_pos_data = f.readline().split(',')
        lat0 = float(origin_pos_data[0].strip().split(' ')[1])
        lon0 = float(origin_pos_data[1].strip().split(' ')[1])        

        # Set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # Tetrieve current global position
        global_pos_current = [self._longitude, self._latitude, self._altitude]

        # Convert to current local position using global_to_local()
        local_pos_current = global_to_local(global_pos_current, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(
            self.global_home, self.global_position, self.local_position))
        print('global home {0}, position {1}, current local position {2}'.format(
            self.global_home, global_pos_current, local_pos_current))     


        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset, points = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("Grid = {}, north offset = {}, east offset = {}".format(grid.shape, north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # init_pos_grid = (-north_offset, -east_offset)

        # Convert start position to current position rather than map center
        grid_start = (int(local_pos_current[0] -north_offset)
                    , int(local_pos_current[1] -east_offset))

        
        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
    
        # Adapt to set goal as latitude / longitude position and convert
        global_goal = (-122.39827335, 37.79639627, 0)
        local_goal = global_to_local(global_goal, self.global_home)
        grid_goal = (int(local_goal[0] - north_offset),
                     int(local_goal[1] - east_offset)) 

        # Run A* to find a path from start to goal
        # Add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)


        # A* == Default
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        # A* == Medial  
        # https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3663081/
        topological_skeleton = medial_axis(invert(grid))
        adjacent_grid_start, adjacent_grid_goal = find_goal(topological_skeleton, grid_start, grid_goal)

        print('Closest Local Start and Goal: ', adjacent_grid_start, adjacent_grid_goal)
  
        
        path, _ = a_star(invert(topological_skeleton).astype(np.int), heuristic, tuple(adjacent_grid_start), tuple(adjacent_grid_goal))

        # Prune path to minimize number of waypoints
        pruned_path = path_pruning(path)

        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        print(self.waypoints)

        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        logger.info('Creating Log File')
        self.start_log('Logs', self.prefix + '_flightLog.txt')
        logger.info('Log File - {}'.format(self.prefix + '_flightLog.txt'))
        # print('Starting connection')
        logger.info('Starting Connection')
        self.connection.start()
        while self.in_mission:
           pass
        # print('Closing log file')
        logger.info('Closing Log File')
        self.stop_log()


if __name__ == "__main__":

    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    prefix = f'{datetime.datetime.now():%Y-%m-%d-%H-%M}'
    run_log = prefix + '_run_log.txt'

    fh = logging.FileHandler(run_log)
    fh.setLevel(logging.INFO)
    fh.setFormatter(formatter)
    logger.addHandler(fh)

    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    logger.info('Logging Flight Run In {}'.format(run_log))



    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--waypoint_file', type=str, default='', help='csv containing waypoints, i.e. square_waypoints.csv')

    args = parser.parse_args()


    waypoints = []
    if args.waypoint_file != '':
        print('Using waypoint file : ' + args.waypoint_file )
        logger.info('Using waypoint file : ' + args.waypoint_file)
        with open(args.waypoint_file) as dataFile:
            dataReader = csv.reader(dataFile)
            for row in dataReader:
                waypoint = [ float(row[0]), float(row[1]), float(row[2]) ]
                waypoints.append(waypoint)


    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    drone.all_waypoints = waypoints
    time.sleep(1)
    drone.start()
