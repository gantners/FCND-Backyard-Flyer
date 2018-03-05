import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class Phases(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.waypoint_index = 0

        #
        self.all_waypoints = self.calculate_box()

        # initial state
        self.flight_state = Phases.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == Phases.TAKEOFF:

            # coordinate conversion
            altitude = -1.0 * self.local_position[2]

            # check if altitude is above 95% of target altitude
            if altitude > 0.95 * self.target_position[2]:
                self.waypoint_transition()
                return

        if self.flight_state == Phases.WAYPOINT:
            # check waypoint reached
            north = self.local_position[0]
            east = self.local_position[1]
            northTarget = self.target_position[0]
            eastTarget = self.target_position[1]
            print("-----------------------------------------------------------")
            print("North: {}, East: {}".format(north, east))
            if self.close_too(northTarget,north) and self.close_too(eastTarget,east):
                print("Waypoint reached, NorthTarget: {}, EastTarget: {}".format(northTarget, eastTarget))
                self.waypoint_index += 1

            if self.waypoint_index >= len(self.all_waypoints):
                # all waypoints reached
                print("Reached final position, landing")
                self.landing_transition()
            else:
                self.waypoint_transition()
            return

    def close_too(self, target, point, within = 0.2):
        return abs(target - point) < within

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == Phases.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
                    abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """

        if not self.in_mission:
            return
        if self.flight_state == Phases.MANUAL:
            self.arming_transition()
        elif self.flight_state == Phases.ARMING:
            self.takeoff_transition()
        elif self.flight_state == Phases.DISARMING:
            self.manual_transition()

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_state = Phases.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = Phases.TAKEOFF

    def calculate_box(self):
        """TODO: Fill out this method

        1. Return waypoints to fly a box
        """
        all_waypoints = []
        all_waypoints.append([10,  0,  3, 0])
        all_waypoints.append([10, 10, 3, 0])
        all_waypoints.append([0, 10, 3, 0])
        all_waypoints.append([0, 0, 3, 0])

        return all_waypoints

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
       # print("waypoint transition")
        self.target_position = self.all_waypoints[self.waypoint_index]
        print("next target: {}, {}, {}, {}".format(self.target_position[0],self.target_position[1],self.target_position[2],self.target_position[3]))
        self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],self.target_position[3])
        self.flight_state = Phases.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = Phases.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = Phases.DISARMING

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
        self.flight_state = Phases.MANUAL

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
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
