#!/usr/bin/env python

import rospy
import numpy
import random
import statistics
import csv
from geometry_msgs.msg import Point
from mrs_msgs.srv import ReferenceStampedSrv, ReferenceSrv, ReferenceSrvRequest, ReferenceStampedSrvRequest
from mrs_msgs.msg import ReferenceStamped, UavState
from std_msgs.msg import Float64, Int32
from nav_msgs.msg import Odometry
from threading import Thread, Lock
from math import pi, cos, sin, sqrt

NUM_UAVS = 10
BOUND_X = 10.0 # Box dimensions
BOUND_Y = 10.0
BOUND_Z = 3.0
MIN_GOAL_DISTANCE = 2.5 # Minimum distance between goals
GOAL_TOLERANCE = 0.1 # Goal tolerance in meters
GOAL_HOLD_TIME = 0.5 # Time to stay at the goal
COLLISION_THRESHOLD = 0.5
REPLACE_GOAL_FREQUENCY = 0.0

uav_states = {}
goal_positions = {}
goal_reached_times = {}
publishers_uav_state = {}
in_collision = {}
flight_dists_uavs = {}
flight_vels_uavs = {}
max_flight_vels_uavs = {}
number_of_messages_uavs = {}
number_of_collisions = 0
min_dist_global = 1000.0
active = False

mutex = Lock()

def compute_distance(p1, p2):
    """Compute Euclidean distance between two points."""
    return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

def compute_minimum_distance(current_uav, uav_positions):
    """Compute the minimum distance from current UAV to all other UAVs."""
    min_dist = float('inf')
    for uav_id, position in uav_positions.items():
        if uav_id != current_uav:
            dist = compute_distance(uav_positions[current_uav], position)
            if dist < min_dist:
                min_dist = dist

    return min_dist

def get_vec3_norm(v):
    return sqrt(v.x**2 + v.y**2 + v.z**2)

def uav_state_callback(msg, uav_id):
    """Callback to update the UAV state."""
    global uav_states, active 
    if uav_id in uav_states.keys() and active:
        with mutex:
            flight_dists_uavs[uav_id].append(compute_distance(uav_states[uav_id], msg.pose.pose.position))
            vel_norm = get_vec3_norm(msg.twist.twist.linear)
            flight_vels_uavs[uav_id].append(vel_norm)
            if vel_norm > max_flight_vels_uavs[uav_id]:
                max_flight_vels_uavs[uav_id] = vel_norm

    uav_states[uav_id] = msg.pose.pose.position

def call_reference_service(uav_id, x, y, z, heading):
    """Helper function to call the reference service for a specific UAV."""
    service_name = f"/uav{uav_id}/rbl_controller/set_goal"
    
    try:
        # Create a service proxy
        reference_service = rospy.ServiceProxy(service_name, ReferenceSrv)

        # Prepare the request
        request = ReferenceSrvRequest()
        
        # Set the reference position
        request.reference.position = Point(x=x, y=y, z=z)
        request.reference.heading = heading

        # Call the service and process the response
        response = reference_service(request)
        if not response.success:
            rospy.logwarn(f"Failed to set reference for UAV {uav_id}: {response.message}")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed for UAV {uav_id}: {e}")

def set_uav_positions(direction_there):
    # Initialize the ROS node
    scale = 10.0  # Circle radius
    z_position = 1.0  # Common altitude for UAVs
    heading = 0.0  # Default heading (no rotation)

    if direction_there:
        scale = -10

    for i in range(NUM_UAVS):
        angle = 2 * pi * i / NUM_UAVS  # Equally distribute around the circle
        x = scale * cos(angle)
        y = scale * sin(angle)
        call_reference_service(i + 1, x, y, z_position, heading)

        goal_positions[i+1] = Point(x, y, z_position)
        goal_reached_times[i+1] = None


def write_results_to_file(filename, flight_times, flight_dists, flight_vels, min_dists, max_vels):

    with open(filename, "w") as f:
        f.write("Trial, Success, FlightTime, FlightDist, FlightVel, MinDist, MaxVel\n")
        failures = 0
        for i in range(len(flight_times)):
            success = min_dists[i] >= COLLISION_THRESHOLD
            f.write(f"{i:d}, {success:d}, {flight_times[i]:.4f}, {flight_dists[i]:.4f}, {flight_vels[i]:.4f}, {min_dists[i]:.4f}, {max_vels[i]:.4f}\n")
            if not success:
                failures += 1

        # compute stats 
        success_rate = 1 - (failures / float(len(flight_times)))
        mean_flight_time = statistics.mean(flight_times)
        stdev_flight_time = statistics.stdev(flight_times)
        mean_flight_dist = statistics.mean(flight_dists)
        stdev_flight_dist = statistics.stdev(flight_dists)
        mean_flight_vel = statistics.mean(flight_vels)
        stdev_flight_vel = statistics.stdev(flight_vels)
        mean_min_dist = statistics.mean(min_dists)
        stdev_min_dist = statistics.stdev(min_dists)
        mean_max_vel = statistics.mean(max_vels)
        stdev_max_vel = statistics.stdev(max_vels)
        f.write("-------------------------------------------------------\n")
        f.write(f"Succes rate: {success_rate:.4f}\n")
        f.write(f"Flight time: mean = {mean_flight_time:.4f}, stdev = {stdev_flight_time:.4f}\n")
        f.write(f"Flight dist: mean = {mean_flight_dist:.4f}, stdev = {stdev_flight_dist:.4f}\n")
        f.write(f"Flight vel: mean = {mean_flight_vel:.4f}, stdev = {stdev_flight_vel:.4f}\n")
        f.write(f"Minimum dist.: mean = {mean_min_dist:.4f}, stdev = {stdev_min_dist:.4f}\n")
        f.write(f"Max. flight vel: mean = {mean_max_vel:.4f}, stdev = {stdev_max_vel:.4f}\n")

def load_previous_data(filename):
    """
    Load data from the file generated by write_results_to_file.
    
    Args:
        filename (str): The name of the file to read from.
    
    Returns:
        tuple: A tuple containing the following lists:
               - flight_times
               - flight_dists
               - flight_vels
               - min_dists
               - max_vels
    """
    flight_times = []
    flight_dists = []
    flight_vels = []
    min_dists = []
    max_vels = []

    try:
        with open(filename, "r") as f:
            reader = csv.reader(f)
            # Skip the header row
            next(reader)
            
            for row in reader:
                if len(row) == 0 or row[0].startswith('-'):  # Stop at the stats section
                    break
                
                # Extract data from the row
                trial = int(row[0])  # Trial number (not used in outputs)
                success = int(row[1])  # Success flag (not used in outputs)
                flight_times.append(float(row[2]))
                flight_dists.append(float(row[3]))
                flight_vels.append(float(row[4]))
                min_dists.append(float(row[5]))
                max_vels.append(float(row[6]))
                
    except FileNotFoundError:
        print(f"File '{filename}' not found. Starting fresh.")
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")

    return flight_times, flight_dists, flight_vels, min_dists, max_vels

def main():

    global active
    rospy.init_node("antipodal_positions_tester", anonymous=True)
    rospy.loginfo("Antipodal circle positions test started.")

    filename = "results_antipodal_positions.txt"
    flight_times, flight_dists, flight_vels, min_dists, max_vels = load_previous_data(filename)

    # Subscribe to UAV state topics
    for uav_id in range(1, NUM_UAVS + 1):
        flight_vels_uavs[uav_id] = []
        flight_dists_uavs[uav_id] = []
        max_flight_vels_uavs[uav_id] = 0
        number_of_messages_uavs[uav_id] = 0
        rospy.Subscriber(
            f"/uav{uav_id}/estimation_manager/odom_main", Odometry,
            uav_state_callback, uav_id
        )

    # Wait for all services to be available
    rospy.loginfo('Waiting for all services to be available.')
    for uav_id in range(1, NUM_UAVS + 1):
        service_name = f"/uav{uav_id}/control_manager/reference"
        rospy.wait_for_service(service_name)

    rate = rospy.Rate(100)  # 10 Hz

    # Wait for all states to be available
    while len(uav_states) < NUM_UAVS:
        rospy.loginfo_throttle(1.0, 'Waiting for all uav states to be available.')
        rate.sleep()

    # Initialize goal positions
    for uav_id in range(1, NUM_UAVS + 1):
        goal_positions[uav_id] = uav_states[uav_id]
        goal_reached_times[uav_id] = None

    # fly to initial position
    number_of_goals_navigated = 0
    min_dist_global = float('inf')
    direction_there = False
    set_uav_positions(direction_there)
    start_time = rospy.Time.now()
    active = True
    flight_to_init_pose = True

    rospy.loginfo('Entering while loop.')
    while not rospy.is_shutdown() and number_of_goals_navigated < 100:
        goal_reached = True
        global_dist_to_goal = 0
        for uav_id in range(1, NUM_UAVS + 1):

            # Check if the UAV is within the goal tolerance
            dist_to_goal = sqrt(
                (uav_states[uav_id].x - goal_positions[uav_id].x) ** 2 +
                (uav_states[uav_id].y - goal_positions[uav_id].y) ** 2 +
                (uav_states[uav_id].z - goal_positions[uav_id].z) ** 2
            )

            if dist_to_goal > global_dist_to_goal: 
                global_dist_to_goal = dist_to_goal

            min_dist = compute_minimum_distance(uav_id, uav_states)

            if min_dist < min_dist_global: 
                min_dist_global = min_dist

            if dist_to_goal < GOAL_TOLERANCE:
                if goal_reached_times[uav_id] is None:
                    goal_reached_times[uav_id] = rospy.Time.now()
                    number_of_messages_uavs[uav_id] = len(flight_vels_uavs[uav_id])
                    goal_reached = False
                elif (rospy.Time.now() - goal_reached_times[uav_id]).to_sec() < GOAL_HOLD_TIME:
                    # Goal hold time not satisfied
                    goal_reached = False
            else:
                goal_reached_times[uav_id] = None
                goal_reached = False

        rospy.loginfo_throttle(0.5, 'Dist to goal = {0:.2f}'.format(global_dist_to_goal))

        if goal_reached:

            active = False

            if not flight_to_init_pose:
                flight_vels_per_uav = []
                flight_times_per_uav = []
                flight_dist_total = 0
                with mutex: 
                    for uav_id in range(1, NUM_UAVS + 1):
                        flight_times_per_uav.append((goal_reached_times[uav_id] - start_time).to_sec())
                        flight_dist_total += sum(flight_dists_uavs[uav_id][:number_of_messages_uavs[uav_id]])
                        flight_vels_per_uav.append(statistics.mean(flight_vels_uavs[uav_id][:number_of_messages_uavs[uav_id]]))
                        goal_reached_times[uav_id] = None

                number_of_goals_navigated += 1
                rospy.loginfo('Goal reached')

                flight_times.append(max(flight_times_per_uav))
                flight_dists.append(flight_dist_total/float(NUM_UAVS))
                flight_vels.append(statistics.mean(flight_vels_per_uav))
                min_dists.append(min_dist_global)
                max_vels.append(max(max_flight_vels_uavs.values()))
                rospy.loginfo('#goals = {5:d}, Flight time = {0:.2f}, flight dist = {1:.2f}, vel = {2:.2f}, min_dist = {3:.2f}, max_vel = {4:.2f}'.format(flight_times[-1], flight_dists[-1], flight_vels[-1], min_dists[-1], max_vels[-1], number_of_goals_navigated))

                if len(flight_times) >= 2:
                    write_results_to_file(filename, flight_times, flight_dists, flight_vels, min_dists, max_vels)

            flight_to_init_pose = False
            min_dist_global = float('inf')
            direction_there = not direction_there
            with mutex: 
                for uav_id in range(1, NUM_UAVS + 1):
                    flight_vels_uavs[uav_id] = []
                    flight_dists_uavs[uav_id] = []
                    max_flight_vels_uavs[uav_id] = 0
                    number_of_messages_uavs[uav_id] = 0

            start_time = rospy.Time.now()
            set_uav_positions(direction_there)
            active = True

        rate.sleep()

    rospy.loginfo('Evaluation finished')
    write_results_to_file("results_antipodal_positions.txt", flight_times, flight_dists, flight_vels, min_dists, max_vels)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
