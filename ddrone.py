#!/usr/bin/env python

"""
set_attitude_target.py: (Copter Only)
This example shows how to move/direct Copter and send commands
 in GUIDED_NOGPS mode using DroneKit Python.
Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.
Tested in Python 2.7.10
"""



ground_speed = 30
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import random

# Set up option parsing to get connection string
import argparse
connection_string = 'udp:127.0.0.1:14550'

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude * 0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust=thrust)
        time.sleep(0.2)

def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle=0.0, pitch_angle=0.0,
                 yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                 thrust=0.5, duration=0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# Provide your vehicle's current x, y, z, roll, pitch, yaw, and timestamp values
current_x = 26
current_y = 78
current_z = 90
current_roll = -45 
current_pitch = 67
current_yaw=70
current_timestamp = 10
print("x:", current_x)
print("y:", current_y)
print("z:", current_z)
print("roll:", current_roll)
print("pitch:", current_pitch)
print("yaw:", current_yaw)
print("timestamps:", current_timestamp)
# Set the vehicle's current state
vehicle.location.global_relative_frame.lat = current_x
vehicle.location.global_relative_frame.lon = current_y
vehicle.location.global_relative_frame.alt = current_z
vehicle.attitude.roll = current_roll
vehicle.attitude.pitch = current_pitch
vehicle.attitude.yaw = current_yaw
vehicle.time_stamp = current_timestamp

# Take off 2.5m in GUIDED_NOGPS mode.
arm_and_takeoff_nogps(15)

# Hold the position for 3 seconds.
print("Hold position for 3 seconds")
set_attitude(duration=3)

# Uncomment the lines below for testing roll angle and yaw rate.
# Make sure that there is enough space for testing this.

# set_attitude(roll_angle=1, thrust=0.5, duration=3)
# set_attitude(yaw_rate=30, thrust=0.5, duration=3)

# Move the drone forward and backward.
# Note that it will be in front of the original position due to inertia
gotodistance= 20

print("Move forward")
set_attitude(pitch_angle=-5, thrust=0.5, duration=(gotodistance/ground_speed))

print("Move backward")
set_attitude(pitch_angle=5, thrust=0.5, duration=15)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

# Close the vehicle object before exiting the script
print("Close vehicle object")
vehicle.close()

print("Completed")

def odometry_callback(msg):
    # Extract the required data from the message
    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    position_z = msg.pose.pose.position.z
    orientation_x = msg.pose.pose.orientation.x
    orientation_y = msg.pose.pose.orientation.y
    orientation_z = msg.pose.pose.orientation.z
    orientation_w = msg.pose.pose.orientation.w
    timestamp = msg.header.stamp

    # Set the vehicle's current state
    vehicle.location.global_relative_frame.lat = position_x
    vehicle.location.global_relative_frame.lon = position_y
    vehicle.location.global_relative_frame.alt = position_z
    vehicle.attitude.roll = 0.0
    vehicle.attitude.pitch = 0.0
    vehicle.attitude.yaw = math.degrees(math.atan2(2.0 * (orientation_w * orientation_z + orientation_x * orientation_y),
                                                   1.0 - 2.0 * (orientation_y**2 + orientation_z**2)))
    print("Received ROS Odometry message:")
    print("Position: x=%.2f, y=%.2f, z=%.2f" % (position_x, position_y, position_z))
    print("Orientation: roll=%.2f, pitch=%.2f, yaw=%.2f" %
          (vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw))
    print("Timestamp: %s" % timestamp)

# Initialize the ROS node
rospy.init_node('dronekit_no_gps_control')

# Subscribe to the ROS Odometry topic
rospy.Subscriber('/odom', Odometry, odometry_callback)