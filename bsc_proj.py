
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import math
import random
import numpy as np
import pygame
import time

# Set up locations of the used roads and intersections as global variables to be used by the modules
ROAD_1 = [[48.99847763416068, 8.00006947575974], [48.99809419046005, 8.000087148841695]]
ROAD_2 = [[48.99805428354699, 8.000128508513056], [48.99806604464851, 8.000806295740245]]
ROAD_3 = [[48.998119354141096, 8.00088717637275], [48.99846495693862, 8.000879261801801]]
ROAD_4 = [[48.998536194048654, 8.000802017482261], [48.998526519082134, 8.000174687198811]]
INTERSECTION_1 = [[48.99809013722182, 8.000139335563482], [48.998000811959145, 8.000029638190817]]
INTERSECTION_2 = [ROAD_2[1], ROAD_3[0]]
INTERSECTION_3 = [ROAD_3[1], ROAD_4[0]]
INTERSECTION_4 = [ROAD_4[1], ROAD_1[0]]

ROAD_WIDTH = 0.000068 # Used to determine whether the car is close enough to the middle of the road or not


# Help function in the development
def print_location(data):
    print(data.latitude, data.longitude)


# Initialise the situation by spawning the car and GNSS sensor at a set location

def set_up():
    try:

        # ...
        client = carla.Client("localhost", 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        spectator = world.get_spectator()
        bp_library = world.get_blueprint_library()
        map = world.get_map()

        vehicle_bp = random.choice(bp_library.filter('vehicle.bmw.grandtourer'))
        print(vehicle_bp)
        transform = carla.Transform(carla.Location(87.1881,-86.9629,9.84228),carla.Rotation(0,-90,0))
        transform_spec = carla.Transform(spectator.get_location(),carla.Rotation(0,-90,0))
        vehicle = world.try_spawn_actor(vehicle_bp, transform)

        # Wait for world to get the vehicle actor
        world.tick()

        spec_transform = transform   # spec_transform location defined for spectator position
        spec_transform.location.z += 1
        spec_transform.location.y += 5

        # Set spectator at given transform (vehicle transform)
        spectator.set_transform(spec_transform)
        #vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))

        world.tick()

        # Create gnss sensor actor on top of the vehicle (vehicle transform).
        gnss_bp = random.choice(bp_library.filter('gnss'))
        gnss = world.try_spawn_actor(gnss_bp, transform, attach_to=vehicle)


        # ...
        return vehicle, gnss

    finally:
        pass

# Selects the correct behaviour depending on if the vehicle is on road or in an intersection.
# Determines the situation by comparing the location to the sets road_points and intersection_points.
def behaviour_selection(data, car):
    location = [data.latitude, data.longitude]
    print(location)

    # Check if the car is on any of the locations marked road and mark which road.
    on_road = False
    in_intersection = False

    if ROAD_1[0][0] >= location[0] >= ROAD_1[1][0] and ROAD_1[0][1]-ROAD_WIDTH <= location[1] <= ROAD_1[1][1]+ROAD_WIDTH:
        road_num = 1
        on_road = True
    elif ROAD_2[0][1] <= location[1] <= ROAD_2[1][1] and ROAD_2[0][0]-ROAD_WIDTH <= location[0] <= ROAD_2[0][0]+ROAD_WIDTH:
        road_num = 2
        on_road = True
    elif ROAD_3[0][0] <= location[0] <= ROAD_3[1][0] and ROAD_3[0][1]-ROAD_WIDTH <= location[1] <= ROAD_3[1][1]+ROAD_WIDTH:
        road_num = 3
        on_road = True
    elif ROAD_4[0][0] <= location[0] <= ROAD_4[1][0] and ROAD_4[0][0]-ROAD_WIDTH <= location[1] <= ROAD_4[1][0]+ROAD_WIDTH:
        road_num = 4
        on_road = True

    # If the car is on road, call the road-module
    if on_road:
        road(location, car, road_num)

    # If the car is not on road, determine if it is in an intersection

    elif INTERSECTION_1[0][0] >= location[0] >= INTERSECTION_1[1][0] and INTERSECTION_1[0][1] >= location[1] >= INTERSECTION_1[1][1]:
        in_intersection = True
        intersection_num = 1

    elif INTERSECTION_2[0][0] <= location[0] <= INTERSECTION_2[0][0] and INTERSECTION_2[1][1] <= location[1] <= INTERSECTION_2[1][1]:
        in_intersection = True
        intersection_num = 2

    elif INTERSECTION_3[1][0] <= location[0] <= INTERSECTION_3[1][0] and INTERSECTION_3[0][1] <= location[1] <= INTERSECTION_3[0][1]:
        in_intersection = True
        intersection_num = 3

    elif INTERSECTION_4[0][1] <= location[0] <= INTERSECTION_4[0][1] and INTERSECTION_4[1][0] <= location[1] <= INTERSECTION_4[1][0]:
        in_intersection = True
        intersection_num = 4

    if in_intersection:
        intersection(location, car, intersection_num)

    if not in_intersection and not on_road:
        print("Car is out of bounds.")


def road(position, car, road_name):
    # Determine if the car is too far, to the left or to the right, from the middle of the lane (closer to the lane marking than the middle)
    left = False
    right = False

    if road_name == 1:
        print("On road 1")
        # Calculate the distance from the center off the lane using general line form and distance of point.
        a = (ROAD_1[1][1]-ROAD_1[0][1])*position[0]
        b = (ROAD_1[1][0]-ROAD_1[0][0])*position[1]
        c = ROAD_1[1][0]*ROAD_1[0][1]-ROAD_1[1][1]*ROAD_1[0][0]
        #print(a,b,c)
        off_center = abs(a-b+c)/((ROAD_1[1][1]-ROAD_1[0][1])**2+(ROAD_1[1][0]-ROAD_1[0][0])**2)**(1/2)
        #print(off_center)

        if off_center > ROAD_WIDTH/100: # Check if the car is too much off the center of the lane.
            if a-b+c < 0:
                print("Steering right")
                car.apply_control(carla.VehicleControl(throttle=0.5, steer=0.3))

            elif a-b+c > 0: # Check if the is too much to the right on the road.
                print("Steering left")
                car.apply_control(carla.VehicleControl(throttle=0.5, steer=-0.3))

        else:
            car.apply_control(carla.VehicleControl(throttle=1, steer=-0.0))

    if road_name == 2:
        print("On road 2")
        # Calculate the distance from the center off the lane using general line form and distance of point.
        a = (ROAD_2[1][1]-ROAD_2[0][1])*position[0]
        b = (ROAD_2[1][0]-ROAD_2[0][0])*position[1]
        c = ROAD_2[1][0]*ROAD_2[0][1]-ROAD_2[1][1]*ROAD_2[0][0]
        #print(a,b,c)
        off_center = abs(a-b+c)/((ROAD_2[1][1]-ROAD_2[0][1])**2+(ROAD_2[1][0]-ROAD_2[0][0])**2)**(1/2)
        #print(off_center)

        if off_center > ROAD_WIDTH/100: # Check if the car is too much off the center of the lane.
            if a-b+c > 0:
                print("Steering right")
                car.apply_control(carla.VehicleControl(throttle=0.5, steer=0.3))

            elif a-b+c < 0: # Check if the is too much to the right on the road.
                print("Steering left")
                car.apply_control(carla.VehicleControl(throttle=0.5, steer=-0.3))

        else:
            car.apply_control(carla.VehicleControl(throttle=1, steer=-0.0))


def intersection(position, car, intersection_num):
    print("Turning right from the intersection")
    car.apply_control(carla.VehicleControl(throttle=0.2, steer=1))

def main():
    actor_list = []

    car, gnss = set_up() #Initialise the situation
    actor_list.append(car)
    actor_list.append(gnss)

    gnss.listen(lambda data: behaviour_selection(data, car)) # Calls for behaviour selection on every game tick

    time.sleep(25) # Keeps the script running for a set amount of time

    # Remove the actors from the simulation to free memory
    for actor in actor_list:
        actor.destroy()
    print("done")

main()