# -*- coding: utf-8 -*-

"""Revised automatic control
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os
import random
import sys

import carla

from agents.navigation.behavior_agent import BehaviorAgent


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()

        origin_settings = world.get_settings()

        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        weather = carla.WeatherParameters(cloudiness=1.0,
                                          precipitation=10.0,
                                          fog_density=1.0,
                                          sun_altitude_angle=90.0)
        world.set_weather(weather)


        waypoints = client.get_world().get_map().generate_waypoints(distance=1)
        filtered_waypoints = []
        for waypoint in waypoints:
            if(waypoint.road_id == 20):
                filtered_waypoints.append(waypoint)

        # read all valid spawn points
        all_default_spawn = world.get_map().get_spawn_points()
        # randomly choose one as the start point
        spawn_point = random.choice(all_default_spawn) if all_default_spawn else carla.Transform()
        # spawn_point = filtered_waypoints[0].transform 
        # spawn_point.location.z += 0.6
        # print('spawn_point: ', spawn_point.location)

        # create the blueprint library
        # ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        ego_vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # spawn the vehicle
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)

        # we need to tick the world once to let the client update the spawn position
        world.tick()

        # create the behavior agent
        agent = BehaviorAgent(vehicle, behavior='normal')

        # set the destination spot
        spawn_points = world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        # to avoid the destination and start position same
        if spawn_points[0].location != agent.vehicle.get_location():
            destination = spawn_points[0]
        else:
            destination = spawn_points[1]

        # destination = filtered_waypoints[40].transform 
        # destination.location.z += 0.6
        # print('destination: ', destination.location)



        # generate the route
        agent.set_destination(agent.vehicle.get_location(), destination.location, clean=True)

        while True:
            agent.update_information(vehicle)
            print('len(agent._local_planner.waypoints_queue)', len(agent._local_planner.waypoints_queue))

            world.tick()
            
            if len(agent._local_planner.waypoints_queue)<1:
                print('======== Success, Arrivied at Target Point!')
                vehicle.destroy()
                break
                
            # top view
            spectator = world.get_spectator()
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                    carla.Rotation(pitch=-90)))

            speed_limit = vehicle.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)

            control = agent.run_step(debug=True)
            vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        # vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
