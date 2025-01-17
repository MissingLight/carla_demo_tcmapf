"""
In this script, we are going to learn how to spawn a vehicle on the road and make it autopilot.
At the same time, we will collect camera and lidar data from it.
"""

import carla
import os
import random
import numpy as np

from agents.navigation.controller import VehiclePIDController
from agents.navigation.behavior_agent import BehaviorAgent

def draw_waypoints(world, waypoints, road_id=None, life_time=50.0):
#   id = None
#   for waypoint in waypoints:
#     if id == None:
#         id = waypoint.road_id
#     if waypoint.road_id != id:
#         id = waypoint.road_id
#         print(id)

    # f = open('waypoints.txt', 'w')
    for waypoint in waypoints:
        if(waypoint.road_id in road_id):
            world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                    color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                    persistent_lines=True)
            # print('x: ', waypoint.transform.location.x, 'y: ',waypoint.transform.location.y)            
            # f.write(str(waypoint.transform.location.x) + '    ' + str(waypoint.transform.location.y) + '\n')
    # f.close()           

def loadPath(path):
    x_list = []
    y_list = []
    a_list = []
    file = open(path)
    for eachLine in file:
        x = float(eachLine.split()[0])
        y = float(eachLine.split()[1]) 
        a = float(eachLine.split()[2])
        x_list.append(x)
        y_list.append(y)
        a_list.append(a)
    return x_list,y_list,a_list


def main():
    actor_list = []
    sensor_list = []

    try:
        # First of all, we need to create the client that will send the requests, assume port is 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        
        
        ############################################################################
        ############################ Initiate the environment ####################

        # print(client.get_available_maps()) 
        # Retrieve the world that is currently running
        # world = client.get_world()
        world = client.load_world('Town10HD_Opt') # you can also retrive another world by specifically defining
        print(world.get_map())
        # world.unload_map_layer(carla.MapLayer.Buildings) # 隐藏所有层/Buildings

        env_objs_buildings = world.get_environment_objects(carla.CityObjectLabel.Buildings)
        env_objs_Vegetation = world.get_environment_objects(carla.CityObjectLabel.Vegetation)
        env_objs_roads = world.get_environment_objects(carla.CityObjectLabel.Roads)

        # InstancedFoliageActor
        # Access individual building IDs and save in a set
        # print(len(env_objs_buildings))
        # print(len(env_objs_roads))

        objects_to_toggle = []
        objects_to_toggle.append(env_objs_buildings[914].id) # tall building on the right
        for i in np.arange(920,1087,1):
            objects_to_toggle.append(env_objs_buildings[i].id)

        for i in np.arange(1,len(env_objs_Vegetation),1):
            objects_to_toggle.append(env_objs_Vegetation[i].id)


        # building_01 = env_objs_buildings[500]
        # building_02 = env_objs_buildings[600]
        # objects_to_toggle = {building_01.id, building_02.id}

        # Toggle buildings off
        world.enable_environment_objects(objects_to_toggle, False)
        # Toggle buildings on
        # world.enable_environment_objects(objects_to_toggle, True)


        blueprint_library = world.get_blueprint_library()
        # Set weather for your world
        weather = carla.WeatherParameters(cloudiness=1.0,
                                          precipitation=10.0,
                                          fog_density=1.0,
                                          sun_altitude_angle=90.0)
        world.set_weather(weather)


        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        world.tick()

        ############################################################################
        ############################  Create and spawn vehicles ####################
        # create the ego vehicle
        # ego_vehicle_bp = blueprint_library.find('vehicle.mercedes-benz.coupe')
        ego_vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
        # set color for the car
        if ego_vehicle_bp.has_attribute('color'):
            color = random.choice(ego_vehicle_bp.get_attribute('color').recommended_values)
            ego_vehicle_bp.set_attribute('color', color)

        # get a random valid occupation in the world
        spawn_points = world.get_map().get_spawn_points()
        # transform = random.choice(spawn_points)
        spawn_transform = spawn_points[147]
        # print('transform',transform)
        # # spawn the vehilce
        # ego_vehicle = world.spawn_actor(ego_vehicle_bp, spawn_transform)
        # # # set the vehicle autopilot mode
        # ego_vehicle.set_autopilot(True)
        # # collect all actors to destroy when we quit the script
        # actor_list.append(ego_vehicle)

        # # spawn the second car
        # ego_vehicle_bp2 = random.choice(blueprint_library.filter('vehicle'))
        # if ego_vehicle_bp2.has_attribute('color'):
        #     color = random.choice(ego_vehicle_bp2.get_attribute('color').recommended_values)
        #     ego_vehicle_bp2.set_attribute('color', color)        
        # ego_vehicle2 = world.spawn_actor(ego_vehicle_bp2, spawn_points[148])
        # ego_vehicle2.set_autopilot(True)
        # actor_list.append(ego_vehicle2)




        # for i in range(100):
        #     current_loc = ego_vehicle.get_location()
        #     next_loc = current_loc
        #     next_loc.x += 1.0
        #     ego_vehicle.set_location(next_loc)


        ############################# Add sensors ####################################

        # # add a camera
        # camera_bp = blueprint_library.find('sensor.camera.rgb')
        # # camera relative position related to the vehicle
        # camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)

        # output_path = '../outputs/output_basic_api'
        # if not os.path.exists(output_path):
        #     os.makedirs(output_path)

        # # set the callback function
        # camera.listen(lambda image: image.save_to_disk(os.path.join(output_path, '%06d.png' % image.frame)))
        # sensor_list.append(camera)

        # # we also add a lidar on it
        # lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        # lidar_bp.set_attribute('channels', str(32))
        # lidar_bp.set_attribute('points_per_second', str(90000))
        # lidar_bp.set_attribute('rotation_frequency', str(40))
        # lidar_bp.set_attribute('range', str(20))

        # # set the relative location
        # lidar_location = carla.Location(0, 0, 2)
        # lidar_rotation = carla.Rotation(0, 0, 0)
        # lidar_transform = carla.Transform(lidar_location, lidar_rotation)

        # # spawn the lidar
        # lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        # lidar.listen(
        #     lambda point_cloud: point_cloud.save_to_disk(os.path.join(output_path, '%06d.ply' % point_cloud.frame)))
        # sensor_list.append(lidar)




        ################################# Waypoints ####################################

        waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)

        # draw_waypoints(world, waypoints, road_id=[0,1,2,3,4,5,6,7,8,9,10,11,12], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[13,14,15,16,17,18,19,20,21,22], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[24,33,41,89,90,100,105,125,150,151,152,160,172,179], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[254,255,256,268,286,296,315,338,344,375,382,395,466,467,479], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[472,476,486,515,516,565,566,579,584,597,608,630,637,655,675], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[676,703,712,735,736,763,780,787,795,801,811,823,832,843,848], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[875,876,900,933,934], life_time=20)


        # draw_waypoints(world, waypoints, road_id=[13,14,15,16,18,19,20,21,22], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[254,255,256,268,286,296,315,338,344,375,382,395,466,467], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[875,876], life_time=20)
        # draw_waypoints(world, waypoints, road_id=[933,934], life_time=20)

        ######################### load path from plannar ##############
        x_list_1,y_list_1,a_list_1 = loadPath('./path_data/path_0_agent_0_interp.txt')
        x_list_2,y_list_2,a_list_2 = loadPath('./path_data/path_0_agent_1_interp.txt')
        x_list_3,y_list_3,a_list_3 = loadPath('./path_data/path_0_agent_2_interp.txt')
        x_list_4,y_list_4,a_list_4 = loadPath('./path_data/path_0_agent_3_interp.txt')
        x_list_5,y_list_5,a_list_5 = loadPath('./path_data/path_0_agent_4_interp.txt')
        x_list_6,y_list_6,a_list_6 = loadPath('./path_data/path_0_agent_5_interp.txt')


        #############################################################################
        ################### Manual Control the vehicle ##############################
        filtered_waypoints = []
        for waypoint in waypoints:
            if(waypoint.road_id == 20):
                filtered_waypoints.append(waypoint)
        draw_waypoints(world, waypoints, road_id=[20], life_time=10)


        # spawn the manual car 1
        ego_vehicle_bp_1 = random.choice(blueprint_library.filter('vehicle'))
        if ego_vehicle_bp_1.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_1.get_attribute('color').recommended_values)
            ego_vehicle_bp_1.set_attribute('color', color)   
        spawn_points_manual_1 = filtered_waypoints[3].transform     
        spawn_points_manual_1.location.x = x_list_1[0]
        spawn_points_manual_1.location.y = y_list_1[0]
        spawn_points_manual_1.location.z += 2.6
        ego_vehicle_manual_1 = world.spawn_actor(ego_vehicle_bp_1, spawn_points_manual_1)
        actor_list.append(ego_vehicle_manual_1)

        target_points_manual_1 = filtered_waypoints[60].transform
        target_points_manual_1.location.x = x_list_1[-1]
        target_points_manual_1.location.y = y_list_1[-1]


        world.debug.draw_string(target_points_manual_1.location, 'O', draw_shadow=False,
                                    color=carla.Color(r=255, g=0, b=0), life_time=10,
                                    persistent_lines=True)
        
        print('start: ', spawn_points_manual_1.location)
        print('target: ', spawn_points_manual_1.location)




        # spawn the manual car 2
        ego_vehicle_bp_2 = random.choice(blueprint_library.filter('vehicle'))
        if ego_vehicle_bp_2.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_2.get_attribute('color').recommended_values)
            ego_vehicle_bp_2.set_attribute('color', color)   
        spawn_points_manual_2 = filtered_waypoints[3].transform     
        spawn_points_manual_2.location.x = x_list_2[0]
        spawn_points_manual_2.location.y = y_list_2[0]
        spawn_points_manual_2.location.z += 0.6
        ego_vehicle_manual_2 = world.spawn_actor(ego_vehicle_bp_2, spawn_points_manual_2)
        actor_list.append(ego_vehicle_manual_2)
        target_points_manual_2 = filtered_waypoints[60].transform
        target_points_manual_2.location.x = x_list_2[-1]
        target_points_manual_2.location.y = y_list_2[-1]


        # spawn the manual car 3
        ego_vehicle_bp_3 = random.choice(blueprint_library.filter('vehicle'))
        if ego_vehicle_bp_3.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_3.get_attribute('color').recommended_values)
            ego_vehicle_bp_3.set_attribute('color', color)   
        spawn_points_manual_3 = filtered_waypoints[3].transform     
        spawn_points_manual_3.location.x = x_list_3[0]
        spawn_points_manual_3.location.y = y_list_3[0]
        spawn_points_manual_3.location.z += 0.6
        ego_vehicle_manual_3 = world.spawn_actor(ego_vehicle_bp_3, spawn_points_manual_3)
        actor_list.append(ego_vehicle_manual_3)

        target_points_manual_3 = filtered_waypoints[60].transform
        target_points_manual_3.location.x = x_list_3[-1]
        target_points_manual_3.location.y = y_list_3[-1]


        # spawn the manual car 4
        ego_vehicle_bp_4 = random.choice(blueprint_library.filter('vehicle'))
        if ego_vehicle_bp_4.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_4.get_attribute('color').recommended_values)
            ego_vehicle_bp_4.set_attribute('color', color)   
        spawn_points_manual_4 = filtered_waypoints[3].transform     
        spawn_points_manual_4.location.x = x_list_4[0]
        spawn_points_manual_4.location.y = y_list_4[0]
        spawn_points_manual_4.location.z += 0.6
        ego_vehicle_manual_4 = world.spawn_actor(ego_vehicle_bp_4, spawn_points_manual_4)
        actor_list.append(ego_vehicle_manual_4)

        target_points_manual_4 = filtered_waypoints[60].transform
        target_points_manual_4.location.x = x_list_4[-1]
        target_points_manual_4.location.y = y_list_4[-1]



        # spawn the manual car 5
        ego_vehicle_bp_5 = random.choice(blueprint_library.filter('vehicle'))
        if ego_vehicle_bp_5.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_5.get_attribute('color').recommended_values)
            ego_vehicle_bp_5.set_attribute('color', color)   
        spawn_points_manual_5 = filtered_waypoints[3].transform     
        spawn_points_manual_5.location.x = x_list_5[0]
        spawn_points_manual_5.location.y = y_list_5[0]
        spawn_points_manual_5.location.z += 0.6
        ego_vehicle_manual_5 = world.spawn_actor(ego_vehicle_bp_5, spawn_points_manual_5)
        actor_list.append(ego_vehicle_manual_5)

        target_points_manual_5 = filtered_waypoints[60].transform
        target_points_manual_5.location.x = x_list_5[-1]
        target_points_manual_5.location.y = y_list_5[-1]


        # # spawn the manual car 6
        # ego_vehicle_bp_6 = random.choice(blueprint_library.filter('vehicle'))
        # if ego_vehicle_bp_6.has_attribute('color'):
        #     color = random.choice(ego_vehicle_bp_6.get_attribute('color').recommended_values)
        #     ego_vehicle_bp_6.set_attribute('color', color)   
        # spawn_points_manual_6 = filtered_waypoints[3].transform     
        # spawn_points_manual_6.location.x = x_list_5[0]
        # spawn_points_manual_6.location.y = y_list_5[0]
        # spawn_points_manual_6.location.z += 0.6
        # ego_vehicle_manual_6 = world.spawn_actor(ego_vehicle_bp_6, spawn_points_manual_6)
        # actor_list.append(ego_vehicle_manual_6)

        # target_points_manual_6 = filtered_waypoints[60].transform
        # target_points_manual_6.location.x = x_list_6[-1]
        # target_points_manual_6.location.y = y_list_6[-1]



        # ego_vehicle_bpmanual2 = random.choice(blueprint_library.filter('vehicle'))
        # if ego_vehicle_bpmanual2.has_attribute('color'):
        #     color = random.choice(ego_vehicle_bpmanual2.get_attribute('color').recommended_values)
        #     ego_vehicle_bpmanual2.set_attribute('color', color)   
        # spawn_points_manual2 = filtered_waypoints[2].transform     
        # spawn_points_manual2.location.z += 0.6
        # ego_vehicle_manual2 = world.spawn_actor(ego_vehicle_bpmanual2, spawn_points_manual2)
        # actor_list.append(ego_vehicle_manual2)

        # target_points_manual2 = filtered_waypoints[61].transform



        # ticks_to_track = 10
        # for i in range(ticks_to_track):
        #     control_signal = custom_controller.run_step(1, target_points_manual)
        #     ego_vehicle_manual.apply_control(control_signal)
        # behaviour_agent = BehaviorAgent(ego_vehicle_manual, behavior='normal')
        # behaviour_agent.set_destination(behaviour_agent.vehicle.get_location(), target_points_manual.location, clean=True)


        ###########################################################################################
        ###########################################################################################

        # settings = world.get_settings()
        # settings.synchronous_mode = True
        # world.apply_settings(settings)

        i = 0
        

        while True:
            # print('Getting into the loop')
            # behaviour_agent.update_information(ego_vehicle_manual)
            # print('len(agent._local_planner.waypoints_queue)', len(behaviour_agent._local_planner.waypoints_queue))
            # print('incoming_waypoint', behaviour_agent.incoming_waypoint)


            world.tick()
            
            # set the sectator to follow the ego vehicle
            spectator = world.get_spectator()
            # following_transform = ego_vehicle.get_transform()
            # spectator.set_transform(carla.Transform(transform.location + carla.Location(x=0,y=0,z=20),
            #                                         carla.Rotation(yaw=90, pitch=-90)))
            # spectator.set_transform(carla.Transform(spawn_transform.location + carla.Location(x=40,y=0,z=40),
            #                                         carla.Rotation(yaw=-180, pitch=-20)))

            spectator.set_transform(carla.Transform(spawn_transform.location + carla.Location(x=-30,y=0,z=70),
                                                    carla.Rotation(yaw=-180, pitch=-90)))


            # spectator.set_transform(carla.Transform(spawn_transform.location + carla.Location(x=20,y=0,z=70),
            #                                         carla.Rotation(yaw=-180, pitch=-90)))  # straight view on the top of cross road


            
            if ego_vehicle_manual_1.get_transform().location.x > target_points_manual_1.location.x:
                print('======== Success, Arrivied at Target Point!')
                ego_vehicle_manual_1.destroy()
                break

            # ##################### Manually set position ####################### 
            # current_loc = ego_vehicle_manual.get_location()
            # next_loc = current_loc
            # next_loc.x -= 0.10
            # ego_vehicle_manual.set_location(next_loc)
            trans_1 = ego_vehicle_manual_1.get_transform()
            trans_1.location.x = x_list_1[i]
            trans_1.location.y = y_list_1[i]
            trans_1.rotation.yaw = a_list_1[i]
            ego_vehicle_manual_1.set_transform(trans_1)

            trans_2 = ego_vehicle_manual_2.get_transform()
            trans_2.location.x = x_list_2[i]
            trans_2.location.y = y_list_2[i]
            trans_2.rotation.yaw = a_list_2[i]
            ego_vehicle_manual_2.set_transform(trans_2)

            trans_3 = ego_vehicle_manual_3.get_transform()
            trans_3.location.x = x_list_3[i]
            trans_3.location.y = y_list_3[i]
            trans_3.rotation.yaw = a_list_3[i]
            ego_vehicle_manual_3.set_transform(trans_3)

            trans_4 = ego_vehicle_manual_4.get_transform()
            trans_4.location.x = x_list_4[i]
            trans_4.location.y = y_list_4[i]
            trans_4.rotation.yaw = a_list_4[i]
            ego_vehicle_manual_4.set_transform(trans_4)
            
            trans_5 = ego_vehicle_manual_5.get_transform()
            trans_5.location.x = x_list_5[i]
            trans_5.location.y = y_list_5[i]
            trans_5.rotation.yaw = a_list_5[i]
            ego_vehicle_manual_5.set_transform(trans_5)

            # trans_6 = ego_vehicle_manual_6.get_transform()
            # trans_6.location.x = x_list_6[i]
            # trans_6.location.y = y_list_6[i]
            # trans_6.rotation.yaw = a_list_6[i]
            # ego_vehicle_manual_5.set_transform(trans_6)


            i+=2

            # current_loc2 = ego_vehicle_manual2.get_location()
            # next_loc2 = current_loc2
            # next_loc2.x -= 0.10
            # next_loc2.y += 0.10
            # ego_vehicle_manual2.set_location(next_loc2)


            # ##################### Automatic controller ####################### 
            # speed_limit = ego_vehicle_manual.get_speed_limit()
            # behaviour_agent.get_local_planner().set_speed(speed_limit/10)

            # control = behaviour_agent.run_step(debug=True)
            # ego_vehicle_manual.apply_control(control)


    finally:
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')

