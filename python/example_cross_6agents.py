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
        env_objs_TrafficSigns = world.get_environment_objects(carla.CityObjectLabel.TrafficSigns)
        env_objs_TrafficLight = world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
        env_objs_static = world.get_environment_objects(carla.CityObjectLabel.Static )
        env_objs_terrain = world.get_environment_objects(carla.CityObjectLabel.Terrain )

        objects_to_toggle = []
        # objects_to_toggle.append(env_objs_buildings[914].id) # tall building on the right
        # for i in np.arange(920,1087,1):
        #     objects_to_toggle.append(env_objs_buildings[i].id)
        # for i in np.arange(1,len(env_objs_Vegetation),1):
        #     objects_to_toggle.append(env_objs_Vegetation[i].id)
        # for i in np.arange(1,len(env_objs_TrafficSigns),1):
        #     objects_to_toggle.append(env_objs_TrafficSigns[i].id)
        # for i in np.arange(1,len(env_objs_static),1):
        #     objects_to_toggle.append(env_objs_static[i].id)
        # for i in np.arange(1,len(env_objs_terrain),1):
        #     objects_to_toggle.append(env_objs_terrain[i].id)
        # for i in np.arange(1,len(env_objs_TrafficLight),1):
        #     objects_to_toggle.append(env_objs_TrafficLight[i].id)

        # Toggle objects off
        world.enable_environment_objects(objects_to_toggle, False)
        # Toggle buildings on
        # world.enable_environment_objects(objects_to_toggle, True)

        # Toggle map layers off
        world.unload_map_layer(carla.MapLayer.Foliage)


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



        ################################# Waypoints ####################################

        waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)


        ######################### load path from plannar ##############

        ##############  testPresidentCar6Agents  #####################
        # x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents/fit/path_0_agent_0.txt')
        # x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents/fit/path_0_agent_1.txt')
        # x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents/fit/path_0_agent_2.txt')
        # x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents/fit/path_0_agent_3.txt')
        # x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents/fit/path_0_agent_4.txt')
        # x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents/fit/path_0_agent_5.txt')


        # x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents/fit/path_1_agent_0.txt')
        # x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents/fit/path_1_agent_1.txt')
        # x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents/fit/path_1_agent_2.txt')
        # x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents/fit/path_1_agent_3.txt')
        # x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents/fit/path_1_agent_4.txt')
        # x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents/fit/path_1_agent_5.txt')

        # x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents/fit/path_2_agent_0.txt')
        # x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents/fit/path_2_agent_1.txt')
        # x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents/fit/path_2_agent_2.txt')
        # x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents/fit/path_2_agent_3.txt')
        # x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents/fit/path_2_agent_4.txt')
        # x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents/fit/path_2_agent_5.txt')

        # x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents/fit/path_3_agent_0.txt')
        # x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents/fit/path_3_agent_1.txt')
        # x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents/fit/path_3_agent_2.txt')
        # x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents/fit/path_3_agent_3.txt')
        # x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents/fit/path_3_agent_4.txt')
        # x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents/fit/path_3_agent_5.txt')


        ##############  testPresidentCar6Agents 2 #####################
        # x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_0_agent_0.txt')
        # x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_0_agent_1.txt')
        # x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_0_agent_2.txt')
        # x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_0_agent_3.txt')
        # x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_0_agent_4.txt')
        # x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_0_agent_5.txt')


        # x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_1_agent_0.txt')
        # x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_1_agent_1.txt')
        # x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_1_agent_2.txt')
        # x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_1_agent_3.txt')
        # x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_1_agent_4.txt')
        # x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_1_agent_5.txt')

        # x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_2_agent_0.txt')
        # x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_2_agent_1.txt')
        # x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_2_agent_2.txt')
        # x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_2_agent_3.txt')
        # x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_2_agent_4.txt')
        # x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_2_agent_5.txt')

        # x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_3_agent_0.txt')
        # x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_3_agent_1.txt')
        # x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_3_agent_2.txt')
        # x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_3_agent_3.txt')
        # x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_3_agent_4.txt')
        # x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_3_agent_5.txt')

        x_list_1,y_list_1,a_list_1 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_4_agent_0.txt')
        x_list_2,y_list_2,a_list_2 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_4_agent_1.txt')
        x_list_3,y_list_3,a_list_3 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_4_agent_2.txt')
        x_list_4,y_list_4,a_list_4 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_4_agent_3.txt')
        x_list_5,y_list_5,a_list_5 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_4_agent_4.txt')
        x_list_6,y_list_6,a_list_6 = loadPath('./path_data/testPresidentCar6Agents_2/fit/path_4_agent_5.txt')

        #############################################################################
        ################### Manual Control the vehicle ##############################
        filtered_waypoints = []
        for waypoint in waypoints:
            if(waypoint.road_id == 20):
                filtered_waypoints.append(waypoint)
        # draw_waypoints(world, waypoints, road_id=[20], life_time=10)


        vehicle_type_list = ['vehicle.mini.cooper_s','vehicle.mini.cooper_s_2021','vehicle.micro.microlino',
                             'vehicle.citroen.c3','vehicle.audi.a2','vehicle.bmw.grandtourer',
                             'vehicle.jeep.wrangler_rubicon','vehicle.nissan.micra','vehicle.seat.leon',
                             'vehicle.toyota.prius']
        
        
        # vehicle_type_list = ['vehicle.nissan.micra', 'vehicle.micro.microlino']
        # vehicle_type_list = ['vehicle.nissan.micra']

        vehicle_type_list_president = ['vehicle.tesla.model3']


        # spawn the manual car 1
        ego_vehicle_bp_1 = blueprint_library.find(random.choice(vehicle_type_list_president))
        if ego_vehicle_bp_1.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_1.get_attribute('color').recommended_values)
            ego_vehicle_bp_1.set_attribute('color', '1, 1, 1')   
        spawn_points_manual_1 = filtered_waypoints[3].transform     
        spawn_points_manual_1.location.x = x_list_1[0]
        spawn_points_manual_1.location.y = y_list_1[0]
        spawn_points_manual_1.location.z += 2.6
        spawn_points_manual_1.rotation.yaw = a_list_1[0]
        ego_vehicle_manual_1 = world.spawn_actor(ego_vehicle_bp_1, spawn_points_manual_1)
        actor_list.append(ego_vehicle_manual_1)

        target_points_manual_1 = filtered_waypoints[60].transform
        target_points_manual_1.location.x = x_list_1[-1]
        target_points_manual_1.location.y = y_list_1[-1]



        # spawn the manual car 2
        ego_vehicle_bp_2 = blueprint_library.find(random.choice(vehicle_type_list_president))
        if ego_vehicle_bp_2.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_2.get_attribute('color').recommended_values)
            ego_vehicle_bp_2.set_attribute('color', '1, 1, 1')   
        spawn_points_manual_2 = filtered_waypoints[3].transform     
        spawn_points_manual_2.location.x = x_list_2[0]
        spawn_points_manual_2.location.y = y_list_2[0]
        spawn_points_manual_2.location.z += 0.6
        spawn_points_manual_2.rotation.yaw = a_list_2[0]
        ego_vehicle_manual_2 = world.spawn_actor(ego_vehicle_bp_2, spawn_points_manual_2)
        actor_list.append(ego_vehicle_manual_2)
        target_points_manual_2 = filtered_waypoints[60].transform
        target_points_manual_2.location.x = x_list_2[-1]
        target_points_manual_2.location.y = y_list_2[-1]


        # spawn the manual car 3
        ego_vehicle_bp_3 = blueprint_library.find(random.choice(vehicle_type_list_president))
        if ego_vehicle_bp_3.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_3.get_attribute('color').recommended_values)
            ego_vehicle_bp_3.set_attribute('color', '1, 1, 1')   
        spawn_points_manual_3 = filtered_waypoints[3].transform     
        spawn_points_manual_3.location.x = x_list_3[0]
        spawn_points_manual_3.location.y = y_list_3[0]
        spawn_points_manual_3.location.z += 0.6
        spawn_points_manual_3.rotation.yaw = a_list_3[0]
        ego_vehicle_manual_3 = world.spawn_actor(ego_vehicle_bp_3, spawn_points_manual_3)
        actor_list.append(ego_vehicle_manual_3)

        target_points_manual_3 = filtered_waypoints[60].transform
        target_points_manual_3.location.x = x_list_3[-1]
        target_points_manual_3.location.y = y_list_3[-1]


        # spawn the manual car 4
        ego_vehicle_bp_4 = blueprint_library.find(random.choice(vehicle_type_list))
        if ego_vehicle_bp_4.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_4.get_attribute('color').recommended_values)
            ego_vehicle_bp_4.set_attribute('color', color)   
        spawn_points_manual_4 = filtered_waypoints[3].transform     
        spawn_points_manual_4.location.x = x_list_4[0]
        spawn_points_manual_4.location.y = y_list_4[0]
        spawn_points_manual_4.location.z += 0.6
        spawn_points_manual_4.rotation.yaw = a_list_4[0]
        ego_vehicle_manual_4 = world.spawn_actor(ego_vehicle_bp_4, spawn_points_manual_4)
        actor_list.append(ego_vehicle_manual_4)

        target_points_manual_4 = filtered_waypoints[60].transform
        target_points_manual_4.location.x = x_list_4[-1]
        target_points_manual_4.location.y = y_list_4[-1]



        # spawn the manual car 5
        ego_vehicle_bp_5 = blueprint_library.find(random.choice(vehicle_type_list))
        if ego_vehicle_bp_5.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_5.get_attribute('color').recommended_values)
            ego_vehicle_bp_5.set_attribute('color', color)   
        spawn_points_manual_5 = filtered_waypoints[3].transform     
        spawn_points_manual_5.location.x = x_list_5[0]
        spawn_points_manual_5.location.y = y_list_5[0]
        spawn_points_manual_5.location.z += 0.6
        spawn_points_manual_5.rotation.yaw = a_list_5[0]    
        ego_vehicle_manual_5 = world.spawn_actor(ego_vehicle_bp_5, spawn_points_manual_5)
        actor_list.append(ego_vehicle_manual_5)

        target_points_manual_5 = filtered_waypoints[60].transform
        target_points_manual_5.location.x = x_list_5[-1]
        target_points_manual_5.location.y = y_list_5[-1]


        # # spawn the manual car 6
        ego_vehicle_bp_6 = blueprint_library.find(random.choice(vehicle_type_list))
        if ego_vehicle_bp_6.has_attribute('color'):
            color = random.choice(ego_vehicle_bp_6.get_attribute('color').recommended_values)
            ego_vehicle_bp_6.set_attribute('color', color)   
        spawn_points_manual_6 = filtered_waypoints[3].transform     
        spawn_points_manual_6.location.x = x_list_6[0]
        spawn_points_manual_6.location.y = y_list_6[0]
        spawn_points_manual_6.location.z += 0.6
        spawn_points_manual_6.rotation.yaw = a_list_6[0]
        ego_vehicle_manual_6 = world.spawn_actor(ego_vehicle_bp_6, spawn_points_manual_6)
        actor_list.append(ego_vehicle_manual_6)

        target_points_manual_6 = filtered_waypoints[60].transform
        target_points_manual_6.location.x = x_list_6[-1]
        target_points_manual_6.location.y = y_list_6[-1]


        ###########################################################################################
        ###########################################################################################

        # settings = world.get_settings()
        # settings.synchronous_mode = True
        # world.apply_settings(settings)

        i = 0
        j = 0

        while True:


            world.tick()
            
            # set the sectator to follow the ego vehicle
            spectator = world.get_spectator()
            # following_transform = ego_vehicle.get_transform()
            # spectator.set_transform(carla.Transform(transform.location + carla.Location(x=0,y=0,z=20),
            #                                         carla.Rotation(yaw=90, pitch=-90)))
            # spectator.set_transform(carla.Transform(spawn_transform.location + carla.Location(x=35,y=0,z=70),
            #                                         carla.Rotation(yaw=-180, pitch=-55)))

            spectator.set_transform(carla.Transform(spawn_transform.location + carla.Location(x=10,y=0,z=135),
                                                    carla.Rotation(yaw=-180, pitch=-75)))



            # ##################### Manually set position ####################### 
 
            if j > 200:

                trans_1 = ego_vehicle_manual_1.get_transform()
                if i>=len(x_list_1):
                    trans_1.location.x = x_list_1[-1]
                    trans_1.location.y = y_list_1[-1]
                    trans_1.rotation.yaw = a_list_1[-1]
                else:
                    trans_1.location.x = x_list_1[i]
                    trans_1.location.y = y_list_1[i]
                    trans_1.rotation.yaw = a_list_1[i]
                ego_vehicle_manual_1.set_transform(trans_1)

                trans_2 = ego_vehicle_manual_2.get_transform()
                if i>=len(x_list_2):
                    trans_2.location.x = x_list_2[-1]
                    trans_2.location.y = y_list_2[-1]
                    trans_2.rotation.yaw = a_list_2[-1]
                else:
                    trans_2.location.x = x_list_2[i]
                    trans_2.location.y = y_list_2[i]
                    trans_2.rotation.yaw = a_list_2[i]
                ego_vehicle_manual_2.set_transform(trans_2)

                trans_3 = ego_vehicle_manual_3.get_transform()
                if i>=len(x_list_3):
                    trans_3.location.x = x_list_3[-1]
                    trans_3.location.y = y_list_3[-1]
                    trans_3.rotation.yaw = a_list_3[-1]
                else:
                    trans_3.location.x = x_list_3[i]
                    trans_3.location.y = y_list_3[i]
                    trans_3.rotation.yaw = a_list_3[i]
                ego_vehicle_manual_3.set_transform(trans_3)

                trans_4 = ego_vehicle_manual_4.get_transform()
                if i>=len(x_list_4):
                    trans_4.location.x = x_list_4[-1]
                    trans_4.location.y = y_list_4[-1]
                    trans_4.rotation.yaw = a_list_4[-1]
                else:
                    trans_4.location.x = x_list_4[i]
                    trans_4.location.y = y_list_4[i]
                    trans_4.rotation.yaw = a_list_4[i]
                ego_vehicle_manual_4.set_transform(trans_4)


                trans_4 = ego_vehicle_manual_4.get_transform()
                if i>=len(x_list_4):
                    trans_4.location.x = x_list_4[-1]
                    trans_4.location.y = y_list_4[-1]
                    trans_4.rotation.yaw = a_list_4[-1]
                else:
                    trans_4.location.x = x_list_4[i]
                    trans_4.location.y = y_list_4[i]
                    trans_4.rotation.yaw = a_list_4[i]
                ego_vehicle_manual_4.set_transform(trans_4)

                trans_5 = ego_vehicle_manual_5.get_transform()
                if i>=len(x_list_5):
                    trans_5.location.x = x_list_5[-1]
                    trans_5.location.y = y_list_5[-1]
                    trans_5.rotation.yaw = a_list_5[-1]
                else:
                    trans_5.location.x = x_list_5[i]
                    trans_5.location.y = y_list_5[i]
                    trans_5.rotation.yaw = a_list_5[i]
                ego_vehicle_manual_5.set_transform(trans_5)

                trans_6 = ego_vehicle_manual_6.get_transform()
                if i>=len(x_list_6):
                    trans_6.location.x = x_list_6[-1]
                    trans_6.location.y = y_list_6[-1]
                    trans_6.rotation.yaw = a_list_6[-1]
                else:
                    trans_6.location.x = x_list_6[i]
                    trans_6.location.y = y_list_6[i]
                    trans_6.rotation.yaw = a_list_6[i]
                ego_vehicle_manual_6.set_transform(trans_6)


                i += 1
            else:
                j += 1




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

