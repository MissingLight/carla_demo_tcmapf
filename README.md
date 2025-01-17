
# Carla Simulator

Tgus repository is original from a Zhihu blog:

--[Carla Learning Tutorial](https://www.zhihu.com/column/c_1324712096148516864)

## Direct Blog Links
[Tutorial 1: An Overview of the Carla Concept](https://zhuanlan.zhihu.com/p/338641593) \
[Tutorial 2: Install Carla](https://zhuanlan.zhihu.com/p/338927297) \
[Tutorial 3: Use Basic API](https://zhuanlan.zhihu.com/p/340031078) \
[Tutorial 4: Synchronize Mode](https://zhuanlan.zhihu.com/p/340031078) \
[Tutorial 5: Traffic Manager](https://zhuanlan.zhihu.com/p/346636395) \
[Tutorial 6: Behavior Planning(Part1) ](https://zhuanlan.zhihu.com/p/355420522) \
[Tutorial 7: Behavior Planning(Part2) ](https://zhuanlan.zhihu.com/p/376411890) 

## Installing Carla
Make sure your device have at least 100G free space, the unreal engine takes a lot of space.


## Using Carla for TC-MAPF simulation

Using `crossroad_waypoints.py` to fit the discrete waypoints of TC-CBS output. The data will be saved to `/path/data`. Be careful that I align the map coordinate in the Carla city to the coordinate of our grid map.

Using `example_cross_xxxxx.py` to run the simulation. In Carla simulator, press the play button and then run the code in the terminal. `basic_api.py` provides a very simple introduction of running the simualtion, where you can enable or disable any building or element in the city envrionment. You can set the weather, and even plot the planned path on the city. These examples should be self-contained. Following them will help you know how to make a environment and make cars move.

Make sure to refer to Carla's [official document](https://carla.readthedocs.io/en/latest/). The key is to understand how to spawn a vehicle, and then set its pose, which is easy.