# topology_map
navigation based on topo map


# BASIC STEPS

install ROS
```
cd catkin_ws/src

git clone

cd ..

catkin_make --pkg topology_map
```

the mapping node:
```
rosrun topology_map main
```

the UI:
```
rosrun topology_map TopoUI
```

save the building map:
$rosrun topology_map saveMapClient.py

the map files (saved, built, simulation truth) are default in /home/${USER_NAME}/topoMaps/

in the read mode, you can read map file
in the build mode, you can build map
in the simulation mode, you can use the map you build in the second mode to simulate the input
in the realtime mode, you can fetch the building map from the main node


the mapping node:
$rosrun topology_map main
$rosrun topology_map TopoUI

switch to build mode, build your own map
switch to the simulation mode, connect to ROS, then start moving mode, watch the effect in main node
you can rosrun another TopoUI, and use the realtime node to watch the building node from UI

#TODO big list
search through the built map

auto select in unknown maps

sub nodes

local topo struct abstraction