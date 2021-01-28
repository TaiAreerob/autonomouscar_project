# Vectormap Builder

#### Compile
    g++ -std=c++17 -o waypoint2vectormap singleWaypointToOpenplannerVectormap.cpp
#### How to use
    mkdir target_dir
    ./waypoint2vectormap soongsil_waypoints.csv target_dir/

Since Autoware's vectormap builder(https://tools.tier4.jp/) can not simply generate Openplanner Vectormap(Aisan vectormap), this source code transforms Waypoint.csv to Aisan vectormap.

Input : output waypoint.csv from "https://tools.tier4.jp/feature/vector_map_builder_ll2/ ". See the guide in the link

Output : vectormap capable with openplanner.
