Relevant source files

-   [as2\_behaviors/as2\_behaviors\_path\_planning/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/CMakeLists.txt)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/common/include/cell\_node.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/common/include/cell_node.hpp)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/common/include/graph\_searcher.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/common/include/graph_searcher.hpp)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/common/include/utils.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/common/include/utils.hpp)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/config/behavior\_default.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/config/behavior_default.yaml)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/launch/path\_planner\_behavior\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/launch/path_planner_behavior_launch.py)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/a\_star/launch/a\_star-path\_planner\_behavior.launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/a_star/launch/a_star-path_planner_behavior.launch.py)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/CMakeLists.txt)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/include/voronoi.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/include/voronoi.hpp)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/include/voronoi\_searcher.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/include/voronoi_searcher.hpp)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/launch/voronoi-path\_planner\_behavior.launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/launch/voronoi-path_planner_behavior.launch.py)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/src/voronoi.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/src/voronoi.cpp)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/src/voronoi\_searcher.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/src/voronoi_searcher.cpp)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/thirdparty/dynamicvoronoi/README.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/thirdparty/dynamicvoronoi/README.md)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/thirdparty/dynamicvoronoi/bucketedqueue.h](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/thirdparty/dynamicvoronoi/bucketedqueue.h)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/thirdparty/dynamicvoronoi/bucketedqueue.hxx](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/thirdparty/dynamicvoronoi/bucketedqueue.hxx)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/thirdparty/dynamicvoronoi/dynamicvoronoi.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/thirdparty/dynamicvoronoi/dynamicvoronoi.cpp)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/thirdparty/dynamicvoronoi/dynamicvoronoi.h](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/thirdparty/dynamicvoronoi/dynamicvoronoi.h)
-   [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/thirdparty/dynamicvoronoi/point.h](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/thirdparty/dynamicvoronoi/point.h)

This page documents the path planning behavior in Aerostack2, which enables drones to navigate autonomously through environments while avoiding obstacles. The path planning system is designed with a plugin-based architecture that allows for different path planning algorithms to be implemented and used interchangeably. For information about trajectory generation behaviors, see [Trajectory Generation](https://deepwiki.com/aerostack2/aerostack2/3.1-trajectory-generation).

## Overview

The path planning system in Aerostack2 is implemented as a behavior plugin that integrates with the overall behavior architecture of the framework. It takes a goal position and the current drone position and generates a path that avoids obstacles in the environment.

```
Common ComponentsPath Planning PluginsPath Planner FrameworkPathPlannerBehaviorPathPlannerPluginBasePlugin RegistryA* PluginVoronoi PluginNew Plugins...GraphSearcherCellNodeUtility Functions
```

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/CMakeLists.txt102-114](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/CMakeLists.txt#L102-L114) [as2\_behaviors/as2\_behaviors\_path\_planning/include/as2\_behaviors\_path\_planning/path\_planner\_plugin\_base.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/include/as2_behaviors_path_planning/path_planner_plugin_base.hpp)

## Architecture

The path planning behavior follows a plugin-based architecture, which consists of:

1.  **Path Planner Behavior**: The main component that interfaces with the AS2 behavior system
2.  **Plugin Base**: Defines the interface that all path planning plugins must implement
3.  **Path Planning Plugins**: Specific implementations of path planning algorithms
4.  **Common Utilities**: Shared components used by various plugins

### Path Planning Plugin Infrastructure

```
PluginBase#std::vector<Point> path_+initialize(as2::Node*, tf_buffer)+on_activate(PoseStamped, Goal)+on_deactivate()+on_modify()+on_pause()+on_resume()+on_execution_end()+on_run()VoronoiPlugin-DynamicVoronoi dynamic_voronoi_-VoronoiSearcher graph_searcher_+initialize(as2::Node*, tf_buffer)+on_activate(PoseStamped, Goal)+on_deactivate()+on_modify()+on_pause()+on_resume()+on_execution_end()+on_run()-update_costs(occ_grid)-update_dynamic_voronoi(occ_grid)AStarPlugin+initialize(as2::Node*, tf_buffer)+on_activate(PoseStamped, Goal)+on_deactivate()+on_modify()+on_pause()+on_resume()+on_execution_end()+on_run()
```

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/include/voronoi.hpp54-106](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/include/voronoi.hpp#L54-L106) [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/src/voronoi.cpp46-150](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/src/voronoi.cpp#L46-L150)

### Path Search Framework

At the core of most path planning plugins is a graph search algorithm. The framework provides a common `GraphSearcher` template class that can be extended for specific search algorithms:

```
GraphSearcher+solve_graph(start, end)#update_graph(graph)#calc_h_cost(current, end)#calc_g_cost(current)#hash_key(point)#cell_in_limits(point)#cell_occuppied(point)VoronoiSearcher+update_voronoi(voronoi)#calc_h_cost(current, end)#calc_g_cost(current)#hash_key(point)#cell_in_limits(point)#cell_occuppied(point)CellNode-Point2i coordinates_-CellNodePtr parent_ptr_-double g_cost_-double h_cost_+set_g_cost(g_cost)+get_g_cost()+get_h_cost()+get_total_cost()Point2i+int x+int y+bool operator==()+bool operator!=()
```

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/common/include/graph\_searcher.hpp36-168](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/common/include/graph_searcher.hpp#L36-L168) [as2\_behaviors/as2\_behaviors\_path\_planning/common/include/cell\_node.hpp36-125](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/common/include/cell_node.hpp#L36-L125) [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/include/voronoi\_searcher.hpp35-54](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/include/voronoi_searcher.hpp#L35-L54)

## Path Planning Plugins

The system currently includes two path planning plugins:

### A\* Plugin

The A\* plugin implements the A\* search algorithm, which is a popular pathfinding algorithm that uses a heuristic to guide the search. It finds the shortest path from start to goal by evaluating the cost to reach each node plus a heuristic estimate of the cost to reach the goal from that node.

### Voronoi Plugin

The Voronoi plugin uses Voronoi diagrams for path planning. A Voronoi diagram partitions the space based on distance to obstacles, creating paths that naturally maximize the distance from obstacles.

```
Data VisualizationVoronoi Path Planning ProcessUnsupported markdown: listUnsupported markdown: listUnsupported markdown: listUnsupported markdown: listUnsupported markdown: listOccupancy GridVoronoi Grid VisualizationDistance Field VisualizationPath Visualization Marker
```

The Voronoi plugin process:

1.  **Occupancy Grid Processing**: Receives the occupancy grid representing the environment
2.  **Voronoi Diagram Generation**: Creates a Voronoi diagram using the `DynamicVoronoi` class
3.  **Distance Field Computation**: Calculates distances from obstacles
4.  **Path Search**: Uses `VoronoiSearcher` (derived from `GraphSearcher`) to find a path along the Voronoi diagram
5.  **Visualization**: Publishes visualization markers for debugging

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/src/voronoi.cpp76-120](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/src/voronoi.cpp#L76-L120) [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/src/voronoi.cpp181-242](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/src/voronoi.cpp#L181-L242)

## Usage

### Configuration

The path planning behavior can be configured through parameters:

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `enable_visualization` | bool | false | Enable visualization topics for debugging |
| `enable_path_optimizer` | bool | false | Enable path smoothing |
| `safety_distance` | float | 1.0 | Minimum distance to obstacles (meters) |
| `plugin_name` | string | \- | Name of the path planning plugin to use (e.g., "a\_star" or "voronoi") |

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/config/behavior\_default.yaml1-5](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/config/behavior_default.yaml#L1-L5)

### Launching

You can launch the path planning behavior using one of the provided launch files:

**General launcher** (requires specifying the plugin):

```
ros2 launch as2_behaviors_path_planning path_planner_behavior_launch.py plugin_name:=a_star
```

**Plugin-specific launchers**:

```
ros2 launch as2_behaviors_path_planning a_star-path_planner_behavior.launch.py
```

or

```
ros2 launch as2_behaviors_path_planning voronoi-path_planner_behavior.launch.py
```

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/launch/path\_planner\_behavior\_launch.py31-87](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/launch/path_planner_behavior_launch.py#L31-L87) [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/a\_star/launch/a\_star-path\_planner\_behavior.launch.py31-46](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/a_star/launch/a_star-path_planner_behavior.launch.py#L31-L46) [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/launch/voronoi-path\_planner\_behavior.launch.py31-47](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/launch/voronoi-path_planner_behavior.launch.py#L31-L47)

### Integration with AS2 Behavior System

The path planning behavior integrates with the AS2 behavior system as a ROS 2 action server. It accepts `NavigateToPoint` action goals containing the target position and returns success when a path is found.

```
"Occupancy Grid""Path Planning Plugin""PathPlannerBehavior""Client""Occupancy Grid""Path Planning Plugin""PathPlannerBehavior""Client"alt[Path found][No path found]"NavigateToPoint Action""on_activate(drone_pose, goal)""Subscribe to occupancy grid""Map updates""Process map & create internal representation""Find path from drone to goal""return true""SUCCESS""return false""FAILURE"
```

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/src/voronoi.cpp82-120](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/src/voronoi.cpp#L82-L120)

## Extending the System

To create a new path planning plugin, you need to:

1.  Create a new class that inherits from `as2_behaviors_path_planning::PluginBase`
2.  Implement all required methods (`initialize`, `on_activate`, etc.)
3.  Register the plugin using the `PLUGINLIB_EXPORT_CLASS` macro
4.  Add the plugin to the plugin list in the main CMakeLists.txt

The most important method to implement is `on_activate`, which is called when the behavior is activated with a goal. This method should:

1.  Receive the current drone pose and goal position
2.  Check if a path is possible
3.  Calculate a path if possible
4.  Store the path in the `path_` member variable
5.  Return `true` if a path was found, `false` otherwise

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/src/voronoi.cpp82-120](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/src/voronoi.cpp#L82-L120) [as2\_behaviors/as2\_behaviors\_path\_planning/plugins/voronoi/src/voronoi.cpp323-324](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/plugins/voronoi/src/voronoi.cpp#L323-L324)

## Common Utilities

The path planning system provides several utility functions to help with coordinate conversions between different spaces:

```
// Convert a point in real-world coordinates to a cell in the occupancy grid
Point2i pointToCell(geometry_msgs::msg::PointStamped point, nav_msgs::msg::MapMetaData map_info, 
                    std::string target_frame_id, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

// Convert a pose to a cell
Point2i poseToCell(geometry_msgs::msg::PoseStamped pose, nav_msgs::msg::MapMetaData map_info, 
                   std::string target_frame_id, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

// Convert a cell to a point in the map frame
geometry_msgs::msg::PointStamped cellToPoint(int cell_x, int cell_y, 
                                             nav_msgs::msg::MapMetaData map_info,
                                             std_msgs::msg::Header map_header);
```

Sources: [as2\_behaviors/as2\_behaviors\_path\_planning/common/include/utils.hpp52-129](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_path_planning/common/include/utils.hpp#L52-L129)

## Conclusion

The path planning system in Aerostack2 provides a flexible architecture for implementing different path planning algorithms. The plugin-based design allows for easy extension with new algorithms while reusing common components. The current implementation includes A\* and Voronoi-based path planning, providing options for different scenarios.