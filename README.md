# easynav_simple_stack

> [!WARNING]
> The development of the plugins in this repo has been moved to [easynav_plugins](https://github.com/EasyNavigation/easynav_plugins).

## SimpleMapsManager Plugin for Easy Navigation

The `SimpleMapsManager` plugin provides a lightweight, ROS 2-compatible map management component for the Easy Navigation (EasyNav) framework. It is based on a 2D boolean grid (`SimpleMap`) and is optimized for efficient updates and metric conversion. This plugin can handle both static and dynamic map layers, integrating sensor-based perception to update the dynamic map.

---

### Features

- Loads static maps from disk or incoming `OccupancyGrid` messages.
- Publishes static and dynamic map layers as `nav_msgs/msg/OccupancyGrid`.
- Applies sensor updates to a dynamic map layer.
- Can receive incoming maps via a subscriber interface.
- Offers a service interface to save maps to the original file path at runtime.
- Compatible with `pluginlib` for dynamic loading.

---

### Parameters

All parameters must be defined in the namespace corresponding to the plugin name (e.g., `my_plugin.package`, `my_plugin.map_path_file`):

| Name                        | Type   | Description                                                        | Default |
|-----------------------------|--------|--------------------------------------------------------------------|---------|
| `plugin_name.package`       | string | Name of the ROS 2 package containing the static map file.          | `""`    |
| `plugin_name.map_path_file` | string | Path to the map file (relative to the specified package).          | `""`    |

If both parameters are provided, the plugin loads the static map from the resolved file path during initialization.

---

### Topics

| Topic                                 | Type                              | Direction | Description                                    |
|---------------------------------------|-----------------------------------|-----------|------------------------------------------------|
| `<node>/<plugin_name>/map`            | `nav_msgs/msg/OccupancyGrid`      | Publisher | Published static map (latched).                |
| `<node>/<plugin_name>/dynamic_map`    | `nav_msgs/msg/OccupancyGrid`      | Publisher | Dynamic map updated based on sensor data.      |
| `<node>/<plugin_name>/incoming_map`   | `nav_msgs/msg/OccupancyGrid`      | Subscriber| Allows runtime map replacement via messages.   |

---

### Services

| Service                                | Type                     | Description                                        |
|----------------------------------------|--------------------------|----------------------------------------------------|
| `<node>/<plugin_name>/savemap`         | `std_srvs/srv/Trigger`   | Saves the current static map to its original path |

---

### File Format

Map files used by `SimpleMap` follow a plain-text format:

```
<width> <height> <resolution> <origin_x> <origin_y>
0 1 1 0 0 1 ...
```

- The first line contains metadata: width, height, resolution (meters/cell), origin x, origin y.
- The second line is the grid content, in row-major order, using `0` for free and `1` for occupied cells.

---

### Example Plugin Configuration

```yaml
maps_manager_node:
  ros__parameters:
    maps_manager_plugins:
      - simple
    simple:
      package: "my_maps_pkg"
      map_path_file: "maps/warehouse.map"
```

---

### Example Usage of Save Service

To save the current map to the original file location:

```bash
ros2 service call /<node>/simple/savemap std_srvs/srv/Trigger "{}"
```

---

### License

This plugin is part of the [Easy Navigation (EasyNav)](https://github.com/IntelligentRoboticsLab/easy_navigation) framework and is released under the GNU General Public License v3.0.
