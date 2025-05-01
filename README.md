# easynav_simple_stack

## SimpleMapsManager Plugin for Easy Navigation

This plugin provides a lightweight map management solution for the Easy Navigation (EasyNav) framework, using a simple 2D grid map based on standard C++ types (`SimpleMap`). It is designed to be efficient and easy to integrate into ROS 2 systems.

---

### Features

- Supports loading maps from files or from `nav_msgs::msg::OccupancyGrid` messages.
- Converts and publishes maps as `nav_msgs::msg::OccupancyGrid`.
- Maintains both static and dynamic map layers.
- Can be used as a plugin via the `pluginlib` infrastructure.

---

### Parameters

All parameters must be specified in the namespace of the plugin (e.g., `my_manager.package`, `my_manager.map_path_file`):

| Name                 | Type   | Description                                        | Default |
|----------------------|--------|----------------------------------------------------|---------|
| `plugin_name.package`            | string | Name of the ROS 2 package containing the map file. | `""`    |
| `plugin_name.map_path_file`      | string | Relative path to the map file within the package.  | `""`    |

If both parameters are provided, the static map is loaded from the file at runtime.

---

### Topics

| Topic name                            | Type                              | Direction | Description                                |
|---------------------------------------|-----------------------------------|-----------|--------------------------------------------|
| `<node>/<plugin_name>/map`                 | `nav_msgs/msg/OccupancyGrid`      | Publisher | Published static map (latched).            |
| `<node>/<plugin_name>/dynamic_map`         | `nav_msgs/msg/OccupancyGrid`      | Publisher | Dynamic map updated with sensor data.      |
| `<node>/<plugin_name>/incoming_map`        | `nav_msgs/msg/OccupancyGrid`      | Subscriber| Allows external modules to overwrite maps. |

---

### Usage

Declare and load the plugin in your component node:

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

### File Format

Maps saved or loaded with `SimpleMap` have the following format:

```
<width> <height> <resolution> <origin_x> <origin_y>
0 0 1 0 0 1 ...
```

Where the second line contains one value per cell (`0` for false, `1` for true), in row-major order.

---

### License

This plugin is part of the Easy Navigation (EasyNav) framework and is licensed under the GNU General Public License v3.0.