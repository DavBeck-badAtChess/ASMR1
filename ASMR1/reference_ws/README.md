# What ships, in detail

```
reference_ws/
├── README.md                          (this file)
└── src/
    ├── my_robot_perception/           (full package — sensor helpers,
    │                                   goal_checker_node,
    │                                   scan_to_pointcloud_node)
    ├── my_robot_interfaces/           (full package — defines
    │                                   SetVelocity.action)
    ├── my_robot_control/              (full package — overlays your
    │                                   Week 2 my_robot_control with
    │                                   velocity_controller_node added)
    └── my_robot_bringup/
        ├── worlds/
        │   └── obstacle_world.sdf     (drop-in: the obstacle world for
        │                               this minilab)
        └── config/
            └── my_robot.rviz          (drop-in: RViz config with
                                        debug displays)
```