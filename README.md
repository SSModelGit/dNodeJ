# dNodeJ

The simple testing package for trialing distributed algorithms written in Julia. This specifically targets planning algorithms that need to simulate "multiple distributed agents connected according to an arbitrary network graph." This package specifically provides the backbone architecture for specifying:

* The distributed agent - i.e., the distributed agent Node.
* The network graph dictating connections between agent Nodes.

The current implementations provided use: `PyCall` in combination with `rclpy` from ROS2 (assuming that each node implementation is running in a Docker container). Future additions will also target native Julia implementations of network architectures, such as `RemoteChannel`.

## Structure
```
dNodeJ
├── dNodeArch
│   ├── examples
│   ├── src
│   └── test
├── docker
├── ground_model_interfaces
│   ├── include
│   ├── src
│   └── srv
└── simple_world_models
    ├── resource
    ├── simple_world_models
    └── test
```

Currently there are four components, centering around a ROS2 distributed architecture.

* `dNodeArch` contains the Julia files for running a ROS2 connection. The relevant functions are defined in `src/ros2_node_bridge.jl`. The `examples/` folder contains example distributed implementations of the distributed architecture.
* `docker` contains Dockerfile and related information for running `docker compose` projects. The corresponding `docker-compose.yml` is defined at the repository root level.
    * All volume mounts rely on the filepaths specified in the `.env` file (also defined at the repository root level) - please edit to match your system.
    * The currently supported `docker compose` targets are:
        * `dev`: The dev shell, automatically mounting files according to the filepath specified in the `.env` file.
        * `colcon`: Builds any ROS2 project located in the mounted files.
        * `world2dserver`: Spins up a simple 2D world simulation as a ROS2 server, providing world state estimates via client requests.
* `ground_model_interfaces`: Defines all custom ROS2 srv and msg files used for custom world server nodes. In a distributed system, each agent can independently query the world server to get their own "observations".
    * The srv `srv/TwoDimWorldScalarStateReq.srv` is the type used by the `world2dserver` target.
* `simple_world_models`: A simple example package that demonstrates how to implement a custom world.
    * The file `simple_world_models/two_dim_world.py` defines the server used by the `world2dserver` target.
        * The file `simple_world_models/two_dim_world_req.py` is a ROS2-Python method of querying the server. Additional Julia methods are provided in `dNodeArch`.
