# dNodeArch

The simple testing package for trialing distributed algorithms written in Julia. This specifically targets planning algorithms that need to simulate "multiple distributed agents connected according to an arbitrary network graph." This package specifically provides the backbone architecture for specifying:

* The distributed agent - i.e., the distributed agent Node.
* The network graph dictating connections between agent Nodes.

The current implementations provided use: `PyCall` in combination with `rclpy` from ROS2 (assuming that each node implementation is running in a Docker container). Future additions will also target native Julia implementations of network architectures, such as `RemoteChannel`.