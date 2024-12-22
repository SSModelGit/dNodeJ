using PyCall: pyimport
using Parameters: @unpack

export R2Artifacts, connect_ros2_system, init_ros
export R2Node, make_node, add_publisher, add_subscriber

struct R2Artifacts
    rclpy
    std_msgs
    geometry_msgs

    R2Artifacts(rclpy, std_msgs, geometry_msgs) = new(rclpy, std_msgs, geometry_msgs)
end

connect_ros2_system() = R2Artifacts(pyimport("rclpy"), pyimport("std_msgs.msg"), pyimport("geometry_msgs.msg"))

init_ros(r2::R2Artifacts) = r2.rclpy.init()

struct R2Node
    node
    publishers::Dict
    subscribers::Dict

    R2Node(node, publishers::Dict, subscribers::Dict) = new(node, publishers, subscribers)
end

make_node(name::String, r2::R2Artifacts) = R2Node(r2.rclpy.create_node(name), Dict(), Dict())

function add_publisher(node::R2Node, topic_name::String, msg_type::Any, queue_size::Integer)
    if haskey(node.publishers, topic_name)
        println("Attempting to add publisher that already exists")
        return false
    else
        println("Adding new publisher to topic: /", topic_name, " ...")
        node.publishers[topic_name] = node.create_publisher(msg_type, topic_name, queue_size)
        println("success!") # TODO: check if publisher creation is actually successful
        return true
    end
end

function add_subscriber(node::R2Node, topic_name::String, callback::Function, msg_type::Any, frequency::Integer)
    if haskey(node.subscribers, topic_name)
        println("Attempting to add subscriber that already exists")
        return false
    else
        println("Adding new subscriber on topic: /", topic_name, " ...")
        node.subscribers[topic_name] = node.create_subscribtion(msg_type, topic_name, callback, frequency)
        println("success!") # TODO: check if subscriber creation is actually successful
        return true
    end
end

function publish_std_string_msg(node::R2Node, r2::R2Artifacts, msg_content::String, topic::String)
    msg = r2.std_msgs.String()
    msg.data = msg_content
    println("Publishing: ", msg.data)
    node.publishers[topic].publish(msg)
end

retrieve_std_string_msg(msg) = msg.data