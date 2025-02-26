using PyCall: pyimport
using Parameters: @unpack
using JSON: parse

export R2Artifacts, connect_ros2_system, init_ros, close_ros
export R2Node, make_node, add_publisher, add_subscriber, remove_subscriber, publish_msg, publish_std_string_msg, retrieve_std_string_json
export R2ServiceInfo, connect_ground_model, request_r2_service

struct R2Artifacts
    rclpy
    std_msgs
    geometry_msgs
    custom_interfaces

    R2Artifacts(rclpy, std_msgs, geometry_msgs, custom_interfaces) = new(rclpy, std_msgs, geometry_msgs, custom_interfaces)
end

connect_ros2_system() = R2Artifacts(pyimport("rclpy"),
                                    pyimport("std_msgs.msg"), pyimport("geometry_msgs.msg"),
                                    Dict("ground_models"=>pyimport("ground_model_interfaces.srv")))

init_ros(r2::R2Artifacts) = r2.rclpy.init()
close_ros(r2::R2Artifacts) = r2.rclpy.shutdown()

struct R2Node
    node
    publishers::Dict
    subscribers::Dict

    R2Node(node, publishers::Dict, subscribers::Dict) = new(node, publishers, subscribers)
end

make_node(name::String, r2::R2Artifacts) = R2Node(r2.rclpy.create_node(name), Dict(), Dict())

function add_publisher(r2n::R2Node, topic_name::String, msg_type::Any, queue_size::Integer)
    @unpack node = r2n
    if haskey(r2n.publishers, topic_name)
        println("Attempting to add publisher that already exists")
        return false
    else
        println("Adding new publisher to topic: /", topic_name, " ...")
        r2n.publishers[topic_name] = node.create_publisher(msg_type, topic_name, queue_size)
        println("success!") # TODO: check if publisher creation is actually successful
        return true
    end
end

function add_subscriber(r2n::R2Node, topic_name::String, callback::Function, msg_type::Any, frequency::Integer)
    @unpack node = r2n
    if haskey(r2n.subscribers, topic_name)
        println("Attempting to add subscriber that already exists")
        return false
    else
        println("Adding new subscriber on topic: /", topic_name, " ...")
        r2n.subscribers[topic_name] = node.create_subscription(msg_type, topic_name, callback, frequency)
        println("success!") # TODO: check if subscriber creation is actually successful
        return true
    end
end

remove_subscriber(r2n::R2Node, topic_name::String) = pop!(r2n.subscribers, topic_name).destroy()

"""More generic publish function.
"""
function publish_msg(r2n::R2Node, data::Dict, topic::String)
    let pub = r2n.publishers[topic], msg = pub.msg_type()
        for (key, value) in data
            if haskey(msg, key)
                msg[key] = value
            else
                println("Sorry, provided data does not match publisher type: ", pub.msg_type)
                return 500
            end
        end
        println("Publishing: ", msg.data)
        pub.publish(msg)
    end
end

function publish_std_string_msg(r2n::R2Node, data::Dict, topic::String)
    let pub = r2n.publishers[topic], msg = pub.msg_type()
        msg.data = json(data)
        println("Publishing: ", msg.data)
        pub.publish(msg)
    end
end

retrieve_std_string_json(msg) = JSON.parse(string(msg.data))

struct R2ServiceInfo
    name::String
    srv_type::Any

    R2ServiceInfo(name::String, srv_type::Any) = new(name, srv_type)
end

connect_ground_model(service_name::String, srv_type::String, r2::R2Artifacts) = R2ServiceInfo(service_name, r2.custom_interfaces["ground_models"][srv_type])

function request_r2_service(r2n::R2Node, srv_info::R2ServiceInfo, data::Dict, timeout::Float64, r2::R2Artifacts)
    @unpack node = r2n
    client = node.create_client(srv_info.srv_type, srv_info.name)

    while !client.wait_for_service(timeout_sec=timeout)
        println("Service not available, waiting...")
    end

    request = srv_info.srv_type.Request()
    for (key, value) in data
        if haskey(request, key)
            request[key] = value
        else
            node.destroy_client(client)
            return 500, "Data doesn't match service type - canceling client request"
        end
    end

    future = client.call_async(request)
    r2.rclpy.spin_until_future_complete(node, future)
    reply = future.result()
    node.destroy_client(client)
    return 200, reply
end