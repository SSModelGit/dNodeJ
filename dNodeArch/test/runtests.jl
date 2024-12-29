using dNodeArch
using Test

@testset "R2NodeJ.jl" begin
    # Write your tests here.
end

function test_subscribing()
    r2 = connect_ros2_system()
    init_ros(r2)
    r2n = make_node("yappee", r2)
    subber = add_subscriber(r2n, "/chatter", x->println(retrieve_std_string_msg(x)), r2.std_msgs["String"], 1)
    try
        while r2.rclpy.ok()
            r2.rclpy.spin_once(r2n.node)
        end
    finally
        close_ros(r2)
    end
end
