module dNodeArch

# Write your package code here.

function run_example(name::String)
    example_dir = joinpath(dirname(@__FILE__), "../examples")
    example_file = joinpath(example_dir, "$name.jl")
    if isfile(example_file)
        include(example_file)
    else
        error("Example '$name' not found.")
    end
end

include("ros2_node_bridge.jl")

end
