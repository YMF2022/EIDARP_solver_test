struct Parameter
    num_cus::Int64                      # number of customers to be served
    num_node::Int64                     # the number of nodes
    num_bus::Int64                      # the number of buses
    A_bus::Set{Tuple}                   # Bus arcs and their travel time
    A_ts::Dict{Tuple, Float64}          # Transit arcs and their travel time
    A_walk::Dict{Tuple, Float64}        # Walk arcs and their travel time
    tt::Matrix{Float64}                 # Bus traveltime matrix
    dist::Matrix{Float64}               # Distance matrix
    maxwait::Float64                    # Maximum waiting time at transit station
    common_pair::Dict{Tuple, Vector}    # The common ts pairs from every two customers
    t_end::Float64                      # End of operational time
    ω::Float64                          # penalty cost of rejecting one customer
    λ::Vector                           # Vector of weights in the objective function
end

mutable struct Route
    ID::Int64                           # Bus ID of the route - Params
    depot_o::Int64                      # Origin depot of the bus - Params
    depot_d::Int64                      # destination depot of the bus - Params
    capacity::Int64                     # Capacity of the bus (route) - Params
    battery::Float64                    # Batter capacity of the bus (route) - Params
    β::Float64                          # Energy consumption rate in kWh/km - Params
    pathID::Vector{Int64}               # Vector of node ID
    node_pos::Vector{Int64}             # Position of each node
    len::Int64                          # length of the route
    suc::Vector{Int64}                  # Scuccessor of each node, fixed length of all nodes
    pre::Vector{Int64}                  # Predecessor of each node, fixed length of all nodes
    earliest::Vector{Float64}           # Earliest possible time can be visited for each node, fixed length of all nodes
    latest::Vector{Float64}             # Latest possbile time can be visited for each node, fixed length of all nodes
    cap::Vector{Int64}                  # Capacity at each node, fixed length of all nodes
    A::Vector{Float64}                  # Arrival time at vertex i 
    B::Vector{Float64}                  # Beginning of service at vertex i 
    W::Vector{Float64}                  # Wait time before starting service at vertex i 
    D::Vector{Float64}                  # Departure time at vertex i 
    y::Vector{Int64}                    # Cummulative laod after leaving vertex i
    F::Vector{Float64}                  # Slack time at vertex i
    F_min::Vector{Float64}              # Min delay required at i
    repair::Bool                        # Indicate whether or not this route goes through time repair step
    removeE::Bool                       # ture, if charging node(s) is removed from this route 
    # soc::Vector{Float64}                # State-of-charge at vertex i
end

mutable struct Solution
    params::Parameter                   # Parameters
    N::Vector{Node}                     # Vector of nodes
    usedbus::Vector{Bool}               # Vector of used bus/vehicle
    servedcus::Vector{Bool}             # Vector of unserved customers
    routes::Vector{Route}               # Vector of bus routes
    objective_cost::Float64
end

get_SPEC(r::Route) = r.suc, r.pre, r.earliest, r.cap
get_path(r::Route) = r.pathID[1:r.len]
function get_path(s::Solution)
    [get_path(s.routes[k]) for k in findall(==(1), s.usedbus)]
end
# get_path(s::Solution) = [get_path(s.routes[k]) for k in s.usedbus]

copy(r::Route) = Route(r.ID, r.depot_o, r.depot_d, r.capacity, r.battery, r.β,
                        copy(r.pathID), copy(r.node_pos), r.len, 
                        copy(r.suc), copy(r.pre), copy(r.earliest), copy(r.latest), copy(r.cap), 
                        copy(r.A), copy(r.B), copy(r.W), 
                        copy(r.D), copy(r.y), copy(r.F), copy(r.F_min), r.repair, r.removeE)

function copy(s::Solution)
    N = copy.(s.N)
    routes = copy.(s.routes)
    return Solution(s.params, N, copy(s.usedbus), copy(s.servedcus), routes, s.objective_cost)
end

# Only copy the specific bus route and node and charger nodes
function copy(s::Solution, bus::Int64, nodes::Vector{T}) where T <: Union{Node, Pickup, Droppoff, TransitStation}
    route = copy(s.routes[bus])
    routes = copy(s.routes)
    routes[route.ID] = route
    
    N = copy(s.N)
    for node in nodes 
        N[node.ID] = copy(node)
    end

    Solution(s.params, N, copy(s.usedbus), copy(s.servedcus), routes, s.objective_cost)
end

# Only copy the specific nodes and all bus routes
function copy(s::Solution, node1::Node, node2::Node)
    # N = Vector{Node}(undef, length(s.N))
    N = copy(s.N)
    for (i,n) in enumerate(s.N)
        if i == node1.ID
            N[i] = copy(node1)
        elseif i == node2.ID
            N[i] = copy(node2)
        else
            continue
            # N[i] = n 
        end
    end
    Solution(s.params, N, copy(s.usedbus), copy(s.servedcus), copy.(s.routes), s.objective_cost)
end


function update_served_customer!(sol::Solution)
    n_c = sol.params.num_cus
    for c in 1:n_c
        d = sol.N[c].dropoffID
        if isserved(sol.N[c]) && isserved(sol.N[d])
            sol.servedcus[c] = true
        else
            sol.servedcus[c] = false
        end
    end
end