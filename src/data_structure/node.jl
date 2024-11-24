abstract type Node end

"""
    Pickup <: Node

A `Pickup` is a subtype of `Node` that represents a pickup location in a transportation network.
"""
mutable struct Pickup <: Node
    ID::Int64                                       # ID in the departure-expanded network
    dropoffID::Int64                                # ID of the dropoff node
    coord::Tuple{Float64,Float64}                   # x and y coordinates
    timewindow::Tuple{Float64,Float64}              # service time window
    μ::Float64                                      # service time
    q::Int64                                        # load
    L::Float64                                      # max ridetime
    close_ts::Vector{Int64}                         # Transit stations within walking distance
    ts_order::Vector{Vector{Int64}}                 # Transit station physical ID and its dummies by distance
    ts_pair::Vector{Tuple{Int64, Int64}}            # Transit station pairs between O-D ordered by distance
    bus::Int64                                      # Operations: bus ID visiting this node
    walk_ts::Int64                                  # Operations: Transit station ID if customer walk to
    bus_ts::Int64                                   # Operations: Transit station ID if customer take bus to
end

mutable struct Droppoff <: Node
    ID::Int64                                       # ID in the departure-expanded network
    pickupID::Int64                                 # ID of the pickup node
    coord::Tuple{Float64,Float64}                   # x and y coordinates
    timewindow::Tuple{Float64,Float64}              
    μ::Float64                                      # service time
    q::Int64                                        # load
    L::Float64                                      # max ridetime
    close_ts::Vector{Int64}                         # Transit stations within walking distance
    ts_order::Vector{Vector{Int64}}                 # Transit station physical ID and its dummies by distance
    bus::Int64                                      # Operations: bus ID visiting this node
    walk_ts::Int64                                  # Operations: Transit station ID if customer walk from
    bus_ts::Int64                                   # Operations: Transit station ID if customer take bus from  
end

mutable struct TransitStation <: Node
    ID::Int64                                       # ID in the departure-expanded network
    phyID::Int64                                    # the phsical number of transit stop
    coord::Tuple{Float64,Float64}
    layer_no::Int64
    dep_time::Float64
    arr_time::Float64
    lineNo::Int64                                   # The transit line of this node
    transfer::Vector{Int64}                         # Transit station to be transfered
    dweltime::Float64
    μ::Float64                                      # Service time
    cus_board::Matrix{Bool}                         # Operations (not in used): Row is bus, Column is Customers boarding to the transit station
    cus_alight::Matrix{Bool}                        # Operations (not in used): Row is bus, Customer alighting from the transit station
    bus_servetype::Vector{Int64}                    # Operations: The service type: 1 for pickup customer; 2 for delivery customer; 3 for both
    q::Vector{Int64}                                # The number of customers served by each bus, which is dynamic     
end


mutable struct Charger <: Node
    ID::Int64                                       # ID in the departure-expanded network
    phyID::Int64                                    # the phsical number of chargers
    coord::Tuple{Float64,Float64}
    timewindow::Tuple{Float64,Float64}
    α::Float64                                      # charging speed
    μ::Float64                                      # service time
    q::Int64                                        # load 
end

struct Depot <: Node
    ID::Int64                                       # ID in the departure-expanded network
    phyID::Int64                                    # the phsical number of depot: 0 origin; 1 destination
    coord::Tuple{Float64,Float64}
    timewindow::Tuple{Float64,Float64}
    μ::Float64
    q::Int64                                        # load
end

# copy
copy(n::Pickup) = Pickup(n.ID, n.dropoffID, n.coord, n.timewindow, n.μ, n.q, n.L, n.close_ts, 
                        n.ts_order, n.ts_pair, copy(n.bus), copy(n.walk_ts), copy(n.bus_ts))
copy(n::Droppoff) = Droppoff(n.ID, n.pickupID, n.coord, n.timewindow, n.μ, n.q, n.L, n.close_ts, 
                            n.ts_order, copy(n.bus), copy(n.walk_ts), copy(n.bus_ts))
copy(n::TransitStation) = TransitStation(n.ID, n.phyID, n.coord, n.layer_no, n.dep_time, n.arr_time, n.lineNo, 
                                        n.transfer, n.dweltime, n.μ,  
                                        copy(n.cus_board), copy(n.cus_alight), copy(n.bus_servetype), copy(n.q))
copy(n::Charger) = Charger(n.ID, n.phyID, n.coord, n.timewindow, n.α, n.μ, n.q)
copy(n::Depot) = Depot(n.ID, n.phyID, n.coord, n.timewindow, n.μ, n.q)

# Is served by transit service
isservedts(n::Union{Pickup, Droppoff})::Bool = (n.walk_ts + n.bus_ts) > 0

# Is served by bus 
isservedbus(n::Union{Pickup, Droppoff}, k::Int64)::Bool = n.bus == k
isservedbus(n::TransitStation, k::Int64)::Bool = sum(n.cus_board[k,:]) + sum(n.cus_alight[k,:]) > 0
# isservedbus(n::Charger, k::Int)::Bool = any(key[1] == k for key in keys(n.cgr_evt))
isservedbus(n::Charger, k::Int)::Bool = any(key[1] == k for key in keys(n.cgr_evt))

# Is served
isserved(n::Union{Pickup, Droppoff})::Bool = (n.bus + n.walk_ts) > 0
isserved(n::TransitStation, k::Int64) = isservedbus(n, k)

# The served transit station 
servedts(n::Union{Pickup, Droppoff}) = isservedts(n) ? n.walk_ts + n.bus_ts : nothing

# get_ID 
get_ID(n::Node) = n.ID

# Get coordinates
get_coord(node::Node) = [node.coord[1] node.coord[2]]
get_coord(nodes::Vector{T}) where T <: Union{Node, Float64} = vcat(get_coord.(nodes)...)

# Get the early timewindow
function get_earlytw(node::Node, bus::Int64; type::Int64 = 0)
    # if not transit station
    if !isa(node, TransitStation)
        return node.timewindow[1]
    end
    # For transit station
    if node.bus_servetype[bus] == 1  # i is to pickup customer
        return node.arr_time
    elseif node.bus_servetype[bus] == 2 # i is to dropoff customer
        return node.dep_time - MAX_WAIT_TIME
    elseif node.bus_servetype[bus] == 3 # i is for both pickup and dropoff customer
        return node.dep_time - MAX_WAIT_TIME
    elseif node.bus_servetype[bus] == 0 && type == 1
        return node.arr_time
    elseif node.bus_servetype[bus] == 0 && type == 2
        return node.dep_time - MAX_WAIT_TIME
    else
        error("Service type is not assigned to transit node $(node.ID)")
    end
end

# Get the late timewindow
function get_latetw(node::Node, bus::Int64; type::Int64 = 0)
    # If not transit station
    if !isa(node, TransitStation)
        return node.timewindow[2]
    end
    # For transit station
    if node.bus_servetype[bus] == 1  # i is to pickup customer
        return node.arr_time + MAX_WAIT_TIME
    elseif node.bus_servetype[bus] == 2 # i is to dropoff customer
        return T_END
        # return node.dep_time + eidarp.t_end
    elseif node.bus_servetype[bus] == 3 # i is for both pickup and dropoff customer
        return node.arr_time+ MAX_WAIT_TIME
    elseif node.bus_servetype[bus] == 0 & type == 1
        return node.arr_time + MAX_WAIT_TIME
    elseif node.bus_servetype[bus] == 0 & type == 2
        return node.dep_time
    else
        error("Service type is not assigned to transit node $(node.ID)")
    end
end

# Get the load of each node
function get_load(node::Node, bus::Int64)
    if !isa(node, TransitStation)
        return node.q
    else
        return node.q[bus]
    end
end

# Get the maximum ridetime
function get_maxridetime(node::Node)
    if isa(node, Pickup) | isa(node, Droppoff)
        return node.L
    else
        return Inf
    end
end

# Update service type for bus k at a transit node: type = 1 pickup customer; type = 2 deliver customer; type = 3 both
function update_servetype!(node::TransitStation, bus::Int64, type::Int64; update_q::Bool = true)
    if node.bus_servetype[bus] == 0
        node.bus_servetype[bus] = type
    elseif node.bus_servetype[bus] != type
        node.bus_servetype[bus] = 3
    end
    if update_q == true
        if type == 1 
            node.q[bus] += 1
        elseif type == 2 
            node.q[bus] -= 1
        end
    end
end

function update_servetype!(node::TransitStation, bus::Int64)
    cus_alight = sum(node.cus_alight[bus,:])
    cus_board = sum(node.cus_board[bus,:])
    if (cus_alight == 0) && (cus_board == 0)
        node.bus_servetype[bus] = 0
    elseif (cus_alight > 0) && (cus_board == 0)
        node.bus_servetype[bus] = 1
    elseif (cus_alight == 0) && (cus_board > 0)
        node.bus_servetype[bus] = 2
    elseif (cus_alight > 0) && (cus_board > 0)
        node.bus_servetype[bus] = 3
    else
        error("bus_sevetype is wrong for node $(node.ID)")
    end
end