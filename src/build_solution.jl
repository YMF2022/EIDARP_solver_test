"""
Initialize Solution with `eidarp`. 

# Arguments
- `eidarp::EIDARP`: The instance.
"""
function Solution(eidarp::EIDARP)
    # arcs
    A_bus = eidarp.arcs_bus
    A_ts = Dict{Tuple, Float64}()
    for arc in eidarp.arcs_ts
        A_ts[arc] = eidarp.traveltime_ts[arc...]
    end
    A_walk = Dict{Tuple, Float64}()
    for arc in eidarp.arcs_walk
        A_walk[arc] = eidarp.walktime[arc...]
    end
    # nodes
    orders = []
    nodes = eidarp.nodes
    tt = eidarp.traveltime
    for node in nodes
        id = node.ID
        # outgoing and incoming nodes
        # if !isa(node, Depot)
        #     node.incoming = eidarp.nodes_bus_inout[id][1]
        #     node.outgoing = eidarp.nodes_bus_inout[id][2]
        # end
        # if isa(node, TransitStation)
        #     node.incoming_ts = eidarp.nodes_ts_inout[id][1]
        #     node.outgoing_ts = eidarp.nodes_ts_inout[id][2]
        # end
        # sort transit stations by distance to Pickup/Droppoff nodes
        if isa(node, Pickup) | isa(node, Droppoff)
            dist = tt[node.ID, eidarp.ts_map[1,:]] # get distance of physical ts
            rank, ts_order = ts_rank(dist, eidarp.ts_map)
            for (i,ts_list) in enumerate(ts_order)
                tbr = [] # remove ilegal ts
                for ts in ts_list
                    if isa(node, Pickup)
                        # if ts ∉ node.outgoing
                        if ts ∉ eidarp.nodes_bus_inout[id][2]
                            push!(tbr, ts)
                        elseif (node.timewindow[1] + tt[node.ID,ts] > nodes[ts].dep_time) | (node.timewindow[2] + tt[node.ID,ts] < nodes[ts].dep_time - eidarp.max_waittime)
                            push!(tbr, ts)
                        end
                    elseif isa(node, Droppoff)
                        # if ts ∉ node.incoming
                        if ts ∉ eidarp.nodes_bus_inout[id][1]
                            push!(tbr, ts)
                        elseif (nodes[ts].arr_time + tt[ts,node.ID] > node.timewindow[2]) | (nodes[ts].arr_time + tt[ts,node.ID] + eidarp.max_waittime < node.timewindow[1])
                            push!(tbr, ts)
                        end
                    end
                end
                ts_order[i] = setdiff(ts_list, tbr)
            end
            node.ts_order = ts_order
        end
    end
    # prepare ts_pairs
    for node in nodes[1:eidarp.n_c]
        if isa(node, Pickup)
            node_drop = nodes[node.dropoffID]
            pairs = k_nearest_ts(node, node_drop, tt, A_ts, k = DEGREE_TS_EXPLORATION)
            node.ts_pair = pairs
        end
    end

    # prepare common pairs
    common_pair = Dict{Tuple, Vector}()
    for c1 in 1:eidarp.n_c
        ts_c1 = nodes[c1].ts_pair
        # pair_c1 = s.N[c1].ts_pair[1:DEGREE_TS_EXPLORATION]
        for c2 in c1+1:eidarp.n_c
            ts_c2 = nodes[c2].ts_pair
            intersect_pair = intersect(ts_c1, ts_c2)
            if !isempty(intersect_pair)
                common_pair[(c1,c2)] = intersect_pair
            end
        end
    end
    # Parameters
    cgr_dummy = reshape(eidarp.cgr_rng, :, eidarp.n_cgr)
    params = Parameter(eidarp.n_c, eidarp.nnode, eidarp.n_k,
                        Set(A_bus), A_ts, A_walk, tt, eidarp.dist, eidarp.max_waittime, common_pair, eidarp.t_end, ω, ones(Float64, 3))
    usedbus = zeros(Bool, eidarp.n_k)
    servedcus = zeros(Bool, eidarp.n_c)
    routes = [Route(eidarp, k) for k in 1:eidarp.n_k]
    return Solution(params, nodes, usedbus, servedcus,  routes, params.ω*eidarp.n_c)
end


# Initialization of Route
function Route(eidarp::EIDARP, k::Int64; initial_Elevel = initE)
    nnode = eidarp.nnode
    bus = eidarp.buses[k]
    depot_o = nnode - 2*eidarp.n_depot + bus.depot # origin depot
    depot_d = nnode - eidarp.n_depot + bus.depot # destination depot
    capacity = bus.capacity
    battery = bus.maxbattery
    suc = zeros(Int, nnode)
    pre = zeros(Int, nnode)
    earliest = zeros(Float64, nnode)
    latest = zeros(Float64, nnode)
    for n in 1:nnode
        node = eidarp.nodes[n]
        if isa(node, TransitStation)
            earliest[n] = node.dep_time - eidarp.max_waittime
            latest[n] = node.arr_time + eidarp.max_waittime
        else
            earliest[n] = node.timewindow[1]
            latest[n] = node.timewindow[1]
        end
    end
    cap = zeros(Int64, nnode)
    β = bus.β
    len = 0
    # path = Node[]
    pathID = Vector{Int64}(undef, nnode)
    node_pos = zeros(Int64, nnode)
    A = Vector{Float64}(undef, nnode)
    B = Vector{Float64}(undef, nnode)
    W = Vector{Float64}(undef, nnode) 
    D = Vector{Float64}(undef, nnode) 
    # rt = Vector{Float64}(undef, nnode) 
    F = Vector{Float64}(undef, nnode) 
    F_min = Vector{Float64}(undef, nnode)
    y = Vector{Int64}(undef, nnode)
    # soc = initE * battery .* ones(Float64, nnode)
    removeE = true
    return Route(k, depot_o, depot_d, capacity, battery, β,
                pathID, node_pos, len, suc, pre, earliest, latest, cap, 
                A, B, W, D, y, F, F_min, false, removeE)
end

function ts_rank(dist::Vector{Float64}, ts_map)
    rank = Vector[]
    ts_order = Vector[]
    srt = sortperm(dist)
    for phy_ts in srt
        if isempty(rank)
            push!(rank, [phy_ts])
            push!(ts_order, ts_map[:,phy_ts])
            continue
        end
        if dist[phy_ts] == dist[rank[end][end]]
            push!(rank[end], phy_ts)
            ts_order[end] = [ts_order[end]; ts_map[:,phy_ts]]
        else
            push!(rank, [phy_ts])
            ts_order = push!(ts_order, ts_map[:,phy_ts])
        end
    end
    return rank, ts_order
end