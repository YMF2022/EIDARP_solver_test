struct TransitStation
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
end

struct TransitNetwork
    nodes::Vector
    arcs::Vector{Tuple}
    traveltime::Matrix{Float64}
    traveltime_nowaiting::Matrix{Float64}
    transfer_arcs::Vector{Tuple}
    map::Matrix{Int64}
    paths::Dict{Tuple, Vector}
end

function show_ts_arcs(origin::Int64, ts_network::TransitNetwork)
    ts_arcs = ts_network.arcs
    res = []
    for a in ts_arcs
        if a[1] == origin
            push!(res, a)
        end
    end
    return res
end

function show_ts_arcs(origin::Int64, ts_arcs::Vector{Tuple})
    res = []
    for a in ts_arcs
        if a[1] == origin
            push!(res, a)
        end
    end
    return res
end

import Base.+, Base.-
+(range::UnitRange{Int64}, offset::Int64) = UnitRange(range.start + offset, range.stop + offset)
+(tup::Tuple{Int64, Int64}, offset::Int64) = (tup[1] + offset, tup[2] + offset)
-(tup::Tuple{Int64, Int64}, offset::Int64) = (tup[1] - offset, tup[2] - offset)
+(vec::Vector{Int64}, offset::Int64) = [i += offset for i in vec]

get_arcs(arc::NamedTuple{(:ts1, :ts2, :tt), Tuple{Int64, Int64, Float64}}) = (arc.ts1,arc.ts2)


"""
 Function to parse dataings into appropriate data types
"""
function parse_column(data::T) where T <:Union{Int, String, String3, String7, Missing}
    if ismissing(data) || isempty(data)
        return Int[]
    elseif typeof(data) == Int64
        return [data]
    elseif occursin(',', data)
        return parse.(Int, split(data, ','))
    else
        return [parse(Int, data)]
    end
end

function construct_ts_nodes(TS_data::DataFrame, timetables::Vector, dep_ts::Matrix, n_ts::Int64, n_layer::Int64)
    ts_nodes = Vector(undef, n_ts*n_layer)
    for lyr in 1:n_layer
        for ts in 1:n_ts
            coord = (TS_data[ts,:x], TS_data[ts,:y])
            lineNo = TS_data[ts,:line]
            transfer = TS_data[ts, :transfer]
            dwel_time = TS_data[ts, :dweltime]
            if lyr <= size(timetables[lineNo])[1] 
                dep_time = timetables[lineNo][lyr,"$ts"]
                arr_time = dep_time .- dwel_time
                ts_dum = dep_ts[lyr, ts]
                ts_nodes[ts_dum] = TransitStation(ts, coord, lyr, dep_time, arr_time, lineNo, transfer, dwel_time,  0.0)
            else
                ts_dum = dep_ts[lyr, ts]
                dep_ts[lyr, ts] = 0
                ts_nodes[ts_dum] = NaN
            end
        end
    end
    return ts_nodes
end

function find_transfer_arcs(ts1_dm::Int64, ts_nodes::Vector, dep_ts::Matrix; max_transfer_time = 10.0)
    ts1 = ts_nodes[ts1_dm]
    transfers = ts1.transfer
    arcs = Vector{NamedTuple{(:ts1, :ts2, :tt), Tuple{Int64, Int64, Float64}}}()
    for ts2_ID in transfers
        ts2_dummies = dep_ts[:,ts2_ID]
        filter!(x -> x != 0, ts2_dummies)
        for ts2_dm in ts2_dummies
            ts2 = ts_nodes[ts2_dm]
            tt = ts2.dep_time - ts1.dep_time
            if tt > 0 && tt <= max_transfer_time
                push!(arcs, (ts1 = ts1_dm, ts2 = ts2_dm, tt = tt))
            end
        end
    end
    return arcs
end

function find_direct_transfer_arcs(ts_nodes::Vector, dep_ts::Matrix, timetables::Vector, n_tsline::Int64, n_layer::Int64, n_ts::Int64, max_transfer_time::Float64)
    arcs = Vector{NamedTuple{(:ts1, :ts2, :tt), Tuple{Int64, Int64, Float64}}}()
    transfer_arcs = Vector{NamedTuple{(:ts1, :ts2, :tt), Tuple{Int64, Int64, Float64}}}()

    # a range of transit stops at different lines
    stops_line = [size(timetables[i])[2]-1 for i in 1:n_tsline] # the number of stops at each line
    start_indices = cumsum([1;stops_line[1:end-1]])
    ranges_list = [start: start + length - 1 for (start, length) in zip(start_indices, stops_line)]

    for lyr in 1:n_layer
        for l in 1:n_tsline
            ts_range = ranges_list[l] + n_ts*(lyr-1)
            if dep_ts[lyr,ranges_list[l].start] == 0
                continue
            end
            direction = timetables[l].Direction[lyr]
            
            for ts1 in ts_range
                # arcs within the same line
                for ts2 in ts_range  
                    if direction == 1 && ts2 > ts1
                        tt = ts_nodes[ts2].dep_time - ts_nodes[ts1].dep_time
                        push!(arcs, (ts1 = ts1, ts2 = ts2, tt = tt))
                    elseif direction == 0 && ts1 < ts2
                        tt = ts_nodes[ts1].dep_time - ts_nodes[ts2].dep_time
                        push!(arcs, (ts1 = ts2, ts2 = ts1, tt = tt))
                    end
                end
                
                # trasfer arcs to different lines
                if isempty(ts_nodes[ts1].transfer)
                    continue
                else
                    transfer_arcs_node = find_transfer_arcs(ts1, ts_nodes, dep_ts, max_transfer_time = max_transfer_time)
                    if !isempty(transfer_arcs_node)
                        transfer_arcs = [transfer_arcs; transfer_arcs_node]
                    end
                end
            end
        end
    end
    return arcs, transfer_arcs
end

function gen_traveltime_nowaiting(traveltime_ts::Matrix, transfer_arcs::Vector, node_o::Int64, node_d::Int64, path::Vector)
    l = length(path)
    time = traveltime_ts[node_o, node_d]
    for i in 1:l-1
        if (path[i],path[i+1]) ∈ transfer_arcs
            time -= traveltime_ts[path[i],path[i+1]]
        end
    end
    return time
end

function init_ts_network(n_dummies::Int64, transfer_arcs::Vector, dep_ts::Matrix, ts_graph::SimpleWeightedDiGraph)
    traveltime_ts = Inf * ones(n_dummies, n_dummies)
    traveltime_ts_nowaiting = Inf * ones(n_dummies, n_dummies)
    paths_ts = Dict{Tuple, Vector}()
    arcs_transfer = get_arcs.(transfer_arcs)
    for node_o in 1:n_dummies
        if node_o ∉ dep_ts
            continue
        end
        r = dijkstra_shortest_paths(ts_graph, node_o)
        traveltime_ts[node_o,:] = r.dists
        traveltime_ts_nowaiting[node_o,:] = r.dists
        for node_d in 1:n_dummies
            if node_d ∉ dep_ts
                continue
            end
            path = enumerate_paths(r,node_d)
            phy_ts1 = findfirst(x->x==node_o, dep_ts)[2]
            phy_ts2 = findfirst(x->x==node_d, dep_ts)[2]
            if node_o == node_d
                traveltime_ts[node_o, node_d] == 1.0
            end
            if phy_ts1 == phy_ts2
                continue
                # traveltime_ts[node_o, node_d] = Inf
            elseif (node_o, node_d) ∈ arcs_transfer
                traveltime_ts_nowaiting[node_o, node_d] = 0.0
                continue
            elseif length(path) < 2
                continue
            elseif length(path) == 3
                continue
            elseif check_adjacent_arcs(path, arcs_transfer)
                continue
            else
                paths_ts[(node_o, node_d)] = path
                traveltime_ts_nowaiting[node_o,node_d] = gen_traveltime_nowaiting(traveltime_ts, transfer_arcs, node_o, node_d, path)
            end
        end
    end
    return traveltime_ts, traveltime_ts_nowaiting, paths_ts
end

function shortest_arcs!(paths_ts::Dict, traveltime_ts::Matrix, n_dummies::Int64, ts_nodes::Vector, dep_ts::Matrix)
    for i in 1:n_dummies
        visited = [] 
        for j in 1:n_dummies
            if isa(ts_nodes[j], TransitStation)
                if isempty(ts_nodes[j].transfer)
                    continue
                elseif j in visited
                    continue
                else
                    phy_ts2 = findfirst(x->x==j,dep_ts)[2]
                    compares = hcat(dep_ts[:,[phy_ts2; ts_nodes[j].transfer]]...)[1,:]
                    visited = [visited;compares]
                    filter!(x->x!=0, compares)
                    select = compares[argmin(traveltime_ts[i,compares])]
                    filter!(x->x!=select,compares)
                    for arc in [(i,k) for k in compares]
                        delete!(paths_ts, arc)
                    end
                end
            end
        end
    end
end

function remove_same_deptime!(paths_ts::Dict, traveltime_ts::Matrix, ts_nodes::Vector, dep_ts::Matrix, TS_data::DataFrame)
    phy_transfer = Dict()
    for i in 1:size(TS_data)[1]
        ts = TS_data[i,:]
        if !isempty(ts.transfer) 
            if i ∉ vcat(values(phy_transfer)...)
                phy_transfer[i] = [i; ts.transfer]
            end
        end
    end

    same_deptime_pairs = []
    for (k,v) in phy_transfer
        compares = collect(combinations(v,2))
        for com in compares
            ts1_vec = filter!(x->x!=0, dep_ts[:,com[1]])
            ts2_vec = filter!(x->x!=0, dep_ts[:,com[2]])
            for ts1 in ts1_vec
                for ts2 in ts2_vec
                    if ts_nodes[ts1].dep_time == ts_nodes[ts2].dep_time
                        push!(same_deptime_pairs, (ts1, ts2))
                    end
                end
            end
        end
    end

    ts_arcs = collect(keys(sort(paths_ts)))
    for p in same_deptime_pairs
        for arc1 in show_ts_arcs(p[1], ts_arcs)
            for arc2 in show_ts_arcs(p[2], ts_arcs)
                if arc1[2] == arc2[2]
                    if traveltime_ts[arc1...] < traveltime_ts[arc2...]
                        delete!(paths_ts, arc2)
                    elseif traveltime_ts[arc1...] > traveltime_ts[arc2...]
                        delete!(paths_ts, arc1)
                    else
                        delete!(paths_ts, arc2)
                    end
                end
            end
        end
    end

end

function create_di_graph(n_nodes::Int64, direct_arcs::Vector, transfer_arcs::Vector)
    graph = SimpleWeightedDiGraph(n_nodes)
    for arc in direct_arcs
        add_edge!(graph, arc.ts1, arc.ts2, arc.tt)
    end
    for arc in transfer_arcs
        add_edge!(graph, arc.ts1, arc.ts2, arc.tt)
    end
    return graph
end

"""
Check if there are adjacent transfer links
"""
function check_adjacent_arcs(path::Vector, transfer_arcs::Vector)
    for i in 1:length(path)-2
        ts1 = path[i]
        ts2 = path[i+1]
        ts3 = path[i+2]
        if (ts1,ts2) ∈ transfer_arcs
            if (ts2,ts3) ∈ transfer_arcs
                return true
            end
        end
    end
    return false
end

function add_dummy_traveltime_ts(traveltime::Matrix, ts_rng::UnitRange, arcs_ts::Vector, nodes::Vector)
    traveltime_ts = Inf .* ones(ts_rng[end], ts_rng[end]) # Travel time for transit network
    traveltime_ts[ts_rng, ts_rng] = traveltime
    for arc in arcs_ts
        traveltime_ts[arc...] -= nodes[arc[2]].dweltime
    end
    return traveltime_ts
end

function shift_ts_network(ts_network_noshift, ts_rng::UnitRange, nodes::Vector)
    offset = ts_rng[1] - 1
    arcs_ts = ts_network_noshift.arcs .+ offset   
    ts_nodes = Vector(undef, ts_rng[end])
    ts_nodes[ts_rng] = ts_network_noshift.nodes                       
    traveltime_ts = add_dummy_traveltime_ts(ts_network_noshift.traveltime, ts_rng, arcs_ts, nodes)
    traveltime_ts_nowaiting = add_dummy_traveltime_ts(ts_network_noshift.traveltime_nowaiting, ts_rng, arcs_ts, nodes)
    transfer_arcs = ts_network_noshift.transfer_arcs .+ offset
    map = ts_network_noshift.map .+ offset
    paths_ts = Dict(arc => ts_network_noshift.paths[arc-offset].+offset for arc in arcs_ts)
    ts_network = TransitNetwork(ts_nodes, 
                                arcs_ts, 
                                traveltime_ts, 
                                traveltime_ts_nowaiting, 
                                transfer_arcs, 
                                map,
                                paths_ts)

    return ts_network
end

function gen_layer_arcs(n_layer::Int64, ts_arcs::Vector, dep_ts::Matrix)
    layer_arcs = Dict(i => [] for i in 1:n_layer)
    for arc in ts_arcs
        for l in 1:n_layer
            if arc[1] in dep_ts[l,:]
                push!(layer_arcs[l], arc)
            end
        end
    end
    return layer_arcs
end


function construct_ts_network(n_ts::Int64, n_tsline::Int64, n_layer::Int64, ts_nodes::Vector, TS_data::DataFrame, timetables::Vector{DataFrame}, max_transfer_time::Float64)
    n_dummies = n_ts*n_layer
    dep_ts = permutedims(reshape(1:n_dummies, n_ts, n_layer)) # row: layer number; column: transit stops

    # get the direct arcs wihtin the same departure/line and the transfer arcs
    direct_arcs, transfer_arcs = find_direct_transfer_arcs(ts_nodes, dep_ts, timetables, n_tsline, n_layer, n_ts, max_transfer_time)

    # Create a graph
    ts_graph = create_di_graph(n_dummies, direct_arcs, transfer_arcs)
    # Initialize the network by applying dijkstra_shortest_paths algorithm but delete arcs that connect to dummies
    traveltime_ts, traveltime_ts_nowaiting, paths_ts = init_ts_network(n_dummies, transfer_arcs, dep_ts, ts_graph)
    # Keep the one with shortest travel time
    shortest_arcs!(paths_ts, traveltime_ts, n_dummies, ts_nodes, dep_ts)
    # Remove arcs if departure dummy nodes has the same departure time
    remove_same_deptime!(paths_ts, traveltime_ts, ts_nodes, dep_ts, TS_data)

    ts_arcs = collect(keys(sort(paths_ts)))
    layer_arcs = gen_layer_arcs(n_layer, ts_arcs, dep_ts)

    
    ts_network_noshift = TransitNetwork(ts_nodes, 
                                ts_arcs, 
                                traveltime_ts, 
                                traveltime_ts_nowaiting, 
                                get_arcs.(transfer_arcs), 
                                dep_ts,
                                paths_ts)
       
    return ts_network_noshift
end

