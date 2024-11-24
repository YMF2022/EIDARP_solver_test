"""
    Create an IEDARP instance with trimmed bus, transit and walking arcs.
"""
function preprocess(folder::String, NETWORK::String; set_dummy_recharge = false, max_recharge = 3, set_detour = 0.0)

    # Read parameters.csv file 
    TS_data = CSV.read("data/" * NETWORK * "/" * NETWORK * "-network.csv", DataFrame, header=true)
    TS_data.transfer = parse_column.(TS_data.transfer)
    n_ts = size(TS_data)[1]
    n_tsline = maximum(TS_data.line)
    n_c = countlines("$folder/customers.csv") - 1
    n_cgr = countlines("$folder/chargers.csv") - 1
    n_depot = countlines("$folder/depots.csv") - 1
    μ, max_wlk_dist, wlk_speed, dwel_time, n_dummy_cgr, detour_factor, max_wait_time,
    t_0, t_end = CSV.read("$folder/other_parameters.csv", DataFrame)[1, :]
    if set_dummy_recharge
        n_dummy_cgr = max_recharge
    end

    if set_detour != 0.0
        detour_factor = set_detour
    end

    # Read timetables of transit lines
    timetables = [CSV.read("$folder/timetable_line$i.csv", DataFrame) for i in 1:n_tsline]
    n_layer = maximum(nrow.(timetables)) # get the number of layers for the network

    # Read buses
    buses, n_k = readbusdata(folder)

    # Read depots
    depots = CSV.read("$folder/depots.csv", DataFrame)

    # Construct nodes: a ector of different type of nodes
    nnode = 2 * n_c + n_ts * n_layer + n_cgr * n_dummy_cgr + 2 * n_depot # number of nodes: start/end depot are placed to the end
    ts_rng = 2*n_c+1:2*n_c+(n_ts)*n_layer # range of transit stop ndoes
    cgr_rng = 2*n_c+(n_ts)*n_layer+1:2*n_c+(n_ts)*n_layer+n_cgr*n_dummy_cgr # range of charging nodes
    nodes, ts_map, cgr_map, L = constructnodes(folder, TS_data, timetables, nnode, n_c, n_ts, n_cgr, n_layer, ts_rng, cgr_rng,
        depots, n_dummy_cgr, detour_factor, buses[1].speed/60, t_0, t_end, μ, n_k)

    # Define arcs with accessible links
    ts_network_noshift = construct_ts_network(n_ts, n_tsline, n_layer, nodes[ts_rng], TS_data, timetables, max_wait_time)
    # Shift the dummies index 
    ts_network = shift_ts_network(ts_network_noshift, ts_rng, nodes)

    coordsall = get_coord(nodes)
    distmatrix = pairwise(Euclidean(), eachrow(coordsall), eachrow(coordsall))
    traveltime = distmatrix ./ (buses[1].speed / 60)
    timewindow = vcat(gettimewindow.(nodes)...)
    arcs_bus = generate_bus_arcs(timewindow, traveltime, ts_map, max_wait_time, nnode, n_c, n_cgr, n_dummy_cgr, n_ts, n_layer, ts_rng, cgr_rng, n_depot)
    arcs_walk, cusnearTS, walktime = customer_near_TS(distmatrix, ts_map, max_wlk_dist, wlk_speed / 60, n_c, n_layer, max_wait_time, nodes)
    μ = getservicetime.(nodes)
    nodes_bus_inout, nodes_ts_inout, nodes_walk_inout = get_arcs_inout(arcs_bus, ts_network.arcs, arcs_walk, nnode, ts_rng, n_c)
    eidarp = EIDARP(nnode, nodes, arcs_bus, ts_network.arcs, arcs_walk, buses, n_k, n_c, n_ts, n_tsline, n_cgr, n_depot, ts_rng, cgr_rng,
        t_end, timewindow, μ, L, max_wait_time, ts_map, cgr_map, distmatrix, traveltime, ts_network.traveltime, walktime, cusnearTS,
        nodes_bus_inout, nodes_ts_inout, nodes_walk_inout)
    return eidarp, ts_network, ts_network_noshift  
end

# Get service time
getservicetime(node::Node) = node.μ

# Get timewindows
gettimewindow(node::Node) = [node.timewindow[1] node.timewindow[2]]
gettimewindow(node::TransitStation) = [node.dep_time node.arr_time]

function readbusdata(folder::String)
    bus_data = CSV.read("$folder/buses.csv", DataFrame)
    buses = Vector{Bus}()
    for row in eachrow(bus_data)
        type = row.type
        cap = row.capacity
        speed = row.speed
        β = row.consumption
        maxbattery = row.maxBattery
        depot = row.depot
        push!(buses, Bus(type, cap, speed, β, maxbattery, depot))
    end
    return buses, length(buses)
end

function constructnodes(folder::String, TS_data::DataFrame, timetables::Vector{DataFrame}, nnode::Int64, n_c::Int64, n_ts::Int64,
    n_cgr::Int64, n_layer::Int64, ts_rng::UnitRange, cgr_rng::UnitRange,
    depots::DataFrame, n_dummy_cgr::Int64, detour_factor::Float64,
    v_k::Float64, t_0::Float64, t_end::Float64, μ::Float64, n_k::Int64)

    nodes = Vector{Node}(undef, nnode)
    L = [] # initialize maximum ride time for each customer
    # Customers
    cus_data = CSV.read("$folder/customers.csv", DataFrame)
    n_col_cus = ncol(cus_data)
    direct_ridetime = (sqrt.((cus_data.x_o - cus_data.x_d).^2 + (cus_data.y_o - cus_data.y_d).^2))./v_k
    for r in 1:n_c
        maxridetime = direct_ridetime[r] * detour_factor
        push!(L, maxridetime)
        if n_col_cus < 9 # for data only with inbound users
            x_o, y_o, x_d, y_d, ear_dep, late_dep, _ = cus_data[r, :]
            ear_arr = ear_dep + direct_ridetime[r] + μ
            late_arr = late_dep + maxridetime + μ
            nodes[r] = Pickup(r, n_c + r, (x_o, y_o), (ear_dep, late_dep), μ, 1, maxridetime, [], [], [], 0, 0, 0)
            nodes[r+n_c] = Droppoff(n_c + r, r, (x_d, y_d), (ear_arr, late_arr), 0.0, -1, maxridetime, [], [], 0, 0, 0)
        else # for data with inbound and outbound users
            x_o, y_o, x_d, y_d, ear_dep, late_dep, ear_arr, late_arr, isinbound, _ = cus_data[r, :]
            if isinbound == 1.0
                ear_arr = ear_dep + direct_ridetime[r] + μ
                late_arr = late_dep + maxridetime + μ
            else
                ear_dep = ear_arr - maxridetime -μ
                late_dep = late_arr - direct_ridetime[r] - μ
            end
            nodes[r] = Pickup(r, n_c + r, (x_o, y_o), (ear_dep, late_dep), μ, 1, maxridetime, [], [], [], 0, 0, 0)
            nodes[r+n_c] = Droppoff(n_c + r, r, (x_d, y_d), (ear_arr, late_arr), 0.0, -1, maxridetime, [], [], 0, 0, 0)
        end
    end

    # Transit stops
    ts_map = permutedims(reshape(ts_rng, n_ts, n_layer)) # row: layer number; column: transit stops
    for lyr in 1:n_layer
        for ts in 1:n_ts
            coord = (TS_data[ts, :x], TS_data[ts, :y])
            lineNo = TS_data[ts, :line]
            transfer = TS_data[ts, :transfer]
            dwel_time = TS_data[ts, :dweltime]
            if lyr <= size(timetables[lineNo])[1]
                dep_time = timetables[lineNo][lyr, "$ts"]
                arr_time = dep_time .- dwel_time
                ts_dum = ts_map[lyr, ts]
                cus_board = zeros(Bool, n_k, n_c)
                cus_alight = zeros(Bool, n_k, n_c)
                nodes[ts_dum] = TransitStation(ts_dum, ts, coord, lyr, dep_time, arr_time, lineNo, transfer, dwel_time, 0.0, cus_board, cus_alight , zeros(Int64, n_k), zeros(Int64, n_k))
            else
                ts_dum = ts_map[lyr, ts]
                ts_map[lyr, ts] = 0
                nodes[ts_dum] = NaN
            end
        end
    end

    if maximum(vcat(gettimewindow.(nodes[1:ts_rng[end]])...)[:, 2]) > t_end
        t_end = maximum(vcat(gettimewindow.(nodes[1:ts_rng[end]])...)[:, 2]) + 15.0
        @warn "End of operation time changed to: $t_end"
    end

    # Chargers
    cgr_data = CSV.read("$folder/chargers.csv", DataFrame)
    cgr_map = reshape(cgr_rng, n_dummy_cgr, n_cgr)
    for c in 1:n_cgr
        coord = (cgr_data[c, :x], cgr_data[c, :y])
        cgr_speed = cgr_data[c, :charging_speed] / 60 # convert kWh/h to kWh/min
        for i in 1:n_dummy_cgr
            nodes[cgr_map[i, c]] = Charger(cgr_map[i, c], c, coord, (t_0, t_end), cgr_speed, 1.0, 0)
        end
    end
    cgr_coords = Matrix(cgr_data[!, [:x, :y]])

    # Depots
    n_depot = size(depots)[1]
    for i in 1:n_depot
        idx = cgr_rng[end] + i
        nodes[idx] = Depot(idx, i, (depots[i, :].x, depots[i, :].y), (t_0, t_end), 0.0, 0)
        nodes[idx+n_depot] = Depot(idx + n_depot, i, (depots[i, :].x, depots[i, :].y), (t_0, t_end), 0.0, 0)
    end
    @warn("service time only for pickup nodes: 0.5, and charging station: 1.0")
    return nodes, ts_map, cgr_map, L, TS_data
end

tw_check(tw1::Vector{Float64}, tw2::Vector{Float64}, tt::Float64) = ~((tt + tw1[1] > tw2[2]) | (tt + tw1[2] < tw2[1]))
tw_check_l(ear1::Float64, late2::Float64, tt::Float64) = ~(tt + ear1 > late2) # check if its later, ealier is allowed

function find_TS_arcs(stops1::Vector{Int64}, stops2::Vector{Int64}; same_dep=0)
    if same_dep == 0
        arcs = Vector{Tuple}(undef, 2*size(stops1)[1] * size(stops2)[1])
        count = 1
        for ts1 in stops1
            for ts2 in stops2
                arcs[count] = (ts1, ts2); count += 1
                arcs[count] = (ts2, ts1); count += 1
            end
        end
    else
        arcs = Vector{Tuple}(undef, size(stops1)[1] * (size(stops1)[1] - 1))
        count = 1
        for ts1 in stops1
            for ts2 in stops2
                if ts1 != ts2
                    arcs[count] = (ts1, ts2)
                    count += 1
                end
            end
        end
    end
    return arcs
end

function generate_bus_arcs(timewindow::Matrix, traveltime::Matrix, ts_map::Matrix, max_wait_time::Float64, nnode::Int64,
    n_c::Int64, n_cgr::Int64, n_dummy_cgr::Int64, n_ts::Int64, n_layer::Int64, ts_rng::UnitRange,
    cgr_rng::UnitRange, n_depot::Int64)
    tw = timewindow
    A_B_length = 300000
    ab_len = 1
    A_B = Vector{Tuple}(undef, A_B_length)

    ### Origin depot to: cus_p, TS, Chargers, destination depot ###
    for ori in cgr_rng[end]+1:cgr_rng[end]+n_depot
        A_B[ab_len:ab_len+n_c-1] = [(ori, i) for i in 1:n_c]
        ab_len += n_c
        A_B[ab_len:ab_len+n_ts*n_layer-1] = [(ori, i) for i in ts_rng]
        ab_len += n_ts * n_layer # to all TS
        A_B[ab_len:ab_len+n_cgr*n_dummy_cgr-1] = [(ori, i) for i in cgr_rng]
        ab_len += n_cgr * n_dummy_cgr
        A_B[ab_len] = (ori, ori + n_depot)
        ab_len += 1
    end

    for c = 1:n_c
        ### Cus_p to: Cus_p, cus_d, TS_D, TS_P ###
        ### Cus_D to: cus_p, cus_d, TS_D, TS_P, Chargers, destination depot ###      
        for j in 1:n_c

            if j != c
                # Cus_p->Cus_p
                if tw_check_l(timewindow[c, 1], timewindow[j, 2], traveltime[c, j])
                    A_B[ab_len] = (c, j)
                    ab_len += 1
                end
                # Cus_D-> Cus_P
                if tw_check_l(timewindow[n_c+c, 1], timewindow[j, 2], traveltime[n_c+c, j]) # arrives early is allowed
                    A_B[ab_len] = (n_c + c, j)
                    ab_len += 1
                end
                # Cus_D->Cus_D
                if tw_check(timewindow[n_c+c, :], timewindow[n_c+j, :], traveltime[n_c+c, n_c+j])
                    A_B[ab_len] = (n_c + c, n_c + j)
                    ab_len += 1
                end
            end
            # Cus_p->Cus_D
            if tw_check(timewindow[c, :], timewindow[n_c+j, :], traveltime[c, n_c+j])
                A_B[ab_len] = (c, j + n_c)
                ab_len += 1
            end

        end

        # Cus_D->Chargers
        A_B[ab_len:ab_len+n_cgr*n_dummy_cgr-1] = [(n_c + c, j) for j in cgr_rng]
        ab_len += n_cgr * n_dummy_cgr
        # Cus_D->destination depot
        A_B[ab_len:ab_len+n_depot-1] = [(n_c + c, j) for j in nnode-n_depot+1:nnode]
        ab_len += n_depot

        for dp in 1:n_layer
            for tsd in ts_map[dp, :]
                # Cus_p -> TS_D/TS_P
                if tw_check_l(timewindow[c, 1], timewindow[tsd, 2] + max_wait_time, traveltime[c, tsd])
                    A_B[ab_len] = (c, tsd)
                    ab_len += 1
                end

                # TS_D/TS_P -> Cus_p
                A_B[ab_len] = (tsd, c)
                ab_len += 1

                # TS_D/TS_P ->Cus_D
                if tw_check_l(timewindow[tsd, 1] - max_wait_time, timewindow[n_c+c, 2], traveltime[n_c+c, tsd])
                    A_B[ab_len] = (tsd, n_c + c)
                    ab_len += 1 # might be removed
                end

                # Cus_D -> TS_D/TS_P
                A_B[ab_len] = (n_c + c, tsd)
                ab_len += 1 # might be removed
            end
        end

        # chargers -> Cus_p
        A_B[ab_len:ab_len+n_cgr*n_dummy_cgr-1] = [(cgr, c) for cgr in cgr_rng]
        ab_len += n_cgr * n_dummy_cgr
    end

    for dp = 1:n_layer
        ### TS_D to: cus_p, cus_d, TS_D, TS_P, Chargers, destination depot ###
        ### TS_P: cus_p, cus_d, TS_P, TS_D ###
        stops1 = ts_map[dp, :] # all transit stops at this departure dp
        for dp2 = dp:n_layer
            stops2 = ts_map[dp2, :]
            if dp == dp2 # (TS_D+TS_P) to (TS_D+TS_P) in the same layer
                arcs = find_TS_arcs(stops1, stops2, same_dep=1)
                for arc in arcs
                    if tw_check_l(timewindow[arc[1], 2], timewindow[arc[2], 2] + max_wait_time, traveltime[arc[1], arc[2]])
                        # A_B[ab_len] = arc; ab_len += 1
                    end
                end
            else # (TS_D+TS_P) to (TS_D+TS_P) different departures
                arcs = find_TS_arcs(stops1, stops2, same_dep=0)
                for arc in arcs
                    if tw_check_l(timewindow[arc[1], 2], timewindow[arc[2], 2] + max_wait_time, traveltime[arc[1], arc[2]])
                        A_B[ab_len] = arc
                        ab_len += 1
                    end
                end
            end
        end

        for ts in ts_map[dp, :]
            # TS ->Chargers
            A_B[ab_len:ab_len+n_cgr*n_dummy_cgr-1] = [(ts, j) for j in cgr_rng]
            ab_len += n_cgr * n_dummy_cgr
            # TS ->destination depot
            A_B[ab_len:ab_len+n_depot-1] = [(ts, j) for j in nnode-n_depot+1:nnode]
            ab_len += n_depot
        end
    end

    ### Chargers to: cus_p, TS_P, destination ###
    # Chargers->destination depot
    A_B[ab_len:ab_len+n_cgr*n_dummy_cgr*n_depot-1] = [(cgr, j) for cgr in cgr_rng for j in nnode-n_depot+1:nnode]
    ab_len += n_cgr * n_dummy_cgr * n_depot
    # Chargers -> ts_rng
    A_B[ab_len:ab_len+n_cgr*n_dummy_cgr*length(ts_rng)-1] = [(j, ts) for j in cgr_rng for ts in ts_rng]
    ab_len += n_cgr * n_dummy_cgr * length(ts_rng)
    ab_len > A_B_length ? error("Please increase the length for A_B") : nothing
    A_B = A_B[1:ab_len-1]
    return A_B
end

function customer_near_TS(dist_matrix::Matrix, ts_map::Matrix, max_wlk_dist::Float64, wlk_speed::Float64, n_c::Int64, n_layer::Int64, max_wait_time, nodes::Vector{Node})
    arcs_walk = Vector{Tuple}()
    R_G_1 = Vector{Int64}()
    R_G_2 = Vector{Int64}()
    walktime = ones(2n_c, size(dist_matrix)[1]) * 100
    for c in 1:n_c
        for dp in 1:n_layer
            for ts in ts_map[dp, :] # customers' origin to TS
                if dist_matrix[c, ts] <= max_wlk_dist
                    walktime[c, ts] = dist_matrix[c, ts] / wlk_speed
                    if (nodes[ts].dep_time - max_wait_time <= nodes[c].timewindow[2] + walktime[c, ts]) && (nodes[ts].dep_time >= nodes[c].timewindow[1] + walktime[c, ts])
                        push!(arcs_walk, (c, ts))
                        push!(nodes[c].close_ts, ts)
                        c ∉ R_G_1 ? push!(R_G_1, c) : nothing
                    end
                end

                if dist_matrix[n_c+c, ts] <= max_wlk_dist # customers' destination to TS
                    walktime[n_c+c, ts] = dist_matrix[n_c+c, ts] / wlk_speed
                    if nodes[ts].arr_time + walktime[c+n_c, ts] <= nodes[c+n_c].timewindow[2]
                        push!(arcs_walk, (n_c + c, ts))
                        push!(nodes[c+n_c].close_ts, ts)
                        c ∉ R_G_2 ? push!(R_G_2, c) : nothing
                    end
                end
            end
        end
    end
    return arcs_walk, [R_G_1, R_G_2], walktime
end

function get_arcs_inout(arcs_bus::Vector, arcs_ts::Vector, arcs_walk::Vector, nnode::Int64, ts_rng::UnitRange, n_c::Int64)
    nodes_bus_inout = Dict(node => [[], []] for node in 1:nnode)
    for arc in arcs_bus
        push!(nodes_bus_inout[arc[1]][2], arc[2]) # push arc[2] to the out of arc[1]
        push!(nodes_bus_inout[arc[2]][1], arc[1]) # push arc[1] to the in of arc[2]
    end

    nodes_ts_inout = Dict(node => [[], []] for node in ts_rng)
    for arc in arcs_ts
        push!(nodes_ts_inout[arc[1]][2], arc[2]) # push arc[2] to the out of arc[1]
        push!(nodes_ts_inout[arc[2]][1], arc[1]) # push arc[1] to the in of arc[2]
    end

    nodes_walk_inout = Dict(c => [[], []] for c in 1:2n_c)
    for arc in arcs_walk
        if arc[1] ∈ 1:n_c
            push!(nodes_walk_inout[arc[1]][2], arc[2]) # push arc[2] to the out of arc[1]
        elseif arc[1] ∈ n_c+1:2n_c
            push!(nodes_walk_inout[arc[1]][1], arc[2]) # push arc[1] to the in of arc[2]
        end
    end

    return nodes_bus_inout, nodes_ts_inout, nodes_walk_inout
end

"""
 Function to parse strings into appropriate data types
"""
function parse_column(data::T) where {T<:Union{Int,String,Missing}}
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