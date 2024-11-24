include("k_nearest.jl")
include("remove_cus.jl")

# Init route when it is from used to not used
function init_route!(k::Int64, sol::Solution)
    # nnode = sol.params.num_node
    route = sol.routes[k]
    sol.usedbus[k] = false
    pathID = get_path(route)
    for n in pathID
        node = sol.N[n]
        if isa(node, TransitStation)
            route.earliest[n] = sol.N[n].dep_time - sol.params.maxwait
            route.latest[n] = sol.N[n].arr_time + sol.params.maxwait
            node.q[k] = 0
            node.bus_servetype[k] = 0
            node.cus_board[k,:] .= 0
            node.cus_alight[k,:] .= 0
        elseif isa(node, Pickup) | isa(node, Droppoff)
            route.earliest[n] = sol.N[n].timewindow[1]
            route.latest[n] = sol.N[n].timewindow[1]
        end
    end
    route.len = 0
    route.pathID .= 0
    route.node_pos .= 0
    route.suc .= 0
    route.pre .= 0
    route.suc[route.depot_o] = route.depot_d
    route.pre[route.depot_d] = route.depot_o
    route.cap .= 0
    # route.soc .= route.battery * initE
    route.repair = 0
end

function update_earliest_latest!(sol::Solution, k::Int64)
    if sol.usedbus[k] == false return end
    N = sol.N
    tt = sol.params.tt
    route = sol.routes[k]
    earliest = route.earliest
    latest = route.latest
    pre, suc = route.pre, route.suc
    cap = route.cap
    # Forward loop: update earliest timewindow and capacity
    i = suc[route.depot_o]
    while i != route.depot_d # while it has not readed the destination depot
        j = route.pre[i] # the predecessor of node i
        e_i, q_i = get_earlytw(N[i], k), get_load(N[i],k)
        earliest[i] = max(e_i, earliest[j] + tt[j,i] + N[j].μ) # Braecker et al. 2014 equation (17)
        cap[i] = cap[j] + q_i # Braecker et al. 2014 equation (18); q here is 1
        i = route.suc[i]
    end
    # Backward loop: update latest timewindow and max capacity
    i = pre[route.depot_d]
    while i != route.depot_o # while it has not reached the origin depot
        j = route.suc[i] # the successor of i
        l_i, q_i = get_latetw(N[i],k), get_load(N[i], k)
        latest[i] = min(l_i, latest[j]- tt[i,j]-N[i].μ) # Braecker et al. 2014 equation (19)
        # sol.maxcap[i] = max(0, sol.maxcap[j] + q_i) # Braecker et al. 2014 equation (20); q here is -1
        i = route.pre[i]
    end
end

# Construct a vector of route by predecessor and secuccessor
function construct_route!(N::Vector{Node}, route::Route)
    order_route!(route, N)
    # len = route.len
    route.A .= 0.0  # Arrival time at vertex i 
    route.B .= 0.0  # Beginning of service at vertex i 
    route.W .= 0.0  # Wait time before starting service at vertex i 
    route.D .= 0.0  # Departure time at vertex i 
    route.y .= 0.0    # Cummulative laod after leaving vertex i 
    route.F .= 0.0  # Slack time of each node
    route.F_min .= 0.0 # Minimum slack time required
    # route.soc .= route.soc[1]
    route.repair = 0
    return route
end

function order_route!(route::Route, N::Vector{Node})
    suc = route.suc
    len = 0
    depot_o, depot_d = route.depot_o, route.depot_d
    if suc[depot_o] != 0
        len = 1
        i = depot_o
        route.pathID[len] = i
        route.node_pos[i] = len
        while i != depot_d
            len += 1
            i = suc[i]
            route.pathID[len] = i
            route.node_pos[i] = len
        end
    else
        error("Route $(route.ID) not in use but ask to be ordred")
    end
    route.len = len
end

order_route!(sol::Solution) = for r in sol.routes[sol.usedbus] order_route!(r, sol.N) end

# It is used to check if all bus arcs are within eidarp.arcs_bus to be align with MILP solver
function check_route_in_arcs(sol::Solution)
    N = sol.N
    # route.path = Vector{Node}(undef, length(suc))
    routes =  sol.routes[sol.usedbus]
    for route in routes
        depot_o, depot_d = route.depot_o, route.depot_d
        suc = route.suc
        if suc[depot_o] != 0
            i = depot_o
            while i != depot_d
                if (i, suc[i]) ∉ sol.params.A_bus
                    @error("$((i, suc[i])) not in arcs bus")
                end
                i = suc[i]
            end
        else
            error("Route $(route.ID) not in use but ask to be ordred")
        end
    end
end

# Find different first- or last-mile ts give a ts
function find_different_ts(ts1::Int64, ts2::Int64, ts_pairs::Vector, firstorlast::Symbol)
    @assert firstorlast == :first ||  firstorlast == :last "firstorlast must be either :first or :last"
    if firstorlast == :first
        ts_other = [first for (first, second) in ts_pairs if second == ts2]
    else
        ts_other = [second for (first, second) in ts_pairs if first == ts1]
    end
    return ts_other
end

# Get the departure time of a node
function get_dep_time(node::Node, sol::Solution)
    if isa(node, Pickup) | isa(node, Droppoff) 
        if (node.walk_ts > 0) && isa(node, Pickup)
            # walk depature time only for pickup node
            return sol.N[node.walk_ts].dep_time - sol.params.A_walk[node.ID, node.walk_ts]
        elseif node.bus > 0
            idx = sol.routes[node.bus].node_pos[node.ID]
            return sol.routes[node.bus].D[idx]
        else
            @error "Node $(node.ID) does not walk/by bus"
        end
    else
        @error "get_dep_time has not defined for other node type"
    end
end

# Get arrival time of a node
function get_arr_time(node::Node, sol::Solution)
    if isa(node, Pickup) | isa(node, Droppoff)
        if (node.walk_ts > 0) && isa(node, Droppoff)
            # walk arrival time only for dropp off node
            return sol.N[node.walk_ts].arr_time + sol.params.A_walk[node.ID, node.walk_ts]
        elseif node.bus > 0
            idx = sol.routes[node.bus].node_pos[node.ID]
            return sol.routes[node.bus].A[idx]
        else
            @error "Node $(node.ID) does not walk/by bus"
        end
    else
        @error "get_dep_time has not defined for other node type"
    end
end

# update node info from bus0 to bus
function update_route_of_node!(k::Int64, k0::Int64, sol::Solution)
    route = sol.routes[k]
    pathID = get_path(route)
    for node_id in pathID
        node = sol.N[node_id]
        if isa(node, Pickup) | isa(node, Droppoff)
            node.bus = k
        elseif isa(node, TransitStation)
            node.bus_servetype[k] = node.bus_servetype[k0]
            node.bus_servetype[k0] = 0
            node.q[k] = node.q[k0]
            node.q[k0] = 0
            node.cus_board[k,:] = copy(node.cus_board[k0,:])
            node.cus_board[k0,:] .= 0
            node.cus_alight[k,:] = copy(node.cus_alight[k0,:])
            node.cus_alight[k0,:] .= 0
        elseif isa(node, Charger)
            pre_n = route.pre[node_id]
            suc_n = route.suc[node_id]
            route.suc[pre_n] = suc_n
            route.pre[suc_n] = pre_n
            route.pre[node_id] = 0
            route.suc[node_id] = 0
            for key in keys(node.cgr_evt)
                if key[1] == k0 
                    # println("Remove $key for at node $node_id")
                    pop!(node.cgr_evt, key)
                end
            end
        end
        sol.N[node.ID] = node
    end
end


# From a set of customers, find those customers with common pairs
function find_common_pair(sol::Solution, customers::Vector{Int64})
    cus_first = customers[1]
    common_pairs = sol.N[cus_first].ts_pair
    cus_repair = [cus_first]
    common_pair_customers = keys(sol.params.common_pair)
    for i in 2:length(customers)
        cus2 = customers[i]
        if ((cus_first, cus2) ∈ common_pair_customers) | ((cus2,cus_first) ∈ common_pair_customers)
            current_common = intersect(common_pairs, sol.N[cus2].ts_pair)
            if !isempty(current_common)
                cus_repair = [cus_repair; cus2]
                common_pairs = current_common
            end
        end
    end
    return cus_repair, common_pairs
end

# Search best firstmile: p is customer ID, ts1_other is a list of feasible boarding ts
function best_firstmile(sol::Solution, p::Int64, ts1_other::Vector{Int64})
    # Remove and search the first mile
    best_sol, best_cost = nothing, Inf
    node_p = sol.N[p]
    for ts1 in ts1_other
        s_tmp_list, costs, _ = randomsearch_firstmile(sol, node_p, sol.N[ts1])
        s_tmp, cost = s_tmp_list[1], costs[1]
        if cost < best_cost
            best_cost = cost
            best_sol = s_tmp
        end
    end
    return best_sol, best_cost
end

function best_lastmile(sol::Solution, p::Int64, ts2_other::Vector{Int64})
    d = sol.N[p].dropoffID
    node_d = sol.N[d]
    best_sol, best_cost = nothing, Inf
    for ts2 in ts2_other
        s_tmp_list, costs, _ = randomsearch_lastmile(sol, sol.N[ts2], node_d)
        s_tmp, cost = s_tmp_list[1], costs[1]
        if cost < best_cost
            best_cost = cost
            best_sol = s_tmp
        end
    end
    return best_sol, best_cost
end

# Function to generate skewed weights based on position and value without using length(cost)
function generate_skewed_weights(cost::Vector{Float64}; decay_factor::Float64 = 0.6)
    base_weights = 1.0 ./ cost  # Reciprocal of the values
    position_weights = [decay_factor^i for i in eachindex(cost)]  # Exponential decay factor
    combined_weights = base_weights .* position_weights  # Combine both weights
    return combined_weights / sum(combined_weights)  # Normalize the weights
end

# Function to calculate weights based on the VNS iteration for degree of destroy
function calculate_weights(iter::Int64, total_iters::Int64, degree_range::Vector{Int64})
    n = length(degree_range)
    # Linearly interpolate between 1 (favoring high numbers) to 0 (favoring low numbers)
    alpha = 1 - iter / total_iters
    # Create weights
    weights = [alpha * (n - i + 1) + (1 - alpha) * i for i in 1:n]
    # Normalize weights to sum to 1
    normalized_weights = weights / sum(weights)
    return normalized_weights
end

function generate_degree_destroy_iter(num_iterations::Int64, degree_rnge::Vector{Int64})
    degree_destroy_samples = Int64[]
    # Sampling process
    for iter in 1:num_iterations
        # Calculate dynamic weights
        weights = calculate_weights(iter, num_iterations, degree_rnge)
        # Sample one element based on the current weights
        sampled_element = sample(degree_rnge, Weights(weights))
        push!(degree_destroy_samples, sampled_element)
    end
    return degree_destroy_samples
end

function generate_degree_destroy_equal_distribution(k_max::Int64, k_min::Int64, n_iter::Int64, result_folder::String; fig = false)
    # Calculate the total range size
    range_size = k_max - k_min + 1

    # Calculate the number of times each value should be repeated
    repeat_count = div(n_iter, range_size)
    remaining = n_iter % range_size

    # Generate the sequence
    distributed_values = Int64[]

    for value in k_max:-1:k_min
        # Append the repeated value
        append!(distributed_values, fill(value, repeat_count))
    end

    # Distribute the remaining slots (if any)
    if remaining > 0
        # append!(distributed_values, collect(k_max:-1:(k_max-remaining+1)))
        append!(distributed_values, fill(k_min, remaining))
    end

    if fig == true
        figure = plot(dpi=500)
        plot(collect(1:n_iter), distributed_values, legend= false, 
                    xlabel = "# iteration", ylabel = "# destroyed customer")
        savefig("$result_folder/destroydegree_stepped_distribution.png")    
    end

    return distributed_values
end