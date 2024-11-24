"""
Greedy search function including the following cases:
    1. Repaire the first mile, and customer walk at last-mile.
    2. Repaire from customers' origin to destination
"""


function greedysearch(sol::Solution, node1::Pickup, node2::Union{Droppoff, TransitStation}, isE::Bool)
    usedbus, tt = sol.usedbus, sol.params.tt
    buses = findall(usedbus)
    A_bus = sol.params.A_bus
    p, d = node1.ID, node2.ID
    # Outputs: best cost, repaired node1, repiared node2
    best_cost, best_sol = Inf, nothing
    max_len_sol = 1000
    solutions = Vector{Solution}(undef, max_len_sol)
    costs = Vector{Float64}(undef, max_len_sol)
    len_sol = 0
    for k in buses
        route_sol = sol.routes[k]
        if isa(node2, TransitStation) && isservedbus(node2, k)
            sol_1node, cost_1node = greedyrepair_1node(node1, node2, k, sol)
            if cost_1node != Inf
                len_sol += 1
                solutions[len_sol] = sol_1node
                costs[len_sol] = cost_1node
                if cost_1node < best_cost
                    best_sol = sol_1node
                    best_cost = cost_1node
                end
            end
            continue
        end
        depot_o, depot_d = route_sol.depot_o, route_sol.depot_d
        i = 0 # predecessor of node1 location
        _c = 0 # counter to avoid infinite loop
        # Step 1: search position for node1
        while i != depot_d # i is the predecessor of node1
            _c += 1; if _c > 1e6 error("Infinite loop") end
            i = i == 0 ? depot_o : i = route_sol.suc[i]
            if i == depot_d break end
            # Operation starts here
            if (i, p) ∉ A_bus continue end
            sol1 = copy(sol, k, [node1])
            # sol1 = copy(sol)
            route1 = sol1.routes[k]
            N1 = sol1.N; node_p = N1[p]
            res = check_first_position(i, node_p, route1, tt, N1)
            if res == :break break
            elseif res == :continue continue end
            suc = res; suc_p = suc[p]
            # Step 2: search poisiton for node2
            j = i # start with predecessor as p, j is a pointer as the predecessor of node2 location
            while j != depot_d
                if j == depot_d break end
                if suc[j] == j error("infinite loop at bus $k") 
                else j = suc[j] end # move to the next position
                suc_d = suc[j]
                if !(((j,d) ∈ A_bus) && ((d,suc_d) ∈ A_bus)) continue end
                sol2 = copy(sol1, k, [node1, node2])
                # sol2 = copy(sol1)
                route2 = sol2.routes[k]
                N2 = sol2.N 
                node_d = N2[d]; node_p = N2[p]
                if isa(node_d, TransitStation) update_servetype!(node_d, k, 2) end
                cus_cost_tmp = 0.0 # always needs to be updated
                res2 = check_last_position(cus_cost_tmp, p, j, node_d, route2, tt, N2, A_bus)
                if res2 == false break end
                # # Step 2.2: ride time check
                # if earliest2[d] - get_latetw(node_p, k) - node_p.μ > node_p.L break end
                # Step 3: eight-step evaluation scheme
                suc_p = route2.suc[p]
                insertpair!(node_p, node_d, i, suc_p, j, suc_d, k, sol2)            
                if !isfeasible(route2, sol2) continue end
                if route2 != sol2.routes[k]
                    error("routes doesnot match")
                end
                # Step 3.2: check if recharge is required
                if isE 
                    # isreacharge_success = recharge!(route2, sol2)
                    # if !isreacharge_success continue end
                end
                # Step 4.1: calculate customer travel cost  
                # We do not need to calcualte customer travel cost as it is part of the bus insertion cost
                len_sol += 1
                # cost = addtional_obj_cost(bus_cost, cus_cost, sol2)
                # cost = bus_cost
                # cost = objective_value(sol2, k, p, d)
                cost = objective_value(sol2)
                solutions[len_sol] = sol2
                costs[len_sol] = cost
                if cost < best_cost
                    best_cost = cost
                    best_sol = sol2
                end
            end
        end
    end
    sol_addbus, cost_addbus = addbus(sol, node1, node2)
    if cost_addbus != Inf
        len_sol += 1
        solutions[len_sol] = sol_addbus
        costs[len_sol] = cost_addbus
        if sol_addbus.N[p].bus == 0
            error("Bus is not assigned to customer $p for addbus")
        end
        if cost_addbus < best_cost
            best_cost = cost_addbus
            best_sol = sol_addbus
        end
    end
    if !isnothing(best_sol)
        solutions = solutions[1:len_sol]
        costs = costs[1:len_sol]
        order = sortperm(costs)
        solutions, costs = solutions[order], costs[order]
        if best_cost != costs[1] error("Best cost doesnot match") end
        if best_sol != solutions[1] error("Best solution doesnot match") end
        if best_sol.N[p].bus == 0
            error("Bus is not assigned to customer $p for addbus")
        end
        # return best_sol, best_cost
        return solutions, costs
    else
        # No feasible solution has been found
        return [sol], [Inf]
    end
end

"""
Greedy search function including the following cases:
    1. Repaire the last mile, and customer first mile is walking.
In this function, return three elements: route, node1, and node2
"""
function greedysearch(sol::Solution, node1::TransitStation, node2::Droppoff; diff_k = [], isE::Bool)
    usedbus, tt = sol.usedbus, sol.params.tt
    buses = findall(usedbus)
    A_bus = sol.params.A_bus
    p, d = node1.ID, node2.ID
    pickup = sol.N[node2.pickupID]
    _, ts1, cus_cost_firstmile, _ = firstmile_cost(pickup, sol)
    cus_cost_first = cus_cost_firstmile + sol.params.A_ts[(ts1, p)]
    
    # Outputs: best cost, repaired node1, repiared node2
    best_cost, best_sol = Inf, nothing
    max_len_sol = 1000
    solutions = Vector{Solution}(undef, max_len_sol)
    costs = Vector{Float64}(undef, max_len_sol)
    len_sol = 0
    for k in setdiff(buses, diff_k)
        if k == pickup.bus
            continue
        end
        route_sol = sol.routes[k]
        if isservedbus(node1, k)
            sol_1node, cost_1node = greedyrepair_1node(node1, node2, k, sol)
            if cost_1node != Inf
                len_sol += 1
                solutions[len_sol] = sol_1node
                costs[len_sol] = cost_1node
                if cost_1node < best_cost
                    best_sol = sol_1node
                    best_cost = cost_1node
                end
            end
            continue
        end
        depot_o, depot_d = route_sol.depot_o, route_sol.depot_d
        i = 0 # predecessor of node1 location
        _c = 0 # counter to avoid infinite loop
        # Step 1: search position for node1
        while i != depot_d # i is the predecessor of node1
            _c += 1; if _c > 1e6 error("Infinite loop") end
            i = i == 0 ? depot_o : i = route_sol.suc[i]
            if i == depot_d break end
            # Operation starts here
            if (i, p) ∉ A_bus continue end
            sol1 = copy(sol, k, [node1])
            # sol1 = copy(sol)
            route1 = sol1.routes[k]
            N1 = sol1.N
            node_p = N1[p]
            update_servetype!(node_p, k, 1)
            # TODO if d ∈ route.r greedysearch(N[d], k, sol)
            res = check_first_position(i, node_p, route1, tt, N1)
            if res == :break break
            elseif res == :continue continue end
            suc = res; suc_p = suc[p]
            # Step 2: search poisiton for node2
            j = i # start with predecessor as p, j is a pointer as the predecessor of node2 location
            while j != depot_d
                if suc[j] == j error("infinite loop") else j = suc[j] end # move to the next position
                if j == depot_d break end
                suc_d = suc[j]
                if !(((j,d) ∈ A_bus) && ((d,suc_d) ∈ A_bus)) continue end
                sol2 = copy(sol1, k, [node1,node2])
                # sol2 = copy(sol1)
                route2 = sol2.routes[k]
                N2 = sol2.N
                node_d = N2[d]; node_p = N2[p]
                update_servetype!(node_p, k, 1)
                res2 = check_last_position(cus_cost_first, p, j, node_d, route2, tt, N2, A_bus)
                if res2 == false break end
                # Step 3: eight-step evaluation scheme
                suc_p = route2.suc[p]
                insertpair!(node_p, node_d, i, suc_p, j, suc_d, k, sol2)
                if route2 != sol2.routes[k]
                    error("routes doesnot match")
                end
                feasibility = isfeasible(route2, sol2)
                if !feasibility
                    continue
                end
                # Step 4.1: calculate customer travel cost  
                # We do not need to calcualte customer travel cost as it is part of the bus insertion cost
                len_sol += 1
                cost = objective_value(sol2)
                # cost = objective_value(sol2, k, p, d)
                solutions[len_sol] = sol2
                costs[len_sol] = cost
                if cost < best_cost
                    best_cost = cost
                    best_sol = sol2
                end
            end
        end
    end
    sol_addbus, cost_addbus = addbus(sol, node1, node2)
    if cost_addbus != Inf
        len_sol += 1
        solutions[len_sol] = sol_addbus
        costs[len_sol] = cost_addbus
        if cost_addbus < best_cost
            best_cost = cost_addbus
            best_sol = sol_addbus
        end
    end
    if !isnothing(best_sol)
        solutions = solutions[1:len_sol]
        costs = costs[1:len_sol]
        order = sortperm(costs)
        solutions, costs = solutions[order], costs[order]
        if best_cost != costs[1] error("Best cost doesnot match") end
        if best_sol != solutions[1] error("Best solution doesnot match") end
        # return best_sol, best_cost
        return solutions, costs
    else
        # No feasible solution has been found
        return [sol], [Inf]
    end
end

# Repaire a customer with transit arc integration, where k is the k-nearest transit pair
function greedysearch(sol::Solution, cus_o::Pickup, ts_pair::Vector, isE::Bool)  
    # ts_pair = cus_o.ts_pair[k]
    if isempty(ts_pair) return [sol], [Inf] end
    max_len_sol = length(ts_pair) + 1
    solutions = Vector{Solution}(undef, max_len_sol)
    costs = Vector{Float64}(undef, max_len_sol)
    len_sol = 0
    sol_best, cost_best = nothing, Inf
    p = cus_o.ID
    d = cus_o.dropoffID
    for pair in ts_pair
        ts_o, ts_d = pair[1], pair[2]
        s_cur = copy(sol)
        s_cur = sol
        s_cur_list, cost_cur1_list = greedysearch(s_cur, s_cur.N[p], s_cur.N[ts_o])
        s_cur, cost_cur1 = s_cur_list[1], cost_cur1_list[1]
        if s_cur.N[p].bus_ts != ts_o continue end
        s_cur_list, cost_cur2_list = greedysearch(s_cur, s_cur.N[ts_d], s_cur.N[d], diff_k = [s_cur.N[p].bus])
        s_cur, cost_cur2 = s_cur_list[1], cost_cur2_list[1]
        # cost_cur = cost_cur1 + cost_cur2
        cost_cur = cost_cur2 + cost_cur1 - firstmile_cost(s_cur[p], s_cur)
        if cost_cur != Inf
            # only include feasible solution
            len_sol += 1
            costs[len_sol] = cost_cur
            solutions[len_sol] = s_cur
            if cost_cur < cost_best
                sol_best = s_cur
                cost_best = cost_cur
            end
        end
        # sol,_ = search(greedysearch, sol, cus_o, N[ts_o], N[ts_d], cus_d)
    end
    if !isnothing(sol_best)
        solutions = solutions[1:len_sol]
        costs = costs[1:len_sol]
        order = sortperm(costs)
        solutions, costs = solutions[order], costs[order]
        if sol_best != solutions[1] error("best solution doesnot match") end
        return solutions, costs
    else
        return [sol], [Inf]
    end
end

function check_first_position(i::Int64, node::Node, route::Route, tt::Matrix, N::Vector{Node})
    p = node.ID; k = route.ID
    suc, pre, earliest, cap = get_SPEC(route)
    suc_p = suc[i]
    ear_p = max(get_earlytw(node, k), earliest[i] + tt[i,p] + N[i].μ)
    if ear_p > get_latetw(node, k) return :break end
    if cap[i] + 1 > route.capacity return :continue end
    pre[p], pre[suc_p] = i, p
    suc[i], suc[p] = p, suc_p
    earliest[p] = ear_p; cap[p] = cap[i] + 1
    return suc
end

function check_last_position(cus_cost::Float64, p::Int64, j::Int64, node::Node, route2::Route, tt::Matrix, N::Vector{Node}, A_bus::Set)
    k = route2.ID; d = node.ID
    suc2, pre2, earliest2, cap2 = get_SPEC(route2)
    # Step 2.1: capacity check and timewindow check for j 
    if j != p # insert d Not directly after p: p-suc_p-...-pre_j-j-d
        if (p, suc2[p]) ∉ A_bus return false end
        m = p
        while m != d
            m == j ? m = d : m = suc2[m]
            m == d ? pre_m = j : pre_m = pre2[m]
            cus_cost += tt[pre_m, m]
            max_ridetime = isa(node, Droppoff) ? N[node.pickupID].L : Inf
            if cus_cost > max_ridetime 
                # println("max ride time $max_ridetime, but $cus_cost")
                return false 
            end
            cap2[m] = m == d ? cap2[pre_m] - 1 : cap2[pre_m] + 1
            if cap2[m] > route2.capacity return false end
            earliest2[m] = max(get_earlytw(N[m], k), earliest2[pre_m] + tt[pre_m,m] + N[pre_m].μ)                        
            if earliest2[m] > get_latetw(N[m], k) return false end
        end
    else # insert d directly after p: ...-p-d-...
        suc2[p] = d
        cus_cost += tt[p,d]
        if (p, d) ∉ A_bus return false end
        cap2[d] = cap2[p] - 1
        if cap2[d] > route2.capacity return false end
        earliest2[d] = max(get_earlytw(node, k), earliest2[p] + tt[p,d] + N[p].μ)
        if earliest2[d] > get_latetw(node, k) return false end
    end
    return cus_cost
end

"""
Greedy search function including the following cases:
    1. Repaire the first mile and customer's last mile by bus.
"""
# function greedysearch(node1::Pickup, node2::TransitStation, sol::Solution)
    
# end


# Only search one node where the first node is a pickup node
function greedyrepair_1node(node1::Pickup, node2::TransitStation, k::Int64, sol::Solution)
    A_bus, tt = sol.params.A_bus, sol.params.tt
    depot_o, depot_d = sol.routes[k].depot_o, sol.routes[k].depot_d
    p = node1.ID
    ts = node2.ID
    cus_cost = 0.0
    best_sol, best_cost = nothing, Inf
    i = 0 # predececor of node 
    while i != ts # i is the predecessor of node1
        i = i == 0 ? depot_o : i = sol.routes[k].suc[i]
        if i == ts break end
        # Operation starts here
        if (i, p) ∉ A_bus continue end
        sol1 = copy(sol, k, [node1, node2])
        # sol1 = copy(sol)
        route1 = sol1.routes[k]
        N1 = sol1.N; node_p = N1[p]
        node_d = N1[ts]
        suc, pre, earliest, cap = get_SPEC(route1)
        suc_p = suc[i]
        ear_p = max(get_earlytw(node_p, k), earliest[i] + tt[i,p] + N1[i].μ)
        if ear_p > get_latetw(node_p, k) break end
        if cap[i] + 1 > route1.capacity continue end
        pre[p], pre[suc_p] = i, p
        suc[i], suc[p] = p, suc_p
        earliest[p] = ear_p; cap[p] = cap[i] + 1
        j = p; pre_j = p
        update_servetype!(node_d, k, 2)
        while j != ts
            j = suc[j]
            earliest[j] = max(get_earlytw(N1[j], k), earliest[pre_j] + tt[pre_j,j] + N1[pre_j].μ)
            cap[j] = cap[pre_j] + 1
            if earliest[j] > get_latetw(N1[j], k) break end
            if cap[j] > route1.capacity break end
            pre_j = j
        end
        node_p.bus = k
        # bus_cost, cus_cost = insertpair!(node_p, node_d, i, suc_p, pre[ts], suc[ts], k, sol1)
        insertpair!(node_p, node_d, i, suc_p, pre[ts], suc[ts], k, sol1)
        if !isfeasible(route1, sol1) continue end
        if route1 != sol1.routes[k]
            error("routes doesnot match")
        end
        # cost = addtional_obj_cost(bus_cost, cus_cost, sol1)
        # cost = objective_value(sol1, k, p)
        cost = objective_value(sol1)
        if cost < best_cost
            best_cost = cost
            best_sol = sol1
        end
    end
    return best_sol, best_cost
end

# Only search one node, which is a dropoff node
function greedyrepair_1node(node1::TransitStation, node2::Droppoff, k::Int64, sol::Solution)
    A_bus = sol.params.A_bus
    tt = sol.params.tt
    depot_o, depot_d = sol.routes[k].depot_o, sol.routes[k].depot_d
    p = node1.ID
    d = node2.ID
    _, ts1, cus_cost, _ = firstmile_cost(sol.N[node2.pickupID], sol)
    cus_cost = cus_cost + sol.params.A_ts[(ts1,p)]
    best_sol, best_cost = nothing, Inf
    i = sol.routes[k].pre[p] # predececor of node2: start from ts
    while i != depot_d
        i = sol.routes[k].suc[i]
        # if i == depot_d break end
        # Operation starts here
        if (i, d) ∉ A_bus continue end
        sol1 = copy(sol, k, [node1, node2])
        # sol1 = copy(sol)
        route1 = sol1.routes[k]
        sol1.routes[k]
        if i == d error("$d already exists in route $k") end
        N1 = sol1.N
        node_p = N1[p]; node_d = N1[d] # pickup and dropoff node
        update_servetype!(node_p, k, 1)
        suc, pre, earliest, cap = get_SPEC(route1)
        if suc[d] == d error("$d suc is itself $k") end
        suc_d = suc[i]
        pre[d], pre[suc_d] = i, d
        suc[i], suc[d] = d, suc_d
        if suc[d] == d error("$d suc is itself at bus $k when pred is $i") end
        temp_cus_cost = cus_cost
        m = p
        while m != d
            m = suc[m]; pre_m = pre[m]
            temp_cus_cost += tt[pre_m, m]
            if temp_cus_cost > node2.L
                return sol,Inf 
            end
            cap[m] = m == d ? cap[pre_m] - 1 : cap[pre_m] + 1
            if cap[m] > route1.capacity 
                return sol,Inf 
            end
            earliest[m] = max(get_earlytw(N1[m],k), earliest[pre_m] + tt[pre_m,m] + N1[pre_m].μ)
            if earliest[m] > get_latetw(N1[m], k) 
                return sol,Inf 
            end
        end
        node_d.bus = k
        if sol1.routes[k].suc[d] == d error("$d suc is itself $k") end
        if node_d != sol1.N[d]
            global sol1_test = copy(sol1)
            global nd = copy(node_d)
            @error("$d is not the same as sol1.N[d]")
        end
        # bus_cost, cus_cost = insertpair!(node_p, node_d, pre[p], suc[p], i, suc_d, k, sol1)
        insertpair!(node_p, node_d, pre[p], suc[p], i, suc_d, k, sol1)
        if route1 != sol1.routes[k]
            error("routes doesnot match")
        end
        if !isfeasible(route1, sol1) continue end

        # cost = addtional_obj_cost(bus_cost, cus_cost, sol1)
        cost = objective_value(sol1)
        # cost = objective_value(sol1, k, d)
        if cost < best_cost
            best_cost = cost
            best_sol = sol1
        end
    end
    return best_sol, best_cost
end