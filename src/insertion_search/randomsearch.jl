"""
Greedy search function including the following cases:
    1. Repaire the first mile, and customer walk at last-mile.
    2. Repaire from customers' origin to destination
"""

function randomsearch_firstmile(sol::Solution, node1::Pickup, node2::Union{Droppoff, TransitStation}; isE::Bool = true)
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
    bus_sf = shuffle(buses)
    for k in bus_sf
        # @info "Bus $k"
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
                feasibility = isfeasible(route2, sol2)
                # println("Check feasibility at $i-$suc_p and $j-$suc_d: $feasibility")
                # if (i==20) && (j==4) && (k==5)
                #     global s1_test = copy(sol2)
                # end
                if !feasibility continue end
                if route2 != sol2.routes[k]
                    error("routes doesnot match")
                end
                # Step 4.1: calculate customer travel cost  
                # We do not need to calcualte customer travel cost as it is part of the bus insertion cost
                len_sol += 1
                # cost = addtional_obj_cost(bus_cost, cus_cost, sol2)
                # cost = bus_cost
                cost = objective_value(sol2)
                # cost = objective_value(sol2, k, p, d)
                solutions[len_sol] = sol2
                costs[len_sol] = cost
                if cost < best_cost
                    best_cost = cost
                    best_sol = sol2
                    break
                end
                if len_sol >= DEGREE_REGRET break end
            end
        end
    end
    sol_addbus, cost_addbus = addbus(sol, node1, node2)
    # println("P: $p, transit: $d, bus $(sol_addbus.N[p].bus), cost: $cost_addbus ")
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
        return solutions, costs, len_sol
    else
        # No feasible solution has been found
        return [sol], [Inf], 0
    end
end

"""
Greedy search function including the following cases:
    1. Repaire the last mile, and customer first mile is walking.
In this function, return three elements: route, node1, and node2
"""
function randomsearch_lastmile(sol::Solution, node1::TransitStation, node2::Droppoff; isE::Bool=true)
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
    
    bus_sf = shuffle(buses)
    for k in bus_sf
        # @info "Bus $k"
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
                    break
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
                feasibility = isfeasible(route2, sol2)
                # println("Check feasibility at $i-$suc_p and $j-$suc_d: $feasibility")
                if !feasibility
                    continue
                end
                # Step 4.1: calculate customer travel cost  
                # We do not need to calcualte customer travel cost as it is part of the bus insertion cost
                len_sol += 1
                # cost = addtional_obj_cost(bus_cost, cus_cost, sol2)
                cost = objective_value(sol2)
                # cost = objective_value(sol2, k, p, d)
                solutions[len_sol] = sol2
                costs[len_sol] = cost
                if cost < best_cost
                    best_cost = cost
                    best_sol = sol2
                    break
                end
                if len_sol >= DEGREE_REGRET break end
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
        return solutions, costs, len_sol
    else
        # No feasible solution has been found
        return [sol], [Inf], 0
    end
end

# Repaire a customer with transit arc integration, where k is the k-nearest transit pair
function randomsearch_bothmile(sol::Solution, cus_o::Pickup, ts_pair::Vector; isE::Bool=true)  
    # ts_pair = cus_o.ts_pair[k]
    if isempty(ts_pair) return [sol], [Inf], 0 end
    max_len_sol = 1000
    solutions = Vector{Solution}(undef, max_len_sol)
    costs = Vector{Float64}(undef, max_len_sol)
    len_sol = 0
    sol_best, cost_best = nothing, Inf
    p = cus_o.ID
    d = cus_o.dropoffID
    for pair in ts_pair
        ts_o, ts_d = pair[1], pair[2]
        # s_cur = copy(sol)
        # s_cur = sol
        s_cur_list, cost_cur1_list, _ = randomsearch_firstmile(sol, sol.N[p], sol.N[ts_o])
        s_cur, cost_cur1 = s_cur_list[1], cost_cur1_list[1]
        if s_cur.N[p].bus_ts != ts_o continue end
        s_cur_list, cost_cur2_list, _ = randomsearch_lastmile(s_cur, s_cur.N[ts_d], s_cur.N[d]) 
        s_cur, cost_cur2 = s_cur_list[1], cost_cur2_list[1]
        # cost_fm = firstmile_cost(s_cur.N[p], s_cur)[3]
        if (cost_cur1 != Inf) && (cost_cur2 != Inf)
            # only include feasible solution
            for (i,c2) in enumerate(cost_cur2_list)
                len_sol += 1
                # cost_cur = cost_cur1 + c2 - cost_fm
                cost_cur = c2
                costs[len_sol] = cost_cur
                solutions[len_sol] = s_cur_list[i]
                if i == 1
                    if cost_cur < cost_best
                        sol_best = s_cur_list[i]
                        cost_best = cost_cur
                    end
                end
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
        return solutions, costs, len_sol
    else
        return [sol], [Inf], 0
    end
end