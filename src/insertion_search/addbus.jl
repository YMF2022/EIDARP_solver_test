
function addbus(sol::Solution, node1::Node, node2::Node)
    n1 = node1.ID; n2 = node2.ID
    tt = sol.params.tt
    N = sol.N
    routes = sol.routes
    unused_veh = findall(!, sol.usedbus)
    
    if isempty(unused_veh) return sol, Inf end
    # Find the bus's depot with minimum distance
    len_unused_veh = sum(.!(sol.usedbus))
    bus_dist = Vector{NamedTuple{(:bus_id, :dist_insert, :battery), Tuple{Int, Float64, Float64}}}(undef, len_unused_veh)
    # bus_id = 0
    for (i, k) in enumerate(unused_veh)
        route = routes[k]
        d_o, d_d = route.depot_o, route.depot_d
        dist_insert = tt[d_o,n1] + tt[n1, n2] + tt[n2,d_d]
        bus_dist[i] = (bus_id = k, 
                       dist_insert = dist_insert,
                       battery = route.battery)
    end
    # bus_dist = sort(bus_dist, by = x -> (x.dist_insert, -x.battery))
    bus_dist = sort(bus_dist, by = x -> x.dist_insert)
    # update the solution
    for b_dist in bus_dist
        bus_id = b_dist.bus_id
        d_o = routes[bus_id].depot_o
        d_d = routes[bus_id].depot_d
        sol_temp = copy(sol, N[n1], N[n2])
        insertpair_adjacent!(sol_temp.N[n1], sol_temp.N[n2], d_o, d_d, bus_id, sol_temp, update_q = true)
        # cost = addtional_obj_cost(bus_cost, cus_cost, sol_temp)
        if isfeasible(sol_temp.routes[bus_id], sol_temp) 
            # println("Add bus $bus_id for $n1 $n2 successful")
            # cost = objective_value(sol_temp, bus_id, n1, n2, isaddbus = true)
            cost = objective_value(sol_temp)
            return sol_temp, cost
        else
            # @error("Add bus infeasible for $n1, $n2, bus $bus_id")
            # global sol_test = sol_temp
            continue
        end
    end
    return sol, Inf
end