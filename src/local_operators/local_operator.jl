function replace_walk(sol::Solution, obj_val::Float64)
    A_walk = sol.params.A_walk
    tt = sol.params.tt
    λ = sol.params.λ
    cus = findall(sol.servedcus)
    # cus = sol.servedcus_set
    for p in cus
        pickup = sol.N[p]
        walk_ts = pickup.walk_ts
        if walk_ts > 0
            # sol_temp = copy(sol, sol.N[p], sol.N[walk_ts])
            sol_temp = copy(sol)
            sol_temp_list, bus_cost_list, _ = randomsearch_firstmile(sol_temp, sol_temp.N[p], sol_temp.N[walk_ts])
            sol_temp, bus_cost_exc = sol_temp_list[1], bus_cost_list[1]
            obj_cur = objective_value(sol_temp)
            if obj_cur < obj_val
                sol = sol_temp
                obj_val = obj_cur
            end
        end
        d = pickup.dropoffID
        dropoff = sol.N[d]
        walk_ts = dropoff.walk_ts
        if walk_ts > 0
            # sol_temp = copy(sol, sol.N[walk_ts], sol.N[d])
            sol_temp = copy(sol)
            sol_temp_list, bus_cost_list, _ = randomsearch_lastmile(sol_temp, sol_temp.N[walk_ts], sol_temp.N[d])
            sol_temp, bus_cost_exc = sol_temp_list[1], bus_cost_list[1]
            # cus_cost_exc = sol.obj.cus_cost[p,4]
            # cus_cost_exc = tt[walk_ts,d] - A_walk[(d,walk_ts)]
            obj_cur = objective_value(sol_temp)
            if obj_cur < obj_val
            # if bus_cost_exc < cus_cost_exc
                sol = sol_temp
                obj_val = obj_cur
            end
        end
    end
    # obj_val = objective_value(sol)
    return sol, obj_val
end

# Replace first/last mile and see if it is possible to find a better solution
function replace_mile_cus(sol::Solution, obj_val::Float64, p::Int64)
    d = sol.N[p].dropoffID
    # if !isserved(sol.N[p]) && !isserved(sol.N[d])
    #     return sol, obj_val
    # end
    s_tmp = copy(sol)
    N = s_tmp.N; node_p = N[p]; node_d = N[d]
    if (node_p.bus_ts > 0) && (node_d.bus_ts > 0)
        # case 1: customer take bus to/from transit station
        ts1 = node_p.bus_ts 
        ts2 = node_d.bus_ts
        ts_pair = node_p.ts_pair
        ts1_other = find_different_ts(ts1, ts2, ts_pair, :first)
        ts2_other = find_different_ts(ts1, ts2, ts_pair, :last)
        if isempty(ts1_other) && isempty(ts2_other)
            return sol, obj_val
        elseif isempty(ts2_other)
            # Remove and search the first mile
            remove_customer_firstmile!(s_tmp, p)
            best_sol, best_cost = best_firstmile(s_tmp, p, ts1_other)
            if best_cost < obj_val
                return best_sol, best_cost
            else
                return sol, obj_val
            end
        elseif isempty(ts1_other)
            # Remove and search the last mile
            remove_customer_lastmile!(s_tmp, p)
            best_sol, best_cost = best_lastmile(s_tmp, p, ts2_other)
            if best_cost < obj_val
                return best_sol, best_cost
            else
                return sol, obj_val
            end
        else
            # Remove and search the first mile
            remove_customer_firstmile!(s_tmp, p)
            sol_first, cost_first = best_firstmile(s_tmp, p, ts1_other)
            # Remove and search the last mile
            s_tmp = copy(sol)
            remove_customer_lastmile!(s_tmp, p)
            sol_last, cost_last = best_lastmile(s_tmp, p, ts2_other)
            if cost_first < obj_val
                return sol_first, cost_first
            elseif cost_last < obj_val
                return sol_last, cost_last
            else
                return sol, obj_val
            end
        end
    elseif node_p.bus_ts > 0
        # customer walk last mile
        ts1 = node_p.bus_ts 
        ts2 = node_d.walk_ts
        ts_pair = node_p.ts_pair
        ts1_other = find_different_ts(ts1, ts2, ts_pair, :first)
        if isempty(ts1_other) return sol, obj_val end
        # Remove and search the first mile
        remove_customer_firstmile!(s_tmp, p)
        best_sol, best_cost = best_firstmile(s_tmp, p, ts1_other)
        if best_cost < obj_val
            return best_sol, best_cost
        else
            return sol, obj_val
        end
    elseif node_d.bus_ts > 0
        # Customer walk first mile
        ts1 = node_p.walk_ts
        ts2 = node_d.bus_ts 
        ts_pair = node_p.ts_pair
        ts2_other = find_different_ts(ts1, ts2, ts_pair, :last)
        if isempty(ts2_other) return sol, obj_val end
        # Remove and search the last mile
        remove_customer_lastmile!(s_tmp, p)
        best_sol, best_cost = best_lastmile(s_tmp, p, ts2_other)
        if best_cost < obj_val
            return best_sol, best_cost
        else
            return sol, obj_val
        end
    else
        # customer walk both miles
        return sol, obj_val
    end
end

# Remove the first or last mile TODO: make the code easier
function replace_mile(sol::Solution, obj_val::Float64)
    customers = findall(sol.servedcus)
    customers = [c for c in customers if isserved(sol.N[c])]
    # customers = rand(customers, Int(ceil(0.1*sum(sol.servedcus))))
    # customers = sol.servedcus_set
    customers = rand(customers, Int64(ceil(0.5*length(customers))))
    for p in customers
        # if isservedts(sol.N[p])
            sol, obj_val = replace_mile_cus(sol, obj_val, p)
        # end
    end
    return sol, obj_val
end

# Reinsert the customers at the same transit stations; similar to destroy and search procedure
function remove_insert_mile(sol::Solution, obj_val::Float64)
    customers = findall(sol.servedcus)
    # customers = sol.servedcus_set
    for p in customers
        s_tmp = copy(sol)
        d = sol.N[p].dropoffID
        if (s_tmp.N[p].bus_ts > 0) && (s_tmp.N[d].bus_ts > 0)
            method = rand([remove_customer_firstmile!, remove_customer_lastmile!])
            n1, n2 = method(s_tmp, p)
            s_tmp_list, _ = search(s_tmp, s_tmp.N[n1], s_tmp.N[n2])
            s_tmp = s_tmp_list[1]
        elseif (s_tmp.N[p].bus_ts > 0) && (s_tmp.N[d].bus_ts == 0)
            n1, n2 = remove_customer_firstmile!(s_tmp, p)
            s_tmp_list, _ = randomsearch_firstmile(s_tmp, s_tmp.N[n1], s_tmp.N[n2])
            s_tmp = s_tmp_list[1]
        elseif (s_tmp.N[p].bus_ts == 0) && (s_tmp.N[d].bus_ts > 0)
            n1, n2 = remove_customer_lastmile!(s_tmp, p)
            s_tmp_list, _ = randomsearch_lastmile(s_tmp, s_tmp.N[n1], s_tmp.N[n2])
            s_tmp = s_tmp_list[1]
        end
        cur_obj_val = objective_value(s_tmp)
        if cur_obj_val < obj_val
            sol = s_tmp
            obj_val = cur_obj_val
        end
    end
    return sol, obj_val
end

# Remove customer ts part and serve customer from its origin to destination
function remove_ts_line(sol::Solution, obj_val::Float64)
    customers = findall(sol.servedcus)
    # customers = sol.servedcus_set
    for p in customers
        s_tmp = copy(sol)
        d = sol.N[p].dropoffID
        if (s_tmp.N[p].bus_ts > 0) && (s_tmp.N[d].bus_ts > 0)
            remove_customer!(s_tmp, p)
            s_tmp_list,_ = randomsearch_firstmile(s_tmp, s_tmp.N[p], s_tmp.N[d], isE = true)
            s_tmp = s_tmp_list[1]
            if s_tmp.N[d].bus_ts > 0
                @error "The $d still served by bus_ts $(s_tmp.N[d].bus_ts)"
            end
            if s_tmp.N[p].bus_ts > 0
                @error "Node $p still served by bus_ts $(s_tmp.N[p].bus_ts)"
            end
        end
        cur_obj_val = objective_value(s_tmp)
        if cur_obj_val < obj_val
            sol = s_tmp
            obj_val = cur_obj_val
        end
    end
    return sol, obj_val
end

# Try to replace ts arcs with others even the cost might increase
function replace_ts_arc(sol::Solution)

end

#=
Below are some operators from bus perspective 
=#
function change_depot(sol::Solution, obj_val::Float64)
    tt = sol.params.tt
    used_bus = findall(sol.usedbus)
    # unused_bus = findall(x->x==0, sol.usedbus)
    for r in used_bus
        route = sol.routes[r]
        pathID = get_path(route)
        next_node = pathID[2]
        before_last_node = pathID[end-1]
        cost_cur = tt[route.depot_o, next_node] + tt[before_last_node,route.depot_d]
        unused_bus = findall(!, sol.usedbus)
        # unused_bus = sort(unused_bus, by = bus -> sol.routes[bus].battery)
        for bus in unused_bus
            new_depot_o = sol.routes[bus].depot_o
            new_depot_d = sol.routes[bus].depot_d
            if new_depot_o == route.depot_o
                continue
            else
                cost_new = tt[new_depot_o, next_node] + tt[before_last_node, new_depot_d]
                if cost_new < cost_cur
                    s_tmp = copy(sol)
                    s_tmp.routes[bus].suc = copy(route.suc)
                    s_tmp.routes[bus].suc[new_depot_o] = pathID[2]
                    s_tmp.routes[bus].suc[pathID[end-1]] = new_depot_d
                    s_tmp.routes[bus].pre = copy(route.pre)
                    s_tmp.routes[bus].pre[pathID[2]] = new_depot_o
                    s_tmp.routes[bus].pre[new_depot_d] = pathID[end-1]
                    order_route!(s_tmp.routes[bus], s_tmp.N)
                    update_route_of_node!(bus, r, s_tmp) # change node's bus to bus
                    res = isfeasible(s_tmp.routes[bus], s_tmp) # Only for update A, B, D, etc. No need for feasiblity check here
                    if res
                        init_route!(r, s_tmp) # init the old route
                        s_tmp.usedbus[bus] = true
                        sol = s_tmp
                        # obj_val = obj_val + (cost_new - cost_cur) * sol.params.λ[1] # only bus cost changed
                        obj_val = objective_value(sol)
                    else
                    end
                    break
                end
            end
        end
    end
    return sol, obj_val
end

function swap_routes(sol::Solution, obj_val::Float64)
    tt = sol.params.tt
    used_bus = findall(sol.usedbus)
    len_bus = sum(sol.usedbus)
    for i in 1:len_bus
        k1 = used_bus[i]
        route1 = sol.routes[k1]
        pathID_1 = get_path(route1)
        depot_o_1 = route1.depot_o
        depot_d_1 = route1.depot_d
        cost_cur_1 = tt[depot_o_1, pathID_1[2]] + tt[pathID_1[end-1], depot_d_1]    
        for j in i+1:len_bus
            k2 = used_bus[j]
            @info k1, k2
            route2 = sol.routes[k2]
            depot_o_2 = route2.depot_o
            depot_d_2 = route2.depot_d
            if depot_o_2 == route1.depot_o
                continue
            else
                pathID_2 = get_path(route2)
                cost_cur_2 = tt[depot_o_2, pathID_2[2]] + tt[pathID_2[end-1], depot_d_2]
                println("bus $k1 cost $cost_cur_1")
                println("bus $k2 cost $cost_cur_2")
                cost_new_1 = tt[depot_o_2, pathID_1[2]] + tt[pathID_1[end-1],depot_d_2]
                cost_new_2 = tt[depot_o_1, pathID_2[2]] + tt[pathID_2[end-1],depot_d_1]
                println("New bus $k1 cost $cost_new_1")
                println("New bus $k2 cost $cost_new_2")
                if (cost_new_1 + cost_new_2) < (cost_cur_1 + cost_cur_2)
                    @info "Exchange bus$k1 and $k2"
                    # s_tmp = copy(sol)
                    # s_tmp.routes[bus].suc = copy(route1.suc)
                    # s_tmp.routes[bus].suc[depot_o_2] = pathID[2]
                    # s_tmp.routes[bus].suc[pathID[end-1]] = depot_d_2
                    # s_tmp.routes[bus].pre = copy(route1.pre)
                    # s_tmp.routes[bus].pre[pathID[2]] = depot_o_2
                    # s_tmp.routes[bus].pre[depot_d_2] = pathID[end-1]
                    # order_route!(s_tmp.routes[bus], s_tmp.N)
                    # update_route_of_node!(bus, r, s_tmp) # change node's bus to bus
                    # res = isfeasible(s_tmp.routes[bus], s_tmp) # Only for update A, B, D, etc. No need for feasiblity check here
                    # if res
                    #     init_route!(route1.ID, s_tmp)
                    #     s_tmp.usedbus[bus] = true
                    #     sol = s_tmp
                    #     obj_val = obj_val + (cost_new - cost_cur) * sol.params.λ[1] # only bus cost changed
                    # end
                    # break
                end
            end
        end
    end
    # return sol, obj_val
end