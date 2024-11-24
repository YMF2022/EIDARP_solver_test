"""
Eigh step evaluation scheme referring to Algorithm 2 Parragh et al. 2009
"""
function isfeasible(route::Route, sol::Solution; isE::Bool = true)
    N, k = sol.N, route.ID
    if route != sol.routes[k]
        error("route does not match")
    end
    construct_route!(N, route)
    pathID = route.pathID[1:route.len]
    e₀ = get_earlytw(sol.N[pathID[1]], k)
    res = true
    D₀ = e₀                                         # Line 1
    res = update_route_info!(sol, route, D₀, pathID = pathID)        # Line 2
    if !res 
        # println("Update route info not feasible")
        return res 
    end
    route.F[1] = cal_Fi(1, route, sol)              # Line 3
    D₀ = e₀ + min(route.F[1], sum(route.W[2:end]))  # Line 4
    res = update_route_info!(sol, route, D₀)        # Line 5
    # Line 6
    res = check_ridetime(route, sol, pathID = pathID)
    if !(res && (sum(route.F_min) <= 1e-3))
        # Line 7
        res = route_time_repair!(route, sol)
        if res == false
            # println("Route time repair not feasible")
            return false
        end

        res = last_check(route, sol)
        if res == false
            # println("Last check not feasible")
            return false
        end
    end
    # Line 8: reinsert chargers if needed
    # if isE
    #     res = recharge!(k, sol)
    #     if res == false
    #         # println("recharge not feasible for bus $k, path: $pathID")
    #     end
    # end
    # @assert (res == true) || (res == false) "Recharge result has to be a boolean value"
    return res
end

function last_check(route::Route, sol::Solution)
    N, k = sol.N, route.ID
    A, B, y = route.A, route.B, route.y
    # for (i,node) in enumerate(route.path)
    for (i, node_id) in enumerate(route.pathID[1:route.len])
        node = sol.N[node_id]
        # # Check B and capacity
        # if (B[i] > get_latetw(node, k) + 1e3) | (y[i] > route.capacity)
        #     error("Bus $k: BoS time at $(node.ID) is greater than timewindow")
        #     return 1
        # end
        # # TBD: Check A: Hard consrtaints for transit stations
        # if isa(node, TransitStation)
        #     if (node.bus_servetype[k] == 2) | (node.bus_servetype[k] == 3) # dropoff customer at this station
        #         if A[i] > node.dep_time + 1e-3
        #             global sol_lc = copy(sol)
        #             error("Bus $k: Arrival time at transit station $(node.ID) is greater than departure time ")
        #         end
        #     end
        # end
        # Check ridetime
        if !check_ridetime(i, route, sol) 
            res = false
            if isa(node, Droppoff)
                # ridetime = calculate_ridetime(node, i, route, sol)
                # println("not feasible at bus$k at $(node.ID), $ridetime")
                pickup = sol.N[node.pickupID]
                if pickup.bus > 0
                    route_p = sol.routes[pickup.bus]
                    if route_p.repair == false
                        # res_repair = repair_firstmile_route(route_p, sol)
                        res_repair = route_time_repair!(route, sol)
                        if res_repair == true
                            # println("Bus$k: Last check for bus $(route.ID) at $(pickup.ID)")
                            res_rd = check_ridetime(i, route, sol)
                            if res_rd == true
                                res = true
                                # println("Check ride time succesful at node $(node.ID): $ridetime")
                            else
                                route_p.repair = false
                                return false
                            end
                        else
                            return false
                        end
                    else
                        return false
                    end
                else
                    return false
                end
            else
                return false
            end
        end
    end
    return true
end

function repair_firstmile_route(route::Route, sol::Solution)
    k = route.ID
    construct_route!(sol.N, route)
    e₀ = get_earlytw(sol.N[route.pathID[1]], k)
    route.F[1] = cal_Fi(1, route, sol)              # Line 3
    D₀ = e₀ + min(route.F[1], sum(route.W[2:end]))  # Line 4
    update_route_info!(sol, route, D₀)        # Line 5
    res_repair = route_time_repair!(route, sol)
    return res_repair
end

# Forward time slack: Equation (2) refering to Parragh et al. 2009
function cal_Fi(i_start::Int64, route::Route, sol::Solution)
    len, k = route.len, route.ID
    D = route.D
    A, W = route.A, route.W
    pathID = get_path(route)
    slack = ones(Float64, len - i_start + 1) * Inf
    for j in i_start:len
        path_tmp = pathID[i_start:len]
        # node_j = path[j]
        node_j = sol.N[pathID[j]]
        L_j = get_maxridetime(node_j)
        if isa(node_j, Droppoff) 
            ridetime = calculate_ridetime(node_j, j, route, sol)
        else
            ridetime = 0.0
        end
        term1 = sum(W[i_start+1:j]) 
        # term1 = 0.0
        if isa(node_j, TransitStation) && ((node_j.bus_servetype[k] == 2) | (node_j.bus_servetype[k] == 3))
            ear_j = get_earlytw(node_j,k)
            term2 = max(0, min(node_j.dep_time - max(A[j], ear_j), L_j - ridetime))
            # println("Term2 at node $(pathID[j]) is: $term2")
            # term2 = max(0, min(node_j.dep_time - ear_j, L_j - ridetime))
        else
            late_j = get_latetw(node_j, k)
            if (j != i_start) && isa(node_j, Droppoff) && (node_j.pickupID in path_tmp)
                term2 = max(0, late_j - D[j])
            else
                term2 = max(0, min(late_j - D[j], L_j - ridetime)) #or rt[j] equal to zero if j-n not visited before i 
            end
            # println("Term2 at node $(pathID[j]) is : $term2")
        end
        slack[j-i_start+1] = term1 + term2
    end
    Fi = minimum(slack)
    return Fi
end

function update_route_info!(sol::Solution, route::Route, D₀::Float64; pathID::Vector{Int64} = Int64[])
    tt = sol.params.tt
    if isempty(pathID)
        pathID = route.pathID[1:route.len]
    end
    k, len, pathID = route.ID, route.len, route.pathID
    A, B, D, W, y = route.A, route.B, route.D, route.W, route.y
    F_min = route.F_min
    A[1], B[1] = 0.0, D₀
    D[1] = B[1] + sol.N[pathID[1]].μ
    W[1] = B[1] - A[1]
    W[1] = 0.0
    y[1] = 0
    
    for i in 2:len
        # node = path[i]
        node = sol.N[pathID[i]]
        tw_ear = get_earlytw(node,k)
        # A[i] = D[i-1] + tt[path[i-1].ID,node.ID]
        A[i] = D[i-1] + tt[pathID[i-1], node.ID]
        B[i] = max(A[i], tw_ear)
        W[i] = max(0, B[i] - A[i])
        D[i] = B[i] + node.μ
        y[i] = y[i-1] + get_load(node, k)
        # Check A at transit station for dropping off customers
        if isa(node, TransitStation) && ((node.bus_servetype[k] == 2) | (node.bus_servetype[k] == 3))
            if A[i] > node.dep_time
                # @error("Arrival time at $(node.ID) more than trains departure")
                return false
            else
                F_min[i] = max(0.0, tw_ear - A[i])
            end
        end
        # Check B and capacity
        if (B[i] > get_latetw(node, k))
            if isa(node, Depot)
                # @error("Service time at destination $(node.ID) exceed maximum timewindow")
            else
                # @error("B $(B[i]) at destination $(node.ID) exceed maximum timewindow $(get_latetw(node,k))")
                return false
            end
        end
        if y[i] > route.capacity
            # @error("Capacity exceed at $(node.ID) at idx $i")
            return false
        end

        # Calculate ride time
        if isa(node, Droppoff)
            # rt[i] = calculate_ridetime(node, i, route, sol, pathID = pathID)
            ridetime = calculate_ridetime(node, i, route, sol, pathID = pathID)
            if ridetime < 0 
                # @error("Ride time at $(node.ID) not feasible")
                return false
            end
        end
    end
    return true
end

function calculate_ridetime(dropoff::Node, idx::Int64, route::Route, sol::Solution; pathID::Vector{Int64} = Int64[], waittime::Bool = WAITTIME)
    N = sol.N
    d, p = dropoff.ID, dropoff.pickupID
    if isempty(pathID)
        pathID = route.pathID[1:route.len]
    end
    pickup = N[p]
    bus_p = pickup.bus
    bus_d = dropoff.bus
    if waittime == false
        return customer_ridetime!(p, sol)
    else
        if bus_p == bus_d
            idx_p = route.node_pos[p]
            return route.A[idx] - route.D[idx_p]
        else
            if (bus_p == 0) && (pickup.walk_ts != 0)
                # First mile is walking
                ts = pickup.walk_ts
                dep_time = N[ts].dep_time - sol.params.A_walk[(p,ts)]
                ridetime = route.A[idx] - dep_time
                return ridetime
            elseif bus_p !=0
                # First mile by another bus
                pos = sol.routes[bus_p].node_pos[p]
                dep_time = sol.routes[bus_p].D[pos]
                # @info "Departure time of $p at bus $bus_p at position $pos is $(route.A[idx] - dep_time)"
                return route.A[idx] - dep_time
            else
                global r_fsb = copy(route)
                global s_fsb = copy(sol)
                error("Customer $p first mile is not assigned at bus $bus_d")
                return 0.0
            end
        end 
    end
end

function check_ridetime(i::Int64, route::Route, sol::Solution; pathID::Vector{Int64} = Int64[],  waittime::Bool = WAITTIME)::Bool
    N = sol.N 
    # node = route.path[i]
    pathID = route.pathID[1:route.len]
    node = N[pathID[i]]
    F_min = route.F_min
    if isempty(pathID)
        pathID = get_path(route)
    end
    if isa(node, Pickup)
        p = node.ID
        d = node.dropoffID
        walk_ts = N[d].walk_ts
        bus = N[d].bus
        if waittime == true
            if walk_ts != 0
                # Check if the last mile is walking
                arrivaltime = N[walk_ts].arr_time + sol.params.A_walk[(d,walk_ts)]
                ridetime = arrivaltime - route.D[i]
                # route.rt[i] = ridetime
                if ridetime > node.L + 1e-3
                    F_min[i] = ridetime - node.L
                    return false
                end
            elseif bus != 0
                # Check if the last mile is by bus
                route_lm = sol.routes[bus]
                idx = route_lm.node_pos[d]
                arrivaltime = sol.routes[bus].A[idx]
                ridetime = arrivaltime - route.D[i]
                # route.rt[i] = ridetime
                # println("arrivaltime $arrivaltime - departure time $(route.D[i]) = $ridetime") 
                if ridetime > node.L + 1e-3
                    F_min[i] = ridetime - node.L
                    return false 
                end
            end
        else
            # waittime is not constrainted
            if isserved(N[d])
                ridetime = customer_ridetime!(p, sol)
                if ridetime > node.L + 1e-3
                    F_min[i] = ridetime - node.L
                    return false
                end
            end
        end
    elseif isa(node, Droppoff)
        # route.rt[i] = calculate_ridetime(node, i, route, sol, pathID = pathID)
        ridetime = calculate_ridetime(node, i, route, sol, pathID = pathID)
        if ridetime > node.L + 1e-3 return false end
    end
    return true
end

function check_ridetime(route::Route, sol::Solution; pathID::Vector{Int64} = Int64[])::Bool
    N = sol.N 
    if isempty(pathID)
        pathID = get_path(route)
    end
    for (i,n) in enumerate(pathID)
        res = check_ridetime(i, route, sol, pathID = pathID)
        if !res return false end
    end
    return true
end

function route_time_repair!(route::Route, sol::Solution; isfirstmile::Bool = false)
    k = route.ID
    F, W, B, A, D = route.F, route.W, route.B, route.A, route.D
    F_min = route.F_min
    N, tt = sol.N, sol.params.tt
    # path = route.path
    pathID = route.pathID[1:route.len]
    len_r = route.len
    # for (i,node) in enumerate(path)
    for (i, node_id) in enumerate(pathID)
        node = N[node_id]
        if isa(node, Pickup) | isa(node, TransitStation)
            F[i] = cal_Fi(i, route, sol)
            if F_min[i] > F[i] + 1e-3
                return false
            end
            W[i] += F[i]
            B[i] = A[i] + W[i]
            D[i] = B[i] + node.μ
            # Check for customer last mile walking
            # if !check_ridetime(i, route, sol) return false end
            if F[i] <= 1e-3 continue end
            for j in i+1:len_r
                # node2 = path[j]
                # A[j] = D[j-1] + tt[path[j-1].ID,node2.ID]
                node2 = N[pathID[j]]
                A[j] = D[j-1] + tt[pathID[j-1],node2.ID]
                B[j] = max(A[j], get_earlytw(node2, route.ID))
                W[j] = max(0, B[j] - A[j])
                D[j] = B[j] + node2.μ
                F_min[j] = max(0, F_min[j] - F[i])
            end  
        end
    end
    route.repair = true
end