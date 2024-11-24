function remove_customer!(sol::Solution, cus::Int64; remove_walk::Bool = true)
    node_p = sol.N[cus]
    node_d = sol.N[node_p.dropoffID]
    if (node_p.bus_ts > 0) && (node_d.bus_ts > 0)
        remove_customer_bothmiles!(sol, cus)
    elseif (node_p.bus_ts > 0) && (node_d.walk_ts >= 0)
        if remove_walk node_d.walk_ts = 0 end
        remove_customer_firstmile!(sol, cus)
    elseif (node_p.walk_ts > 0) && (node_d.bus_ts > 0)
        if remove_walk node_p.walk_ts = 0 end
        remove_customer_lastmile!(sol, cus)
    elseif (node_p.walk_ts > 0) && (node_d.walk_ts >= 0)
        if remove_walk
            node_p.walk_ts = 0; node_d.walk_ts = 0
            sol.servedcus[cus] = false
        end
    else
        remove_customer_OD!(sol, cus)
    end
end

function remove_customer_OD!(sol::Solution, cus::Int64)
    N = sol.N 
    node_p = N[cus]
    d = node_p.dropoffID
    node_d = N[d]
    if node_p.bus != node_d.bus
        error("Remove customer $cus is wrong: different bus at origin and destination")
    end
    remove_node!(node_p, node_d, sol)
end

function remove_customer_firstmile!(sol::Solution, cus::Int64)
    N = sol.N 
    node_p = N[cus]
    node_ts = N[N[cus].bus_ts]
    remove_node!(node_p, node_ts, sol)
    return cus, node_ts.ID
end

function remove_customer_lastmile!(sol::Solution, cus::Int64)
    N = sol.N 
    node_d = N[N[cus].dropoffID]
    node_ts = N[node_d.bus_ts]
    remove_node!(node_ts, node_d, sol)
    return node_ts.ID, node_d.ID
end

function remove_customer_bothmiles!(sol::Solution, cus::Int64)
    N = sol.N 
    node_p = N[cus]
    node_d = N[N[cus].dropoffID]
    node_ts1 = N[node_p.bus_ts]
    node_ts2 = N[node_d.bus_ts]
    remove_node!(node_p, node_ts1, sol)
    remove_node!(node_ts2, node_d, sol)
end

# remove node on a specific route
function remove_node_route!(n::Int64, route::Route, sol::Solution)
    suc, pre = route.suc, route.pre
    pre_n = pre[n]
    suc_n = suc[n]
    suc[pre_n] = suc_n
    pre[suc_n] = pre_n
    suc[n] = 0
    pre[n] = 0
    idx = route.node_pos[n]
    route.len -= 1

    # remove all the charger event
    # pathID = get_path(route)
    # pathID = setdiff(pathID, n)   
    # if !route.removeE
    #     for n_cgr in pathID[2:route.len-1]
    #         if isa(sol.N[n_cgr], Charger)
    #             pre_nc = pre[n_cgr]
    #             suc_nc = suc[n_cgr]
    #             suc[pre_nc] = suc_nc
    #             pre[suc_nc] = pre_nc
    #             suc[n_cgr] = 0
    #             pre[n_cgr] = 0
    #             filter!(((key,_),) -> key[1] != route.ID, sol.N[n_cgr].cgr_evt)
    #             # pop!(sol.N[n_cgr].cgr_evt, (route.ID, pre_nc))
    #         end
    #     end
    #     route.removeE = true
    # end

    # Init the route if the route does not serve any customer
    if route.suc[route.depot_o] == route.depot_d 
        init_route!(route.ID, sol) 
        return
    end

    # move after values one step forward
    # route.pathID[idx:end-1] = route.pathID[idx+1:end]
    # route.A[idx:end-1] = route.A[idx+1:end]
    # route.B[idx:end-1] = route.B[idx+1:end]
    # route.W[idx:end-1] = route.W[idx+1:end]
    # route.D[idx:end-1] = route.D[idx+1:end]
    # route.y[idx:end-1] = route.y[idx+1:end]
    # route.rt[idx:end-1] = route.rt[idx+1:end]
    # route.F[idx:end-1] = route.F_min[idx+1:end]
    # route.F_min[idx:end-1] = route.F_min[idx+1:end]
    # for i in idx:route.len
    #     n = route.pathID[i]
    #     route.node_pos[n] = i
    # end
end

function remove_node!(node1::Node, node2::Node, sol::Solution)
    if isa(node1, Pickup) && isa(node2, TransitStation)
        k = node1.bus
        route = sol.routes[k]
        # update node
        node1.bus = 0
        node1.bus_ts = 0
        remove_node_route!(node1.ID, route, sol)
        node2.cus_board[k,node1.ID] = 0
        update_servetype!(node2, k)
        node2.q[k] += 1
        # update route
        if node2.bus_servetype[k] == 0
            remove_node_route!(node2.ID, route, sol)
        end
        update_earliest_latest!(sol, k)
        # update solution
        sol.servedcus[node1.ID] = false
    elseif isa(node1, TransitStation) && isa(node2, Droppoff)
        k = node2.bus
        route = sol.routes[k]
        # update node
        node1.cus_alight[k,node2.pickupID] = 0
        node1.q[k] -= 1
        update_servetype!(node1, k)
        node2.bus = 0
        node2.bus_ts = 0
        # update route
        if node1.bus_servetype[k] == 0
            remove_node_route!(node1.ID, route, sol)
        end
        remove_node_route!(node2.ID, route, sol)
        # init_earliest_latest!(sol, k)
        update_earliest_latest!(sol, k)
        # update solution
        sol.servedcus[node2.pickupID] = false
    elseif isa(node1, Pickup) && isa(node2, Droppoff)
        k = node1.bus
        route = sol.routes[k]
        # update node
        node1.bus = 0
        node1.bus_ts = 0
        node2.bus = 0
        node2.bus_ts = 0
        # update route
        remove_node_route!(node1.ID, route, sol)
        remove_node_route!(node2.ID, route, sol)
        # init_earliest_latest!(sol, k)
        update_earliest_latest!(sol, k)
        # update solution
        # sol.customers[node1.ID] = []
        sol.servedcus[node1.ID] = false
        # delete!(sol.servedcus_set, node1.ID)
    else
        @warn "Remove node1-$(node1.ID): $(typeof(node1)), node2-$(node2.ID): $(typeof(node2)) Charger node or depot node is not defined"
    end
end

# Calculate the saved bus cost (travel time) of removing a node
function calculate_remove_cost(sol::Solution, node::Int64, k::Int64)::Float64
    tt = sol.params.tt
    route = sol.routes[k]
    suc_n, pre_n = route.suc[node], route.pre[node]
    λ1 = sol.params.λ[1] # the weight for bus cost
    return (tt[pre_n, node] + tt[node, suc_n] - tt[pre_n, suc_n])*λ1
end

# Calculate the saved bus cost (travel time) of removing a customer
function calculate_cus_remove_cost(sol::Solution, cus::Int64)
    node_p = sol.N[cus]
    d = node_p.dropoffID
    node_d = sol.N[d]
    if (node_p.bus_ts > 0) && (node_d.bus_ts > 0)
        k1, k2 = node_p.bus, node_d.bus
        cost1 = calculate_remove_cost(sol, cus, k1)
        cost2 = calculate_remove_cost(sol, d, k2)
        return cost1 + cost2
    elseif (node_p.bus_ts > 0) && (node_d.walk_ts >= 0)
        return calculate_remove_cost(sol, cus, node_p.bus)
    elseif (node_p.walk_ts > 0) && (node_d.bus_ts > 0)
        return calculate_remove_cost(sol, d, node_d.bus)
    elseif (node_p.walk_ts > 0) &&  (node_d.walk_ts >= 0)
        return 0.0
    else
        k1, k2 = node_p.bus, node_d.bus
        @assert k1 == k2 "Only one bus serves customer $cus"
        tt = sol.params.tt
        if sol.routes[k1].suc[cus] != d
            return calculate_remove_cost(sol, cus, k1) + calculate_remove_cost(sol, d, k2)
        else
            suc, pre = sol.routes[k1].suc, sol.routes[k1].pre
            remove_cost = (tt[pre[cus], cus] + tt[cus,d] + tt[d, suc[d]] - tt[pre[cus],suc[d]])*sol.params.λ[1]
            return remove_cost
        end
        return 
    end
end
