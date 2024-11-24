
function insertpair!(node1::Node, node2::Node, p1::Int64, s1::Int64, p2::Int64, s2::Int64, bus::Int64, sol::Solution)
    if s1 == node2.ID
        # add_cost, travel_cost = insertpair!(node1, node2, p1, s2, bus, sol)
        insertpair_adjacent!(node1, node2, p1, s2, bus, sol)
    else
        # addtional bus cost and additional customer cost
        # bus_cost1, cus_cost1 = insertnode!(node1, p1, s1, bus, sol)
        # bus_cost2, cus_cost2 = insertnode!(node2, p2, s2, bus, sol)
        insertnode!(node1, p1, s1, bus, sol)
        insertnode!(node2, p2, s2, bus, sol)
        # travel_cost = 0.0 # the inserted customer travel cost
        # i = node1.ID
        # route = sol.routes[bus]
        # for count in 1:route.len
        #     j = route.suc[i]
        #     travel_cost += tt[i,j]
        #     if j == node2.ID
        #         break
        #     end
        #     i = j
        #     if (count == route.len) && (node2.ID != route.depot_d)
        #         error("Cannot find node2 $(node2.ID) at bus $bus")
        #     end
        # end
        update_customer!(node1, node2, bus, sol)
        # add_cost = bus_cost1 + bus_cost2 + cus_cost1 + cus_cost2 # addtional bus + customer cost
    end
    # return add_cost, travel_cost
end

"""
Insert a node to a route 

# Arguments:
- `node`: to be inserted node
- 'pred': predecessor of the inserted position
- `suc`: successor of the inserted position 
- `k`: route to be inserted
- `sol::Solution`: current solution
"""
function insertnode!(node::Node, p::Int64, s::Int64, k::Int64, sol::Solution; update_q::Bool = false)
    N, tt = sol.N, sol.params.tt
    n  = node.ID
    route = sol.routes[k]
    earliest, latest, cap = route.earliest, route.latest, route.cap
    # update usedbus
    sol.usedbus[k] == false ? sol.usedbus[k] = true : nothing
    # update succ and pred
    if !isa(node, Charger)
        # if !isservedbus(node, k)
        if route.suc[n] == 0 # means this node is not served
            route.suc[p] = n
            route.suc[n] = s
            route.pre[s] = n 
            route.pre[n] = p 
        end
    else
        # if route.suc[n] != 0
        #     global s_test = copy(sol)
        #     error("Node $n has already been inserted in bus $k with $(route.suc[n])")
        # end
        route.suc[p] = n
        route.suc[n] = s
        route.pre[s] = n 
        route.pre[n] = p 
    end

    # update the service type and laod for node if it is a transit station
    # if isa(node, TransitStation) update_servetype!(node, k, 1, update_q = update_q) end
    # Forward loop: update earliest timewindow and capacity
    i = n
    while i != route.depot_d # while it has not readed the destination depot
        j = route.pre[i] # the predecessor of node i
        e_i, q_i = get_earlytw(N[i], k), get_load(N[i],k)
        earliest[i] = max(e_i, earliest[j] + tt[j,i] + N[j].μ) # Braecker et al. 2014 equation (17)
        cap[i] = cap[j] + q_i # Braecker et al. 2014 equation (18); q here is 1
        i = route.suc[i]
    end

    i = n
    while i != route.depot_o # while it has not reached the origin depot
        j = route.suc[i] # the successor of i
        l_i, q_i = get_latetw(N[i],k), get_load(N[i], k)
        latest[i] = min(l_i, latest[j]- tt[i,j]-N[i].μ) # Braecker et al. 2014 equation (19)
        # sol.maxcap[i] = max(0, sol.maxcap[j] + q_i) # Braecker et al. 2014 equation (20); q here is -1
        # if isa(N[i], Pickup) && (i != pickup_insert)
        #     push!(pickups, i)
        # elseif isa(N[i], Droppoff)
        #     push!(pickup_with_drop, N[i].pickupID)
        # end
        i = route.pre[i]
    end

    # num_cus = 0 # cost not used in the results
    # cost = tt[p,n] + tt[n,s] - tt[p,s]
    # return cost, cost * num_cus
end

"""
Insert a adjecent pair of nodes to a route, e.g, ...-pred-node1-node2-succ-...

# Arguments:
- `node1`: to be inserted node
- `node2`: to be inserted node
- 'p': predecessor of node1
- `s`: successor of the node2
- `k`: route to be inserted
- `sol::Solution`: current solution
"""
function insertpair_adjacent!(node1::Node, node2::Node, p::Int64, s::Int64, k::Int64, sol::Solution; update_q::Bool = false)
    N, tt = sol.N, sol.params.tt
    n1, n2 = node1.ID, node2.ID
    route = sol.routes[k]
    # update usedbus
    sol.usedbus[k] == false ? sol.usedbus[k] = true : nothing
    # update succ
    route.suc[n1] = n2
    route.pre[n2] = n1
    if isservedbus(node1, k)
        route.suc[n2] = s
        route.pre[s] = n2
    elseif isservedbus(node2, k)
        route.suc[p] = n1
        route.pre[n1] = p 
    else
        route.suc[p] = n1
        route.suc[n2] = s
        route.pre[n1] = p 
        route.pre[s] = n2
    end
    
    # update customers bus ID
    update_customer!(node1, node2, k, sol)

    # update the service type and laod for n1 n2 if they are transit station
    if isa(node1, TransitStation) update_servetype!(node1, k, 1, update_q = update_q) end
    if isa(node2, TransitStation) update_servetype!(node2, k, 2, update_q = update_q) end

    # update earliest timewindow and capacity in forward loop
    i = n1
    while i != route.depot_d # while it has not readed the destination depot
        j = route.pre[i] # the predecessor of node i
        e_i, q_i = get_earlytw(N[i], k), get_load(N[i],k)
        # @show i,j
        route.earliest[i] = max(e_i, route.earliest[j] + tt[j,i] + N[j].μ) # Braecker et al. 2014 equation (17)
        route.cap[i] = route.cap[j] + q_i # Braecker et al. 2014 equation (18); q here is 1
        i = route.suc[i]
    end
    # update latest timewindow and max capacity in backward loop
    i = n2
    while i != route.depot_o # while it has not reached the origin depot
        j = route.suc[i] # the successor of i
        l_i = get_latetw(N[i],k)
        route.latest[i] = min(l_i, route.latest[j]- tt[i,j]-sol.N[i].μ) # Braecker et al. 2014 equation (19)
        # sol.maxcap[i] = max(0, sol.maxcap[j] + q_i) # Braecker et al. 2014 equation (20); q here is -1
        # if isa(N[i], Pickup) && (i != n1)
        #     push!(pickups, i)
        # elseif isa(N[i], Droppoff)
        #     push!(pickup_with_drop, N[i].pickupID)
        # end
        i = route.pre[i]
    end
    # num_cus = 0 # cost not used in the end
    # cost = tt[p,n1] + tt[n1,n2] + tt[n2,s] - tt[p,s]
    # return cost+cost*num_cus, tt[n1,n2]
end

function update_customer!(n1::Node, n2::Node, k::Int64, s::Solution)
    # update customer and s.unserved customer
    if isa(n1, Pickup) && isa(n2, Droppoff)
        n1.bus = k; n2.bus = k
        n1.walk_ts = 0; n2.walk_ts = 0
        n1.bus_ts = 0; n2.bus_ts = 0
        if isserved(n1) s.servedcus[n1.ID] = true end
        if isserved(s.N[n2.pickupID]) s.servedcus[n2.pickupID] = true end
    elseif isa(n1, Pickup) && isa(n2, TransitStation)
        n1.bus = k
        n1.walk_ts = 0
        n1.bus_ts = n2.ID
        # n2.q[k] -= 1
        n2.cus_board[k,n1.ID] = true
        if isserved(s.N[n1.dropoffID]) s.servedcus[n1.ID] = true end
        # if isserved(s.N[n1.dropoffID]) push!(s.servedcus_set, n1.ID) end
    else isa(n1, TransitStation) && isa(n2, Droppoff)
        # n1.q[k] += 1
        n1.cus_alight[k,n2.pickupID] = true
        n2.bus = k
        n2.walk_ts = 0
        n2.bus_ts = n1.ID
        if isserved(s.N[n2.pickupID]) s.servedcus[n2.pickupID] = true end
        # if isserved(s.N[n2.pickupID]) push!(s.servedcus_set, n2.pickupID) end
    end
end