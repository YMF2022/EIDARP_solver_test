function objective_value(sol::Solution; output::Bool = false)
    tt, A_ts, A_walk = sol.params.tt, sol.params.A_ts, sol.params.A_walk
    N = sol.N
    customers = [Int64[] for _ in 1:sol.params.num_cus]
    cus_cost = zeros(Float64, sol.params.num_cus, 4) # walk, bus, train, total
    # cus_cost = sol.obj.cus_cost # walk, bus, train, total
    cus_cost .= 0.0 
    bus_cost = zeros(Float64, sol.params.num_bus)
    paths = [Int64[] for _ in 1:sol.params.num_bus]
    # bus cost
    usedbus = findall(sol.usedbus)
    for k in usedbus
        route = sol.routes[k]
        order_route!(route, sol.N)
        pathID = get_path(route)
        for i in 1:route.len - 1
            bus_cost[k] += tt[pathID[i], pathID[i+1]]
        end
        paths[route.ID] = pathID
    end
    # customer cost
    for (p, isserved) in enumerate(sol.servedcus)
        if isserved == false continue end
        pickup = N[p]
        d = N[p].dropoffID
        dropoff = N[d]
        if isservedts(pickup)
            # First mile
            if pickup.walk_ts > 0
                ts1 = pickup.walk_ts
                firstmile = [p,ts1]; cus_cost[p,1] += A_walk[(p, ts1)]
            else
                ts1 = pickup.bus_ts
                head = sol.routes[pickup.bus].node_pos[p]
                tail = sol.routes[pickup.bus].node_pos[ts1]
                firstmile = paths[pickup.bus][head:tail]
                len_fm = length(firstmile)
                cus_cost[p,2] += sum(tt[firstmile[i],firstmile[i+1]] for i in 1:len_fm-1)
            end
            # ts mile + Last mile
            if dropoff.walk_ts > 0
                ts2 = dropoff.walk_ts
                cus_cost[p,3] += A_ts[(ts1,ts2)]
                lastmile = [ts2,d]; cus_cost[p,1] += A_walk[(d, ts2)]
            else
                ts2 = dropoff.bus_ts
                cus_cost[p,3] += A_ts[(ts1,ts2)] # ts mile cost
                head = sol.routes[dropoff.bus].node_pos[ts2]
                tail = sol.routes[dropoff.bus].node_pos[d]
                if isnothing(head) | isnothing(tail)
                    @error "Customer's last mile $ts2->$d not served by bus$(dropoff.bus): head = $head, tail = $tail"
                end
                lastmile = paths[dropoff.bus][head:tail]
                len_lm = length(lastmile)
                cus_cost[p,2] += sum(tt[lastmile[i],lastmile[i+1]] for i in 1:len_lm-1)
            end
            customers[p] = [firstmile; lastmile]
        else
            if pickup.bus != dropoff.bus
                error("Bus of customer $p pickup and dropoff is not the same $((pickup.bus, dropoff.bus))")
            end
            head = sol.routes[pickup.bus].node_pos[p]
            tail = sol.routes[pickup.bus].node_pos[d]
            customers[p] = paths[pickup.bus][head:tail]
            cus_cost[p,2] += sum(tt[customers[p][i], customers[p][i+1]] for i in 1:length(customers[p])-1)
        end
        cus_cost[p,4] = sum(cus_cost[p,:])
    end

    # Penalty cost
    penalty_cost = sum(.!sol.servedcus) * sol.params.ω
    # sol.total
    sol.objective_cost = objective_func(sol.params.λ, sum(bus_cost), sum(cus_cost[:,4]), penalty_cost)
    if output == true
        return bus_cost, cus_cost, penalty_cost, customers
    end
    return sol.objective_cost
end

function objective_value(sol::Nothing)
    return Inf
end

# Caculate the addtional objective value when add node1, node2 to a bus
# if n2 is a dropp off, get the full ride time (without wait)
function objective_value(sol::Solution, bus::Int64, n1::Int64, n2::Int64; isaddbus = false)
    tt, A_ts, A_walk = sol.params.tt, sol.params.A_ts, sol.params.A_walk
    N = sol.N
    route = sol.routes[bus]
    pathID = get_path(route)
    node_pos = route.node_pos
    inserted_cus_cost = 0.0 # timetime n1 -> n2
    insert_cost1 = tt[route.pre[n1], n1] + tt[n1, route.suc[n1]] - tt[route.pre[n1], route.suc[n1]]
    insert_cost2 = tt[route.pre[n2], n2] + tt[n2, route.suc[n2]] - tt[route.pre[n2], route.suc[n2]]
    if route.suc[n1] == n2
        insert_cost = tt[route.pre[n1], n1] + tt[n1, n2] + tt[n2, route.suc[n2]] - tt[route.pre[n1], route.suc[n2]]
    else
        insert_cost = insert_cost1 + insert_cost2 # is also additional bus cost
    end
    # @info insert_cost1, insert_cost2
    if isaddbus
        if isa(N[n1], Pickup) && isa(N[n2], TransitStation)
            inserted_cus_cost = firstmile_cost(N[n1], sol)[3]
            d = N[n1].dropoffID      
            if N[d].walk_ts > 0
                inserted_cus_cost += sol.params.A_ts[(n2, N[d].walk_ts)]
                inserted_cus_cost += sol.params.A_walk[(d, N[d].walk_ts)]
            end
        elseif isa(N[n2], Droppoff)
            p = N[n2].pickupID
            inserted_cus_cost = customer_ridetime!(p, sol)
        end
        # global s_test = copy(sol)
        # @info bus, n1, n2, insert_cost, inserted_cus_cost
        return sum([insert_cost, inserted_cus_cost] .* sol.params.λ[1:2])
    end
    # cus_indicator = zeros(Bool, sol.params.num_cus)
    cus_cost_affected = 0.0 # customers affected by inserting these two nodes
    for i in 1:route.len - 1
        n = pathID[i]
        pos1 = node_pos[n1]
        pos2 = node_pos[n2]
        # bus_cost[bus] += tt[n, pathID[i+1]] # bus cost
        if i < pos1
            if isa(N[n], Pickup)
                d = N[n].dropoffID
                if N[d].bus == bus
                    if node_pos[d] > pos2
                        cus_cost_affected += insert_cost
                        # println("insert cost: $cus_cost_affected for $n")
                        # cus_indicator[n] = 1 # forbid repetitive check later for dropoff
                    elseif node_pos[d] > pos1
                        cus_cost_affected += insert_cost1
                        # println("insert cost1: $cus_cost_affected for $n")
                        # cus_indicator[n] = 1
                    end
                elseif (N[n].bus_ts > 0) && (N[n].bus_ts in pathID)
                    bus_ts = N[n].bus_ts
                    if node_pos[bus_ts] > pos2
                        cus_cost_affected += insert_cost
                        # println("insert cost: $cus_cost_affected for $n")
                    elseif node_pos[bus_ts] > pos1
                        cus_cost_affected += insert_cost1
                        # println("insert cost1: $cus_cost_affected for $n")
                    end
                end
            end
        elseif i == pos1
            inserted_cus_cost += tt[n, pathID[i+1]] # the cost specifically for the inserted customer
        elseif i < pos2
            inserted_cus_cost += tt[n, pathID[i+1]] # the cost specifically for the inserted customer
            if isa(N[n], Pickup)
                d = N[n].dropoffID
                bus_ts = N[n].bus_ts
                if N[d].bus == bus
                    if node_pos[d] > pos2
                        cus_cost_affected += insert_cost2
                        # println("insert cost2: $cus_cost_affected for $n")
                        # cus_indicator[n] = 1 # forbid repetitive check later for dropoff
                    end
                elseif (bus_ts > 0) && (bus_ts in pathID)           
                    if node_pos[bus_ts] > pos2
                        cus_cost_affected += insert_cost2
                        # println("insert cost2: $cus_cost_affected for $n")
                    end
                end
            elseif isa(N[n], Droppoff)
                bus_ts = N[n].bus_ts
                if (bus_ts > 0) && (bus_ts in pathID) &&(node_pos[bus_ts] < pos1)
                    cus_cost_affected += insert_cost1
                    # println("insert cost1: $cus_cost_affected for $n")
                end
            end
        elseif i > pos2
            if isa(N[n], Droppoff)
                bus_ts = N[n].bus_ts
                if (bus_ts > 0) && (bus_ts in pathID)
                    if (node_pos[bus_ts] < pos2)
                        cus_cost_affected += insert_cost
                        # println("insert cost: $cus_cost_affected for $n at bus_ts $bus_ts")
                    elseif node_pos[bus_ts] < pos2 
                        cus_cost_affected += insert_cost2
                        # println("insert cost2: $cus_cost_affected for $n")
                    end
                end
            end
        end

    end
    # @info cus_cost_affected, inserted_cus_cost
    # Step 2: Get the full ride time if n2 is a dropoff or n1's last mile walking
    if isa(N[n1], Pickup)
        if isa(N[n2], TransitStation)
            d = N[n1].dropoffID      
            if N[d].walk_ts > 0
                inserted_cus_cost += sol.params.A_ts[(n2, N[d].walk_ts)]
                inserted_cus_cost += sol.params.A_walk[(d, N[d].walk_ts)]
            end
            if N[d].bus_ts > 0
                # error("Customer $n1 last mile transit station should not be assigned to $(N[d].bus_ts)")
            end
        elseif isa(N[n2], Droppoff)
            # sol.obj.cus_cost[n1, 4] = inserted_cus_cost
        end
    elseif isa(N[n1], TransitStation) && isa(N[n2], Droppoff)
        p = N[n2].pickupID
        if N[p].walk_ts > 0
            inserted_cus_cost += sol.params.A_ts[(N[p].walk_ts, n1)]
            inserted_cus_cost += sol.params.A_walk[(p, N[p].walk_ts)]
        elseif N[p].bus_ts > 0
            inserted_cus_cost += sol.params.A_ts[(N[p].bus_ts, n1)]
            inserted_cus_cost += firstmile_cost(N[p], sol)[3] # first mile cost
        else
            error("Customer $p is not served but require for insert last mile")
        end
        sol.obj.cus_cost[p,4] = inserted_cus_cost
    end
    # @info insert_cost, cus_cost_affected, inserted_cus_cost
    total = sum([insert_cost, cus_cost_affected + inserted_cus_cost] .* sol.params.λ[1:2])
    return total
end

# Caculate the addtional objective value when add one node to a bus
function objective_value(sol::Solution, bus::Int64, n::Int64)
    tt, A_ts, A_walk = sol.params.tt, sol.params.A_ts, sol.params.A_walk
    N = sol.N
    route = sol.routes[bus]
    # bus_cost = sol.obj.bus_cost
    # bus_cost[bus] = 0
    pathID = get_path(route)
    node_pos = route.node_pos
    inserted_cus_cost = 0.0 # timetime n1 -> n2
    insert_cost = tt[route.pre[n], n] + tt[n, route.suc[n]] - tt[route.pre[n], route.suc[n]]
    cus_cost_affected = 0.0 # customers affected by inserting these two nodes
    for i in 1:route.len - 1
        n_cur = pathID[i]
        pos = node_pos[n]
        # bus_cost[bus] += tt[n, pathID[i+1]] # bus cost
        if i < pos
            if isa(N[n_cur], Pickup)
                d = N[n_cur].dropoffID
                if N[d].bus == bus
                    if node_pos[d] > pos
                        cus_cost_affected += insert_cost
                    end
                elseif (N[n_cur].bus_ts > 0) && (N[n_cur].bus_ts in pathID)
                    bus_ts = N[n_cur].bus_ts
                    if node_pos[bus_ts] > pos
                        cus_cost_affected += insert_cost
                    end
                end
            end
        elseif i > pos
            if isa(N[n_cur], Droppoff)
                bus_ts = N[n_cur].bus_ts
                if (bus_ts > 0) && (bus_ts in pathID) && (node_pos[bus_ts] < pos)
                    cus_cost_affected += insert_cost
                end
            end
        end
    end
    if isa(N[n], Pickup)
        inserted_cus_cost = firstmile_cost(N[n], sol)[3]
    elseif isa(N[n], Droppoff)
        p = N[n].pickupID
        inserted_cus_cost = customer_ridetime!(p, sol)
    else
        error("Insert one node $n")
    end
    # @info cus_cost_affected, inserted_cus_cost
    total = sum([insert_cost, cus_cost_affected + inserted_cus_cost] .* sol.params.λ[1:2])
    return total
end

function objective_func(λ::Vector, bus_cost::Float64, cus_cost::Float64, penalty::Float64)
    return sum([bus_cost, cus_cost, penalty] .* λ)
end

function addtional_obj_cost(bus_cost::Float64, cus_cost::Float64, sol::Solution)
    λ = sol.params.λ
    penalty = 0.0
    return sum([bus_cost, cus_cost, penalty] .* λ)
end

# Only ride wihtout waiting time
function customer_ridetime!(p::Int64, sol::Solution)::Float64
    N = sol.N
    A_walk, A_ts, tt = sol.params.A_walk, sol.params.A_ts, sol.params.tt
    pickup = N[p]
    d = N[p].dropoffID
    dropoff = N[d]
    cus_cost = 0.0
    if isservedts(pickup)
        # First mile
        firstmile, ts1, cost_firstmile, walk_first = firstmile_cost(pickup, sol)
        cus_cost += cost_firstmile
        # Last mile
        lastmile, ts2, cost_lastmile, walk_last = lastmile_cost(dropoff, sol)
        cus_cost += cost_lastmile
        # TS mile
        sol.obj.cus_cost[p,3] = A_ts[(ts1,ts2)]
        cus_cost += sol.obj.cus_cost[p,3]
        # walking cost
        sol.obj.cus_cost[p,1] = walk_first * cost_firstmile + walk_last * cost_lastmile
        # bus cost
        sol.obj.cus_cost[p,2] = !walk_first * cost_firstmile + !walk_last * cost_lastmile
        # path
        customers[p] = [firstmile; lastmile]

    else
        sol.obj.cus_cost[p,1] = 0.0 # no walking
        sol.obj.cus_cost[p,3] = 0.0 # no ts
        cus_cost, cus_path = cus_inbus_cost(pickup, sol)
        sol.obj.cus_cost[p,2] = cus_cost
        customers[p] = cus_path # path
        # sol.customers[p] = cus_path # path
    end
    sol.obj.cus_cost[p,4] = cus_cost
    return cus_cost
end

function firstmile_cost(pickup::Pickup, sol::Solution)
    tt = sol.params.tt
    A_walk = sol.params.A_walk
    p = pickup.ID
    # First mile
    if pickup.walk_ts > 0
        walk = true
        ts1 = pickup.walk_ts
        firstmile = [p,ts1]; cost = A_walk[(p, ts1)]
    else
        walk = false
        ts1 = pickup.bus_ts
        route = sol.routes[pickup.bus]
        path = get_path(route)
        head = route.node_pos[p]
        tail = route.node_pos[ts1]
        if isnothing(head) | isnothing(tail)
            @error "Customer's first mile $p->$ts1 not served by bus$(pickup.bus): head = $head, tail = $tail"
        end
        firstmile = path[head:tail]
        cost = sum(tt[firstmile[i],firstmile[i+1]] for i in 1:length(firstmile)-1)
    end
    return firstmile, ts1, cost, walk
end

function lastmile_cost(droppoff::Droppoff, sol::Solution)
    tt = sol.params.tt
    A_walk = sol.params.A_walk
    d = droppoff.ID
    # First mile
    if droppoff.walk_ts > 0
        walk = true
        ts2 = droppoff.walk_ts
        lastmile = [ts2,d]; cost = A_walk[(d,ts2)]
    else
        walk = false
        ts2 = droppoff.bus_ts
        route = sol.routes[droppoff.bus]
        path = get_path(route)
        head = route.node_pos[ts2]
        tail = route.node_pos[d]
        if isnothing(head) | isnothing(tail)
            @error "Customer's last mile $ts2->$d not served by bus$(droppoff.bus): head = $head, tail = $tail"
        end
        lastmile = path[head:tail]
        cost = sum(tt[lastmile[i],lastmile[i+1]] for i in 1:length(lastmile)-1)
    end
    return lastmile, ts2, cost, walk
end

# Calculate customer travel cost if their pickup and dropoff are in the same bus
function cus_inbus_cost(pickup::Pickup, sol::Solution)
    bus = pickup.bus
    d = pickup.dropoffID
    p = pickup.ID
    droppoff = sol.N[d]
    if sol.N[p].bus != sol.N[d].bus
        error("Bus of customer $p pickup and droppoff is not the same $((pickup.bus, droppoff.bus))")
    end
    tt = sol.params.tt
    bus_path = get_path(sol.routes[bus])
    # head = findfirst(bus_path .== p)
    # tail = findfirst(bus_path .== d)
    head = sol.routes[bus].node_pos[p]
    tail = sol.routes[bus].node_pos[d]
    cus_path = bus_path[head:tail]
    cost = sum(tt[cus_path[i], cus_path[i+1]] for i in 1:length(cus_path)-1)
    return cost, cus_path
end

# customer travel time (with waiting)
function customer_traveltime(p::Int64, sol::Solution)::Float64
    if !isserved(sol.N[p]) return 0.0 end
    pickup = sol.N[p]
    d = pickup.dropoffID
    droppoff = sol.N[d]
    bus_p = pickup.bus 
    bus_d = droppoff.bus 
    if (bus_p == bus_d) && (bus_p != 0)
        # Bus from origin to destination
        route = sol.routes[bus_p]
        # path = get_path(route)
        idx_p = route.node_pos[p]
        idx_d = route.node_pos[d]
        return route.A[idx_d] - route.D[idx_p]
    else
        # First mile
        if (bus_p == 0) & (pickup.walk_ts != 0)
            # First mile is walking
            ts = pickup.walk_ts
            dep_time = sol.N[ts].dep_time - sol.params.A_walk[(p,ts)]
        elseif bus_p !=0
            # First mile by bus
            idx_p = sol.routes[bus_p].node_pos[p]
            dep_time = sol.routes[bus_p].D[idx_p]
        else
            error("Customer $p first mile is not assigned")
        end
        # Last mile
        if (bus_d == 0) & (droppoff.walk_ts != 0)
            # Last mile walking
            ts = droppoff.walk_ts
            arr_time = sol.N[ts].arr_time + sol.params.A_walk[(d,ts)]
        elseif bus_d !=0
            # Last mile by bus
            idx_d = sol.routes[bus_d].node_pos[d]
            arr_time = sol.routes[bus_d].A[idx_d]
        else
            error("Customer $p last mile is not assigned")
        end
        return arr_time - dep_time
    end 
end
