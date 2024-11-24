destroy!(method::Function, sol::Solution, number::Int64) = method(sol, number)

function random_removal!(sol::Solution, number::Int64)
    # remove randomly selected customers
    update_served_customer!(sol)
    customers = findall(sol.servedcus)
    cus_remove = sample(customers, number; replace=false)
    for c in cus_remove
        remove_customer!(sol, c)
    end 
    return cus_remove
end

function worst_removal!(sol::Solution, number::Int64)
    # remove customers with the most savings
    update_served_customer!(sol)
    customers = findall(sol.servedcus)
    remove_cost= [(calculate_cus_remove_cost(sol, p), p) for p in customers]
    # cus_remove = [x[2] for x in sort(cost_with_customers, by = x -> x[1], rev = true)][1:number]
    sorted_idx = sortperm(remove_cost, rev = true)
    # cost = cost[sorted_idx]
    cus_remove = customers[sorted_idx]
    cus_remove = cus_remove[1:number]
    for c in cus_remove
        remove_customer!(sol, c)
    end
    
    return cus_remove
end

function related_removal!(sol::Solution, number::Int64)
    # Molenbruch's function
    tt = sol.params.tt
    common_pair = sol.params.common_pair
    d_max = maximum(tt)
    t_end = sol.params.t_end
    if isempty(common_pair)
        max_pair_num = Inf
    else
        max_pair_num = maximum(length.(collect(values(common_pair))))
    end
    num_cus = sol.params.num_cus
    customers = findall(sol.servedcus)
    len_cus = sum(sol.servedcus)
    # customers = sol.servedcus_set
    # len_cus = length(customers)    
    weights = [1/3, 1/3, 1/3]
    relate_list = zeros(len_cus-1)
    cus_list = zeros(Int64, len_cus-1)
    l = 0
    p1 = rand(customers)
    d1 = p1 + num_cus
    t_dep1 = get_dep_time(sol.N[p1], sol)
    t_arr1 = get_arr_time(sol.N[d1], sol)
    customers = filter!(x->x!=p1, customers)
    for i in 1:len_cus-1
        p2 = customers[i]
        d2 = p2 + num_cus
        if p2 == p1
            continue
        end
        t_dep2 = get_dep_time(sol.N[p2], sol)
        t_arr2 = get_arr_time(sol.N[d2], sol)
        if haskey(common_pair, (p1,p2))
            num_pair = length(common_pair[(p1,p2)])
        else
            num_pair = 0
        end
        elements = [(tt[p1,p2] + tt[d1,d2])/(2*d_max), # spatial
                    (abs(t_dep1 - t_dep2) + abs(t_arr1 - t_arr2))/(2*t_end), # time
                    num_pair/max_pair_num] # the number of same ts_pairs
        relateness = sum(weights .* elements)
        l += 1
        relate_list[l] = relateness
        cus_list[l] = p2
    end
    odr = sortperm(relate_list, rev=true)
    relate_list = relate_list[odr]
    cus_list = cus_list[odr]
    cus_list = [p1; cus_list[1:number - 1]]
    for c in cus_list
        remove_customer!(sol, c)
    end
    return cus_list
end

function route_removal!(sol::Solution, number::Int64)
    used_bus = findall(sol.usedbus)
    bus_len = [sol.routes[k].len for k in used_bus]
    weights = 1 ./bus_len
    weights = weights / sum(weights) # normalize the weights
    k = sample(used_bus, Weights(weights))
    route = sol.routes[k]
    cus_list = zeros(Int64, route.len)
    i = 0
    pathID = get_path(route)
    for node_id in pathID
        node = sol.N[node_id]
        if isa(node, Pickup)
            remove_customer!(sol, node.ID)
            i += 1
            cus_list[i] = node.ID       
        elseif isa(node, Droppoff)
            c = node.pickupID
            if c ∉ cus_list
                remove_customer!(sol, c)
                i += 1
                cus_list[i] = c               
            end
        end
    end
    cus_list = cus_list[1:i]
    return cus_list
end

function cluster_removal()
    # remove customers arriving/leaving the same transit station
end

