
function repair(method::Function, sol::Solution, customers::Vector{Int64})::Solution
    return method(sol, customers)
end

function repair_standard(sol::Solution, customers::Vector{Int64})::Solution
    arcs_ts = Set(keys(sol.params.A_ts))
    for cus in customers
        if isserved(sol.N[cus]) return sol end
        A_walk = sol.params.A_walk
        A_ts = sol.params.A_ts
        # Step 1: assgin walkable transit station to customer
        d = sol.N[cus].dropoffID
        cus_d = sol.N[d]
        n_c = sol.N[cus].dropoffID - cus
        if !isempty(sol.N[cus].close_ts) && !isempty(cus_d.close_ts)
            # Step 1.1: find customers can walk both to/from transit stations; no bus needed
            for ts_o in sol.N[cus].close_ts
                for ts_d in cus_d.close_ts
                    if (ts_o,ts_d) ∈ arcs_ts
                        if A_walk[(cus,ts_o)] + A_ts[(ts_o,ts_d)] + A_walk[(cus+n_c,ts_d)] <= sol.N[cus].L
                            sol.N[cus].walk_ts = ts_o # update: first mile is connected by walking
                            cus_d.walk_ts = ts_d # update: first mile is connected by walking
                            sol.servedcus[cus] = true
                            # push!(sol.servedcus_set, cus)
                        end
                    end    
                end
            end
        elseif !isempty(sol.N[cus].close_ts) && isempty(cus_d.close_ts)
            # Step 1.2: find customers first mile walk, last mile by bus
            pairs = k_nearest_ts(sol.N[cus], cus_d, :last, k = DEGREE_TS_EXPLORATION)
            if !isempty(pairs)
                ts_o, ts_d = rand(pairs)
                sol.N[cus].walk_ts = ts_o # update the first mile 
                sol_list, cost_list, _ = randomsearch_lastmile(sol, sol.N[ts_d], sol.N[d])
                sol, cost = sol_list[1], cost_list[1]
                if !isserved(sol.N[d])
                    sol.N[cus].walk_ts = 0
                end
            end
            
        elseif isempty(sol.N[cus].close_ts) && !isempty(cus_d.close_ts)
            # Step 1.3: find customers first mile by bus, last mile walk
            pairs = k_nearest_ts(sol.N[cus], cus_d, :first, k = DEGREE_TS_EXPLORATION)
            if !isempty(pairs)
                ts_o, ts_d = rand(pairs)
                sol.N[d].walk_ts = ts_d
                # greedy insert
                sol_list, cost_list, _ = randomsearch_firstmile(sol, sol.N[cus], sol.N[ts_o])
                sol, cost = sol_list[1], cost_list[1]
                if !isserved(sol.N[cus])
                    sol.N[d].walk_ts = 0
                end
            end
        end

        # Step 2: Insert customer to the first-closest ts stations
        if !isserved(sol.N[cus]) 
            ts_pair = sol.N[cus].ts_pair
            if !isempty(ts_pair)
                pair = rand(ts_pair)
                sol_list, cost_list, _ = randomsearch_bothmile(sol, sol.N[cus], [pair])
                sol, cost = sol_list[1], cost_list[1]
            end
        end

        # Step 3: Customer is served directly by bus
        if !isserved(sol.N[cus]) 
            sol_list, cost_list, _ = randomsearch_firstmile(sol, sol.N[cus], sol.N[d])
            sol, cost = sol_list[1], cost_list[1]
        end
    end

    return sol
end

# Repaire two customers with common ts pair with the least cost
# function repair_TSpair(sol::Solution, customers::Tuple{Int64, Int64})
function repair_TSpair(sol::Solution, customers::Vector{Int64})::Solution
    len_cus = length(customers)
    if len_cus < 2
        return repair_standard(sol, customers)
    end
    # Step 1: Find customers with common pair
    cus_repair, common_pairs = find_common_pair(sol, customers)
    # Step 2: assgine the pair with minimum cost
    tt, A_ts, num_cus = sol.params.tt, sol.params.A_ts, sol.params.num_cus
    best_pair, min_cost = nothing, Inf
    for pair in common_pairs
        cost = 0
        for cus in cus_repair
            cus_cost = tt[cus, pair[1]] + A_ts[pair] + tt[pair[2], cus + num_cus]
            cost += cus_cost
        end
        if cost < min_cost
            best_pair = pair
            min_cost = cost
        end
    end
    # Step 3: repair customers with the best pair found in step 2
    if !isnothing(best_pair)
        for cus in cus_repair
            sol_list, _ = randomsearch_bothmile(sol, sol.N[cus], [best_pair])
            sol = sol_list[1]
        end
    end

    # Step 4: repair customers left
    cus_repair = findall(!, sol.servedcus)
    # cus_repair = setdiff(1:sol.params.num_cus, sol.servedcus_set)
    sol = repair_greedy(sol, cus_repair)
    # sol = repair_greedy2(sol, cus_repair)
    return sol
end

function repair_random_order(sol::Solution, customers::Vector{Int64})::Solution
    shuffle!(customers)
    # @show customers
    while !isempty(customers)
        cus = pop!(customers)
        sol = repair_greedy_cus(sol, cus, output_cost = false)
    end
    return sol
end

function repair_regret(sol::Solution, customers::Vector{Int64}; regret::Int64 = DEGREE_REGRET)::Solution
    len_cus = length(customers)
    while !isempty(customers)
        customer_sols = Vector{Solution}(undef, len_cus)
        regret_val = Vector{Float64}(undef, len_cus)
        best_sol, best_cost = nothing, Inf
        for (i,cus) in enumerate(customers)
            best_sol, costs = repair_greedy_cus(sol, cus, output_cost = true, regret = regret)
            best_cost = costs[1]
            # value = sum(costs[i]-best_cost for i in 2:regret)
            value = sum(costs[2:regret].-best_cost)
            customer_sols[i] = best_sol
            regret_val[i] = value
        end
        odr = sortperm(regret_val)
        regret_val = regret_val[odr]
        customer_sols = customer_sols[odr]
        customers = customers[odr]
        popfirst!(customers)
        len_cus -= 1
        sol = copy(customer_sols[1])
    end
    return sol
end

function repair_greedy(sol::Solution, customers::Vector{Int64})::Solution
    while !isempty(customers)
        len_cus = length(customers)
        customer_sols = Vector{Solution}(undef, len_cus)
        customer_sol_cost = Vector{Float64}(undef, len_cus)
        best_sol, best_cost = nothing, Inf
        for (i,cus) in enumerate(customers)
            best_sol, best_cost = repair_greedy_cus(sol, cus, output_cost = true)
            customer_sols[i] = best_sol
            customer_sol_cost[i] = best_cost
        end
        odr = sortperm(customer_sol_cost)
        customer_sol_cost = customer_sol_cost[odr]
        customer_sols = customer_sols[odr]
        customers = customers[odr]
        # @info customers, customer_sol_cost
        popfirst!(customers)
        sol = copy(customer_sols[1])
    end
    return sol
end

function repair_greedy2(sol::Solution, customers::Vector{Int64})::Solution
    len_cus = length(customers)
    customer_sols = Vector{Solution}(undef, len_cus)
    customer_sol_cost = Vector{Float64}(undef, len_cus)
    best_sol, best_cost = nothing, Inf
    for (i,cus) in enumerate(customers)
        best_sol, best_cost = repair_greedy_cus(sol, cus, output_cost = true)
        customer_sols[i] = best_sol
        customer_sol_cost[i] = best_cost
    end
    odr = sortperm(customer_sol_cost)
    customer_sol_cost = customer_sol_cost[odr]
    customer_sols = customer_sols[odr]
    customers = customers[odr]
    # @info customers, customer_sol_cost
    popfirst!(customers)
    sol = copy(customer_sols[1])
    for cus in customers
        sol = repair_greedy_cus(sol, cus, output_cost = false)
    end
    return sol
end