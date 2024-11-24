
"""
Greedy repair one customer
"""
function repair_greedy_cus(sol::Solution, cus::Int64; output_cost::Bool = true, regret::Int64 = 1, out_sol::Bool = false)
    A_walk, A_ts = sol.params.A_walk, sol.params.A_ts
    arcs_ts = Set(keys(A_ts))
    tt = sol.params.tt
    best_sol, best_cost = sol, Inf
    sol_list = Vector{Solution}(undef, regret*100)
    costs = 3e5 * ones(regret*10) # All the feasible costs; infeasible will be a large number which is 100.0
    if isserved(sol.N[cus]) return sol end
    # Step 1: assgin walkable transit station to customer
    best_cost = Inf
    len_cost = 0
    # costs[len_cost] = best_cost
    # sol_list[len_cost] = best_sol
    d = sol.N[cus].dropoffID
    cus_d = sol.N[d]
    if !isempty(sol.N[cus].close_ts) && !isempty(cus_d.close_ts)
        # Step 1.1: find customers can walk both to/from transit stations; no bus needed
        for ts_o in sol.N[cus].close_ts
            for ts_d in cus_d.close_ts
                if (ts_o,ts_d) ∈ arcs_ts
                    # cost = A_walk[(cus,ts_o)] + A_ts[(ts_o,ts_d)] + A_walk[(d,ts_d)]
                    cost = objective_value(best_sol)
                    λ2 = sol.params.λ[2] # the weighted value for customer cost
                    if cost <= sol.N[cus].L
                        if cost < best_cost
                            best_cost = cost
                            # @info "cost of walk bot miles $best_cost"
                            best_sol = copy(sol)
                            best_sol.N[cus].walk_ts = ts_o # update: first mile is connected by walking
                            best_sol.N[d].walk_ts = ts_d # update: first mile is connected by walking
                            best_sol.servedcus[cus] = true
                            # push!(best_sol.customer, cus)
                            len_cost += 1
                            costs[len_cost] = cost 
                            sol_list[len_cost] = best_sol
                        end
                    end
                end    
            end
        end
    end

    if !isempty(sol.N[cus].close_ts)
        # Step 1.2: Customers first mile walk, last mile by bus
        pairs = k_nearest_ts(sol.N[cus], cus_d, :last, k = DEGREE_TS_EXPLORATION)
        if !isempty(pairs)
            # @info "Find last mile by bus for customer $cus"
            for (ts_o, ts_d) in pairs
                sol_tmp = copy(sol)
                sol_tmp.N[cus].walk_ts = ts_o # update the first mile 
                # sol_tmp_list, bus_cost_list = search(greedysearch, sol_tmp, sol_tmp.N[ts_d], sol_tmp.N[d], regret = regret)
                sol_tmp_list, bus_cost_list, len_add = search(sol_tmp, sol_tmp.N[ts_d], sol_tmp.N[d], regret = regret)
                # bus_cost_list here is customers travel cost + bus travel cost + affected customer cost
                # len_add = length(bus_cost_list) 
                if bus_cost_list[1] != Inf
                    # if there is feasible solution
                    # println("Succeful found the last mile for customer $cus")
                    sol_tmp = sol_tmp_list[1]
                    # global s_test = copy(sol_tmp)
                    cost = bus_cost_list[1]
                    if cost < best_cost
                        best_cost = cost
                        best_sol = sol_tmp
                    end
                    costs[len_cost+1:len_cost+len_add] = bus_cost_list
                    sol_list[len_cost+1:len_cost+len_add] = sol_tmp_list
                    len_cost += len_add
                else
                    # Feasible solution is not found
                    # println("Walk first mile: Not found the last mile for customer $cus")
                    sol_tmp.N[cus].walk_ts = 0
                end
            end
        end
    end
        
    if isempty(sol.N[cus].close_ts)
        # Step 1.3: find customers first mile by bus, last mile walk
        pairs = k_nearest_ts(sol.N[cus], cus_d, :first, k = DEGREE_TS_EXPLORATION)
        if !isempty(pairs)
            # ts_o, ts_d = rand(pairs)
            for (ts_o,ts_d) in pairs
                sol_tmp = copy(sol)
                sol_tmp.N[d].walk_ts = ts_d
                sol_tmp_list, bus_cost_list, len_add = search(sol_tmp, sol_tmp.N[cus], sol_tmp.N[ts_o], regret = regret)
                # bus_cost_list here is customers travel cost + bus travel cost + affected customer cost
                # len_add = length(bus_cost_list)
                # println("bus_cost_list:", bus_cost_list)
                if bus_cost_list[1] != Inf
                    # if there is a feasible solution
                    sol_tmp = sol_tmp_list[1]
                    cost = bus_cost_list[1]
                    if cost < best_cost
                        best_cost = cost
                        best_sol = sol_tmp
                    end
                    costs[len_cost+1:len_cost+len_add] = bus_cost_list
                    sol_list[len_cost+1:len_cost+len_add] = sol_tmp_list
                    len_cost += len_add
                end
            end
        end
    end

    # Step 2: Insert customer with the least cost ts pair
    ts_pair = sol.N[cus].ts_pair
    sol_tmp_list, bus_cost_list, len_add = search(sol, sol.N[cus], ts_pair, regret = regret)
    # len_add = length(bus_cost_list)
    if bus_cost_list[1] != Inf
        # If there is a feasible solution
        sol_tmp = sol_tmp_list[1]
        cost = bus_cost_list[1]
        if cost < best_cost
            best_cost = cost
            best_sol = sol_tmp
        end
        costs[len_cost+1:len_cost+len_add] = bus_cost_list
        sol_list[len_cost+1:len_cost+len_add] = sol_tmp_list
        len_cost += len_add
    end

    # Step 3: Customer is served directly by bus
    sol_tmp_list, bus_cost_list, len_add = search(sol, sol.N[cus], sol.N[d], regret = regret)
    # len_add = length(bus_cost_list)
    if bus_cost_list[1] != Inf
        sol_tmp = sol_tmp_list[1]
        cost = bus_cost_list[1]
        if cost < best_cost
            best_cost = cost
            best_sol = sol_tmp
        end
        costs[len_cost+1:len_cost+len_add] = bus_cost_list
        sol_list[len_cost+1:len_cost+len_add] = sol_tmp_list
        len_cost += len_add
    end

    costs_odr = costs[1:len_cost]
    sol_list = sol_list[1:len_cost]
    sol_odr = sortperm(costs_odr)
    sol_list = sol_list[sol_odr]
    # global cost_test = copy(costs)
    costs = costs[sortperm(costs)]
    # best_sol_list = best_cost == Inf ? sol : sol_list[1]
    # if best_sol_list != best_sol 
    #     @warn "Penalty cost needs to be reduced"
    #     best_value = objective_value(best_sol)
    #     value_list = objective_value(best_sol_list)
    #     error("Customer $cus best cost: $best_value and best_cost_list $value_list")
    #     error("Customer $cus best cost: $best_cost and best_cost_list $(costs[1])")
    # end
    if output_cost == false
        if len_cost > 1
            weights = generate_skewed_weights(costs_odr, decay_factor = 0.4)
            best_sol = sample(sol_list, Weights(weights))
        end
        return best_sol
    elseif output_cost == true
        if regret == 1
            if len_cost > 1
                weights = generate_skewed_weights(costs_odr, decay_factor = 0.6)
                j = sample(collect(1:len_cost), Weights(weights))
                best_sol = sol_list[j]
                best_cost = costs_odr[j]
            end
            return best_sol, best_cost
        elseif regret > 1
            if out_sol == false
                return best_sol, costs[1:regret]
            else
                return sol_list[1:len_cost], costs[1:len_cost]
            end
        end
    end
end