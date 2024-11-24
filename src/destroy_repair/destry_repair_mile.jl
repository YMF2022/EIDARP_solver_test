# Function to destroy existing midle mile and repair a random one. It is a way to apply some "noise"
function destroy_repair_mile(sol::Solution)
    customers = findall(sol.servedcus)
    # customers = sol.servedcus_set
    # len_cus = length(customers)
    # customers = rand(customers, Int(ceil(0.1*len_cus)))
    customers = shuffle(customers)
    for p in customers
        if isservedts(sol.N[p])
            d = sol.N[p].dropoffID
            s_tmp = copy(sol)
            node_p = s_tmp.N[p]; node_d = s_tmp.N[d]
            if (node_p.bus_ts > 0) && (node_d.bus_ts > 0)
                # case 1: customer take bus to/from transit station
                ts1 = node_p.bus_ts 
                ts2 = node_d.bus_ts
                ts_pair = node_p.ts_pair
                ts1_other = find_different_ts(ts1, ts2, ts_pair, :first)
                ts2_other = find_different_ts(ts1, ts2, ts_pair, :last)
                if isempty(ts1_other) && isempty(ts2_other)
                    return sol
                elseif isempty(ts2_other)
                    # Remove and search the first mile
                    remove_customer_firstmile!(s_tmp, p)
                    # s_tmp_list, costs = search(greedysearch, s_tmp, s_tmp.N[p], s_tmp.N[rand(ts1_other)])
                    s_tmp_list, costs, _ = randomsearch_firstmile(s_tmp, s_tmp.N[p], s_tmp.N[rand(ts1_other)])
                    s_tmp, cost = s_tmp_list[1], costs[1]
                    if cost == Inf return sol
                    else return s_tmp end
                elseif isempty(ts1_other)
                    # Remove and search the last mile
                    remove_customer_lastmile!(s_tmp, p)
                    # best_sol = best_lastmile(s_tmp, p, ts2_other)
                    # s_tmp_list, costs = search(greedysearch, s_tmp, s_tmp.N[rand(ts2_other)], s_tmp.N[d])
                    s_tmp_list, costs, _ = randomsearch_last(s_tmp, s_tmp.N[rand(ts2_other)], s_tmp.N[d])
                    s_tmp, cost = s_tmp_list[1], costs[1]
                    if cost == Inf return sol
                    else return s_tmp end
                else
                    # Remove and search another ts pair
                    ts_pair_other = setdiff(ts_pair, [(ts1, ts2)])
                    remove_customer!(s_tmp, p)
                    # sol_list, cost_list = search(greedysearch, s_tmp, s_tmp.N[p], ts_pair_other)
                    sol_list, cost_list, _ = randomsearch_bothmile(s_tmp, s_tmp.N[p], ts_pair_other)
                    if cost_list[1] != Inf
                        weights = generate_skewed_weights(cost_list)
                        s_tmp = sample(sol_list, Weights(weights))
                        return s_tmp
                    else
                        return sol
                    end
                end
            elseif node_p.bus_ts > 0
                # customer walk last mile
                ts1 = node_p.bus_ts 
                ts2 = node_d.walk_ts
                ts_pair = node_p.ts_pair
                ts1_other = find_different_ts(ts1, ts2, ts_pair, :first)
                if isempty(ts1_other) return sol end
                # Remove and search the first mile
                remove_customer_firstmile!(s_tmp, p)
                s_tmp_list, costs, _ = randomsearch_firstmile(s_tmp, s_tmp.N[p], s_tmp.N[rand(ts1_other)])
                s_tmp, cost = s_tmp_list[1], costs[1]
                if cost == Inf return sol
                else return s_tmp end
            elseif node_d.bus_ts > 0
                # customer walk first mile
                ts1 = node_p.walk_ts
                ts2 = node_d.bus_ts 
                ts_pair = node_p.ts_pair
                ts2_other = find_different_ts(ts1, ts2, ts_pair, :last)
                if isempty(ts2_other) return sol end
                # Remove and search the last mile
                remove_customer_lastmile!(s_tmp, p)
                ts2 = rand(ts2_other)
                s_tmp_list, costs, _ = randomsearch_lastmile(s_tmp, s_tmp.N[ts2], s_tmp.N[d])
                s_tmp, cost = s_tmp_list[1], costs[1]
                if cost == Inf return sol
                else return s_tmp end
            else
                # customer walk both miles
                return sol
            end
        end
    end
    return sol
end