function initial_solution(eidarp::EIDARP)::Solution
    n_c = eidarp.n_c
    sol = Solution(eidarp)
    A_ts, A_walk = sol.params.A_ts, sol.params.A_walk
    
    # Step 1: assgin customer near transit station to trainsit station
    for cus_o in sort(sol.N[1:n_c], by = n -> n.timewindow[1])
        # @show sol.unservedcus, cus_o.ID
        c = cus_o.ID
        cus_d = sol.N[c+n_c]
        if !isempty(cus_o.close_ts) && !isempty(cus_d.close_ts)
            # Step 1.1: find customers can walk both to/from transit stations; no bus needed
            for ts_o in cus_o.close_ts
                for ts_d in cus_d.close_ts
                    if (ts_o,ts_d) ∈ eidarp.arcs_ts
                        if A_walk[(c,ts_o)] + A_ts[(ts_o,ts_d)] + A_walk[(c+n_c,ts_d)] <= cus_o.L
                            cus_o.walk_ts = ts_o # update: first mile is connected by walking
                            cus_d.walk_ts = ts_d # update: first mile is connected by walking
                            sol.servedcus[c] = true
                            # push!(sol.servedcus_set, c)
                            # @info "Customer $c walk for both first and last mile"
                        end
                    end    
                end
            end
        elseif !isempty(cus_o.close_ts) && isempty(cus_d.close_ts)
            # Step 1.2: find customers first mile walk, last mile by bus
            pairs = k_nearest_ts(cus_o, cus_d, :last, k = 1)
            if !isempty(pairs)
                ts_o, ts_d = pairs[1]
                # @info "$c first mile walk, last mile by bus"
                cus_o.walk_ts = ts_o # update the first mile 
                # sol_list, cost_list = search(randomsearch, sol, sol.N[ts_d], cus_d)
                sol_list, cost_list, _ = randomsearch_lastmile(sol, sol.N[ts_d], cus_d)
                sol, cost = sol_list[1], cost_list[1]
                if !isserved(sol.N[c+n_c])
                    sol.N[c].walk_ts = 0
                end
            end
            
        elseif isempty(cus_o.close_ts) && !isempty(cus_d.close_ts)
            # Step 1.3: find customers first mile by bus, last mile walk
            pairs = k_nearest_ts(cus_o, cus_d, :first, k = 1)
            if !isempty(pairs)
                ts_o, ts_d = pairs[1]
                # @info "$c walk the last mile, fist mile by bus to $ts_o"
                sol.N[c+n_c].walk_ts = ts_d
                # greedy insert
                # sol_list, cost_list = search(randomsearch, sol, sol.N[c], sol.N[ts_o])
                sol_list, cost_list, _ = randomsearch_firstmile(sol, sol.N[c], sol.N[ts_o])
                sol, cost = sol_list[1], cost_list[1]
                if !isserved(sol.N[c])
                    sol.N[c+n_c].walk_ts = 0
                end
            end
        end
    end

    # Step 2: for the rest customersfind the ts pairs if at the closest ts stations
    unservedcus = findall(!, sol.servedcus)
    # unservedcus = setdiff(1:sol.params.num_cus, sol.servedcus_set)
    for c in unservedcus
        ts_pairs = sol.N[c].ts_pair
        if isempty(ts_pairs) continue end
        sol_list, cost_list, _ = randomsearch_bothmile(sol, sol.N[c], ts_pairs)
        sol, cost = sol_list[1], cost_list[1]
    end

    # Step 3: for customers cannot be assigned to ts pairs are served directly by bus
    unservedcus = findall(!, sol.servedcus)
    # unservedcus = setdiff(1:n_c, sol.servedcus_set)
    for c in unservedcus
        cus_o, cus_d = sol.N[c], sol.N[c+n_c]
        sol_list, cost_list, _ = randomsearch_firstmile(sol, cus_o, cus_d)
        sol, cost = sol_list[1], cost_list[1]
    end
    # @show served, sol.unservedcus

    return sol
end
