struct VNSParameter
    n_iter::Int64
    t_max::Float64                      # T_max = t_max * c̄
    T_red::Float64
    α1::Float64                         # The parameter control the first local search 
    α2::Float64                         # The parameter constrol the second local search
    destroy_operators::Set{Function}    # destroy operators
    repair_operators::Set{Function}     # repair operators 
    early_stop::Bool                    # early stop or not
    no_improve::Int64                   # the number of iteration without improving value if early stop
end

function VNS(sol::Solution, vns_params::VNSParameter, t_start::Float64, degree_destroy::Vector{Int64}, c̄::Float64; is_rejection = false)
    n_iter = vns_params.n_iter
    T_max = vns_params.t_max * c̄
    T = T_max
    T_red = vns_params.T_red
    i_new_sol = 0
    count_change_neighborhood = 0
    results = zeros(Float64, vns_params.n_iter, 3)
    result_local_search = zeros(n_iter, 4)
    # destroy_methods = rand(vns_params.destroy_operators, vns_params.n_iter)
    # repair_methods = rand(vns_params.repair_operators, vns_params.n_iter)

    progress = Progress(n_iter-1, desc="Computing...", color=:blue, showspeed=true)
    i = 1
    c_best = c = objective_value(sol)
    s_best = s = sol
    while i <= n_iter
        # Step 1: Shaking
        s′ = copy(s)
        
        num_destroy = min(degree_destroy[i], sum(s′.servedcus))
        # destroy_method = destroy_methods[i]
        # cus = destroy_method(s′, num_destroy)
        dsy_opt = rand(vns_params.destroy_operators)
        cus_repair = destroy!(dsy_opt, s′, num_destroy)
        union!(cus_repair, findall(!, s′.servedcus))
        if is_rejection
            cus_repair = sample(cus_repair, num_destroy, replace=true)
        end
        cus_repair = unique(cus_repair) # organize the unserved customer wihtout disturbing the order
        rpr_opt = rand(vns_params.repair_operators)
        s′ = repair(rpr_opt, s′, cus_repair)  
        # println("repaired cus at iteration $i", cus, rpr_opt)
        # repair_method = repair_methods[i]
        # s′ = repair_method(s′, cus)
        if rand() <= 0.1
            s′ = destroy_repair_mile(s′)
        end
        c′ = s′.objective_cost
        if c′ < vns_params.α1 * c_best
            s″ = s′
            c″ = c′
            # Step 2: Local search
            result_local_search[i, 1] = 1
            s″, c″ = change_depot(s″, c″)
            s″, c″ = replace_mile(s″, c″)
            s″, c″ = remove_insert_mile(s″, c″)
            s″, c″ = remove_ts_line(s″, c″)
            s″, c″ = replace_walk(s″, c″)
            # recharge!()
            # for k in findall(s″.usedbus)
            #     amount, _ = energy_check(k, s″)
            #     if amount > 1e-3
            #         recharge!(k, s″)
            #     end
            # end
            # c″ = objective_value(s″)
            if c″ < c_best
                result_local_search[i, 2] = c″
            end
        else
            s″ = s′
            c″ = c′
        end
        c_test = c″
        # Step 3: Move or not
        T = T - T_max/T_red
        if T < 0 T = rand()*T_max end
        # Accept new neighborhood with deterministic annealing
        if c″ < c_best + T
            count_change_neighborhood += 1
            if c″ >= vns_params.α2 * c_best
                result_local_search[i, 3] = 1
                # Local search
                s″, c″ = change_depot(s″, c″)
                s″, c″ = replace_mile(s″, c″)
                s″, c″ = remove_insert_mile(s″, c″)
                s″, c″ = remove_ts_line(s″, c″)
                s″, c″ = replace_walk(s″, c″)
                if c″ < c_best
                    result_local_search[i, 4] = c″
                end
            end
            s = s″
            c = c″
            if c″ > c_test error("2 local search gives worse results") end
        end
        # Step 4: Update results
        if c″ < c_best
            s_best = s″
            c_best = c″
            i_new_sol = 0
        else
            i_new_sol += 1
            if i < n_iter
                if degree_destroy[i] < DEGREE_DESTROY + 1
                    degree_destroy[i+1] = degree_destroy[i] + 1
                end
            end
            # EARLY_STOP: no better solution after certain number of iterations
            if (vns_params.early_stop == true) && (i_new_sol == vns_params.no_improve)
                results[i,:] .= [c_best, c′, c″]
                results = results[1:i,:]
                break
            end      
        end
        if c′ < c″
            @error("The results of c′ $c′ is smaller than c″$c″ at iteration $i")
        end
        results[i,:] .= [c_best, c′, c″]
        i += 1
        next!(progress)
    end
    # c_final = objective_value_full(s)
    # @assert c_final == c
    t_solve = time() - t_start
    println("Final best: $c_best with solving time $t_solve s.")
    return s_best, c_best, results, i, t_solve, count_change_neighborhood, result_local_search
end


