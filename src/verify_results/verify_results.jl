function print_customer_with_ts(sol::Solution)
    n_c = sol.params.num_cus
    N = sol.N
    num_ts_pair = 0
    for c in 1:n_c
        ts_pair = N[c].ts_pair
        if !isempty(N[c].ts_pair)
            println("$c: ", ts_pair)
            num_ts_pair += length(ts_pair)
        end
    end
    return num_ts_pair
end

function print_customer_with_ts()
    s₀ = initial_solution(eidarp);
    n_c = sol.params.num_cus
    N = sol.N
    for c in 1:n_c
        if !isempty(N[c].ts_pair)
            println("$c: ", N[c].ts_pair)
        end
    end
end

print_customer_with_ts(s₀)