using CSV, DataFrames, Distances
# using ProgressBars
using ProgressMeter
using Plots
using Random
using DataStructures
using StatsBase

import Base.copy

include("data_structure/datastructure.jl")
include("EIDARP/EIDARPInstance.jl")
using .EIDARPInstance
include("outputs.jl")
include("build_solution.jl")
include("utils/operations.jl")
include("feasibility.jl")
include("initialize.jl")
include("insertion_search/search.jl")
include("objectivefun.jl")
include("local_operators/local_operator.jl")
include("destroy_repair/destroy_repair.jl")
include("recharge/recharge.jl")
include("VNS.jl")

@warn "add ts mile cost in the randomsearch ts pair"
@warn "objective_value for add one node"
Random.seed!(8) # 5


#=
1. Remove, Destoy, Local operators
=#
# repair_TSpair
# rpr_methods = Set([repair_TSpair, repair_greedy, repair_random_order, repair_regret])
# dsy_methods = Set([random_removal!, worst_removal!, related_removal!, route_removal!])
# Generate destroy and repair dsy_methods
# destroy_methods = rand(dsy_methods, vns_params.n_iter);
# repair_methods = rand(rpr_methods, vns_params.n_iter);

#=
2. Parameters for EIDARP
=#
const T_END             = 105.0
const WAITTIME          = true # whether or not waiting time is included in ridetime
const ω                 = 200 # Penalty cost of rejecting customer
const initE             = 1.0 # 100% of initial battery level
const minElevel         = 0.1 # 10 % is the minimum battery level
const MAX_WAIT_TIME     = 10.0
# global MAX_WAIT_TIME   = eidarp.max_waittime

#=
3. Parameters for Algorithm
=#
iteration = 500
vns_params = VNSParameter(iteration,    # n_iter
                            1.1,        # T_max = t_max * c̄
                            100,        # T_red
                            1.05,       # α1: control the first local search
                            1.01,        # α2: constrol the second local search
                            Set([random_removal!, worst_removal!, related_removal!, route_removal!]),
                            Set([repair_TSpair, repair_greedy, repair_random_order, repair_regret]),
                            true,       # early stop
                            200         # if no better solution after 200 iterations
                            )
const DEGREE_TS_EXPLORATION = 3 # To the 3rd closest ts pair; dependes on the density of TS network
const DEGREE_REGRET         = 3
const DEGREE_DESTROY        = 0.3
# @warn "Degree of ts exploration is $DEGREE_TS_EXPLORATION for $NETWORK network"


# Start the main loop
function main(num_cus::Int64, NETWORK::String, vns_params::VNSParameter)
    instance = "l2-c$num_cus-d2-bt2"
    max_recharge = 3 # the number of recharge operations
    @error("Iteration number is $iteration")
    # folder = "$NETWORK/cgr_at_depot/$instance" # for cross TS network
    # folder = "$NETWORK/$instance" # for cross TS network
    folder = "$NETWORK/cgr_at_depot/$instance" # for cross TS network
    data_folder = "data/" * folder
    @info "INSTANCE: $folder"
    result_folder = "results/metaheuristics/" * folder
    # result_folder = "results/metaheuristics/" * folder * "/$iteration"
    if !isdir(result_folder) mkdir(result_folder) end
    eidarp, ts_network, ts_network_noshift = preprocess(data_folder, NETWORK, set_dummy_recharge = true, max_recharge = max_recharge);
    # global T_END     = eidarp.t_end
    non_zeros_tt = filter(x -> x!=0, eidarp.traveltime);
    c̄ = mean(non_zeros_tt); # c̄ for T_max = t_max * c̄
    @warn "One transit node can only be visited by one bus"
    @warn "TS repair can only accomadate Degree of destroy to 2"
    @info "The initial battery level is $(initE*100)%"
    # visualize(eidarp) # visualize the input instance

    max_remove_cus = Int(ceil(DEGREE_DESTROY * num_cus))
    # degree_destroy = generate_degree_destroy_iter(iteration, collect(1:DEGREE_DESTROY))
    # degree_destroy = degree_destroy[1:iteration]
    # degree_destroy =  rand(collect(1:DEGREE_DESTROY), iteration)
    # degree_destroy =  DEGREE_DESTROY * ones(Int64, iteration)
    degree_destroy = generate_degree_destroy_equal_distribution(max_remove_cus, 1, iteration, result_folder, fig = true)

    # Algorithm starts here
    t_start = time()
    s₀ = initial_solution(eidarp);
    c₀ = objective_value(s₀)
    t_init = time()
    println("Initial solution is: $c₀ with $(t_init - t_start) s")
    println("Solved customers: $(sum(s₀.servedcus))")
    s, cost, results, iter, t_solve, count_neb_change, res_localserach = VNS(s₀, vns_params, t_start, degree_destroy, c̄, is_rejection = false);
    save_results(instance, s, eidarp, ts_network, ts_network_noshift, iter-1, t_solve, count_neb_change, result_folder, visual = 1)
    save_result_local_search(res_localserach, result_folder)
    fig = plot(results, layout = (3,1), label = ["s_best" "neighborhood" "local_optimal"])
    plot!(fig[2], results[:,1], label = "s_best")
    plot!(fig[3], results[:,2], label = "neighborhood")
    savefig("$result_folder/converge_plot.png")
    results_iter = DataFrame(results, [:c_best, :c′, :c″])
    CSV.write("$result_folder/iter_obj_val.csv", results_iter)
    return 
end

main(100, "cross", vns_params)
