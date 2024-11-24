include("addbus.jl")
include("insert.jl")
include("greedysearch.jl")
include("randomsearch.jl")

"""
For first, last mile or direct
"""
function search(sol::Solution, node1::Node, node2::Node; regret::Int64 = 1, isE::Bool = false)
    if isa(node1, Pickup)
        solutions, costs, len_sol = randomsearch_firstmile(sol, node1, node2)
    else
        solutions, costs, len_sol = randomsearch_lastmile(sol, node1, node2)
    end

    if len_sol < regret
        # @error("The regret value is more than the feasibile solution")
        return solutions, costs, len_sol
    end
    return solutions[1:regret], costs[1:regret], regret
end

"""
For both first and last mile
"""
function search(sol::Solution, node1::Pickup, ts_pair::Vector{Tuple{Int64, Int64}}; regret::Int64 = 1, isE::Bool = false)
    solutions, costs, len_sol = randomsearch_bothmile(sol, node1, ts_pair)
    if len_sol < regret
        # @error("The regret value is more than the feasibile solution")
        return solutions, costs, len_sol
    end
    return solutions[1:regret], costs[1:regret], regret
end