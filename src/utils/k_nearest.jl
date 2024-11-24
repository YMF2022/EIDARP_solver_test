"""
 Find the k-nearest transit station to customer whose first and last mile by bus
"""
function k_nearest_ts(cus_o::Pickup, cus_d::Droppoff, tt::Matrix{Float64}, A_ts::Dict{Tuple, Float64}; k::Int64 = 1)
    arcs_ts = keys(A_ts)
    pairs = []
    ridetime_list = []
    ts_board = vcat(cus_o.ts_order[1:k]...)
    ts_alight = vcat(cus_d.ts_order[1:k]...)
    for ts_o in ts_board
        for ts_d in ts_alight
            if (ts_o, ts_d) ∈ arcs_ts
                # Check if the ride time constraints satisfy
                ridetime = tt[cus_o.ID, ts_o] + A_ts[(ts_o, ts_d)] + tt[ts_d, cus_d.ID]
                if ridetime <= cus_o.L 
                    push!(pairs, (ts_o, ts_d))
                    push!(ridetime_list, ridetime)
                end
            end
        end
    end
    odr = sortperm(ridetime_list)
    ridetime_list = ridetime_list[odr]
    pairs = pairs[odr]
    return pairs
end

# Given the walkting ts stations at last mile, find the k-nearest ts at the first mile
function k_nearest_ts(cus_o::Pickup, cus_d::Droppoff, bus_firstorlast::Symbol; k::Int64 = 1)
    @assert bus_firstorlast == :first ||  bus_firstorlast == :last "firstorlast must be either :first or :last"
    pairs = []
    # ts_pairs = vcat(cus_o.ts_pair[1:k]...)
    ts_pairs = cus_o.ts_pair
    if bus_firstorlast == :first
        # customers' last mile is walking
        ts_alight = vcat(cus_d.close_ts)
        for ts_d in ts_alight
            p = filter(x->x[2] == ts_d, ts_pairs)
            pairs = [pairs; p]
        end
    else bus_firstorlast == :last
        # customers' first mile is walking
        ts_board = vcat(cus_o.close_ts)
        for ts_o in ts_board
            p = filter(x->x[1] == ts_o, ts_pairs)
            pairs = [pairs; p]
        end
    end
    return pairs
end