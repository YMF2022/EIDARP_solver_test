struct ChargeInfo
    cgr::Int64              # physical ID of charger 
    dist::Float64           # additional dist if insert this charger
    max_amount::Float64     # maximum charging amount 
    max_time::Float64       # maximum charging time 
    start_time::Float64     # earliest recharge begin time
end

struct potentialRecharge
    pre::Int64
    suc::Int64
    soc::Float64
    min_amount::Float64     # minimum amount required to be above min SOC
    cgr_info_vec::Vector{ChargeInfo}
    # cgr_dist_amount_start::Vector{NamedTuple}
end

function energy_check(bus::Int64, sol::Solution)
    route = sol.routes[bus]
    pathID = get_path(route)
    len= route.len
    # soc = route.soc
    soc = route.battery * initE .* ones(Float64, len)
    min_SOC = route.battery * minElevel # min SOC
    amount = 0.0
    # Caculate energy
    for i in 2:len
        node_id = pathID[i]
        node = sol.N[node_id]
        if isa(node, Charger) 
            if haskey(node.cgr_evt, (bus, pathID[i-1]))
                cgr_evt = node.cgr_evt[(bus, pathID[i-1])]
            else
                global s_ec = copy(sol)
                println("Bus $(route.ID) with path: $(pathID[1:len])")
                error("Node $node_id does not have key $((bus, pathID[i-1]))")
            end
            soc[i] = soc[i-1] - sol.params.dist[pathID[i-1], pathID[i]] * route.β + cgr_evt.energy
        else
            soc[i] = soc[i-1] - sol.params.dist[pathID[i-1], pathID[i]] * route.β
            if soc[i] < min_SOC
                if i == len
                    amount_at_node = min_SOC - soc[i]
                    if amount_at_node > amount
                        amount = amount_at_node
                    end
                end
            end
        end
    end
    return amount, min_SOC, soc
end

function identify_charging_positions(min_SOC::Float64, bus::Int64, soc::Vector{Float64}, sol::Solution)
    route = sol.routes[bus]
    tt, dist = sol.params.tt, sol.params.dist
    cgr_dummies = sol.params.cgr
    len, pathID, F, y = route.len, route.pathID, route.F, route.y
    flag_pre_check = false # check if there is a recharge before the first point below min_SoC 
    idx_below = findfirst(x -> x < min_SOC, soc[1:len])  # Identify the first node that is below the minimum level
    min_amount_first = min_SOC - soc[idx_below]
    len_potential_recharge = 0
    pos_no_cus = Vector{potentialRecharge}(undef, 10)
    for i in 1:len-1
        node_id = pathID[i]
        suc_node_id = pathID[i+1]
        cgr_info_vec = []
        if y[i] == 0
            if (route.repair == false) 
                F[i] = cal_Fi(i, route, sol)
            end
            for cgr_id in 1:sol.params.num_charger
                cgr_dm = cgr_dummies[1,cgr_id]
                dist_to_cgr = dist[node_id, cgr_dm] + dist[cgr_dm, suc_node_id]
                dist_to_cgr -= dist[node_id, suc_node_id]
                tt_to_cgr = tt[node_id, cgr_dm] + tt[cgr_dm, suc_node_id]
                route.W[i] = route.B[i] - route.A[i]
                access_time = sol.N[cgr_dm].μ
                additional_time = tt[node_id, suc_node_id] - access_time - tt_to_cgr
                max_recharge_time = F[i] + route.W[i] + additional_time # the max recharge time
                # @info i, route.B[i], route.A[i], F[i], tt_to_cgr, max_recharge_time
                if max_recharge_time < 0 
                    # println("charger $cgr_id is not feasible between $node_id and $suc_node_id")
                    continue 
                end
                max_amount = max_recharge_time * sol.N[cgr_dm].α
                start_time = route.A[i] + sol.params.tt[node_id, cgr_dm] + access_time
                cgr_info = ChargeInfo(
                    cgr_id,
                    dist_to_cgr,
                    max_amount,
                    max_recharge_time,
                    start_time
                )
                if !flag_pre_check
                    if (i < idx_below) && (max_amount > min_amount_first)
                        flag_pre_check = true
                    else
                        return 0, []
                    end
                end
                push!(cgr_info_vec, cgr_info)
            end
            if isempty(cgr_info_vec) continue end
            cgr_info_vec = sort(cgr_info_vec, by = x -> x.max_amount, rev = true)
            # println("Chargers available ($node_id and $suc_node_id) $cgr_info_vec") 
            len_potential_recharge += 1
            min_amount = min_SOC - soc[i]
            pos_no_cus[len_potential_recharge] = potentialRecharge(node_id, pathID[i+1], soc[i], min_amount, cgr_info_vec)
        end
    end
    if len_potential_recharge == 0
        return 0, []
    else
        return len_potential_recharge, pos_no_cus[1:len_potential_recharge]
    end
end

function recharge!(bus::Int64, sol::Solution)
    route = sol.routes[bus]
    N = sol.N
    # Compute the amount needed go to different charging station
    dist, cgr_dummies = sol.params.dist, sol.params.cgr
    bus = route.ID
    amount_init, min_SOC, bus_soc = energy_check(bus, sol)
    # println("Energy check for bus $bus is $amount_init")
    if amount_init <= 1e-3 
        return true
    end

    # 1-> find the y[i] == 0 positions
    len_potential_recharge, pos_no_cus = identify_charging_positions(min_SOC, bus, bus_soc, sol)
    if len_potential_recharge == 0
        return false
    end

    # 2 -> try out recharge
    # recharge_amount = Vector{Float64}(undef, max_num_recharge)
    RechargeEvent = NamedTuple{(:pre, :suc, :cgr_dm, :start_time, :cgr_amount), Tuple{Int, Int, Int, Float64, Float64}}
    recharge_events = Vector{RechargeEvent}(undef, len_potential_recharge)
    final_recharge_index = zeros(Bool, len_potential_recharge) # Indicate if this position is used for recharge
    final_recharge_amount = zeros(Float64, len_potential_recharge) # Indicate the final recharge amount at this position
    # println("Maximum number of recharge: $max_num_recharge")
    # for recharge_id in 1:max_num_recharge
    amount = amount_init  
    for recharge_id in 1:len_potential_recharge      
        # Step2: loop over the available chargers
        flag_insert_here = false # indicate whether we have to insert recharge here
        for cgr_info in pos_no_cus[recharge_id].cgr_info_vec
            cgr = cgr_info.cgr
            recharge_intervals = recharge_availabiliity(cgr_info, sol) # available recharge time
            # println("Maximum recharging time $cgr_info")
            # println("recharge_intervals at charger $cgr: $recharge_intervals")
            if isempty(recharge_intervals) continue end
            addtional_consumption = cgr_info.dist * route.β
            pre = pos_no_cus[recharge_id].pre
            suc = pos_no_cus[recharge_id].suc
            max_amount = cgr_info.max_amount # max available amount without capacitated charging station constraints
            if (recharge_id < len_potential_recharge) && (amount + addtional_consumption > max_amount) 
                if pos_no_cus[recharge_id + 1].soc + sum(final_recharge_amount) < min_SOC
                    # we have to recharge at this point 
                    flag_insert_here = true
                end
                # divide the recharge at different positions
                soc_tmp = pos_no_cus[recharge_id + 1].soc + max_amount - addtional_consumption # the minimum amount needed until next recharge
                if soc_tmp < min_SOC
                    # println("Max amount at $cgr is: $max_amount, while we need $amount")
                    return false
                else
                    cgr_dm_idx = findfirst(j -> all(evt.cgr_dm != j for evt in recharge_events), cgr_dummies[:, cgr])
                    if cgr_dm_idx === nothing
                        # global sol_lmt = copy(sol)
                        # global rcgr_evt = copy(recharge_events)
                        # @error("The number of visits to charger $cgr for bus $bus reach the limit")
                        return false
                    end
                    cgr_dm = cgr_dummies[cgr_dm_idx, cgr]
                    for (idx, interval) in enumerate(recharge_intervals)
                        interval = recharge_intervals[idx]
                        cgr_amount = interval[3] * N[cgr_dm].α
                        dist_to_next_cgr = maximum(dist[pos_no_cus[recharge_id + 1].pre, cgr_dummies[1,1:sol.params.num_charger]]) * route.β
                        if pos_no_cus[recharge_id + 1].soc + cgr_amount - addtional_consumption - dist_to_next_cgr < min_SOC
                            # println("Interval $(interval[3]) with amount $cgr_amount at $cgr is not enough")
                            break
                        end
                        # println("Recharge division: The $recharge_id th recharge at $cgr_dm with $max_amount between $pre and $suc")
                        final_recharge_index[recharge_id] = true
                        recharge_events[recharge_id] = (
                            pre = pre,
                            suc = suc,
                            cgr_dm = cgr_dm,
                            start_time = interval[1],
                            cgr_amount = cgr_amount 
                        )
                        # udpate the minimum amount
                        final_recharge_amount[recharge_id] = cgr_amount
                        amount = amount + addtional_consumption - cgr_amount 
                        if amount < 0
                            error("Amount becomes negative at bus $bus")
                        end
                        break
                    end
                end
            # elseif (recharge_id < len_potential_recharge) && (amount <= max_amount)
            #     println("No recharge apply")
            # elseif (recharge_id == len_potential_recharge) && (amount <= max_amount) # the last recharge point
            elseif (recharge_id <= len_potential_recharge) && (amount <= max_amount) # the last recharge point
                # record the recharge id and amount
                cgr_dm_idx = findfirst(j -> all(evt.cgr_dm != j for evt in recharge_events), cgr_dummies[:, cgr])
                if cgr_dm_idx === nothing
                    # global sol_lmt = copy(sol)
                    # global rcgr_evt = copy(recharge_events)
                    # @error("The number of visits to charger $cgr for bus $bus reach the limit")
                    return false
                end
                cgr_dm = cgr_dummies[cgr_dm_idx, cgr]
                for (idx, interval) in enumerate(recharge_intervals)
                    # println("Max amount: $max_amount and amount needed $amount at charger $cgr with dummy $cgr_dm")
                    interval = recharge_intervals[idx]
                    cgr_amount = interval[3] * N[cgr_dm].α
                    if cgr_amount < amount + addtional_consumption
                        return false
                    end
                    final_recharge_index[recharge_id] = true
                    recharge_events[recharge_id] = (
                        pre = pre,
                        suc = suc,
                        cgr_dm = cgr_dm,
                        start_time = interval[1],
                        cgr_amount = amount + addtional_consumption
                    )
                    amount = amount + addtional_consumption - cgr_amount
                    if amount < 1e-3
                        @goto insert_recharge
                    elseif (amount >= 1e-3) && (recharge_id == len_potential_recharge)
                        return false
                    elseif (amount >= 1e-3) && (recharge_id < len_potential_recharge)
                        break
                    end
                end
            else
                # println("Still $amount to be charged at $recharge_id with $max_num_recharge times")
                return false
            end
        end
      if (flag_insert_here == true)  && (final_recharge_index[recharge_id] == false)
        return false
      end
    end

    # Step 3: Insert recharge into route
    @label insert_recharge
    if amount > 1e-3
        return false
    end
    # Check if there exists a node in the event
    # for n in cgr_dummies
    #     for (key, evt) in N[n].cgr_evt
    #         if key[1] == bus 
    #             error("Bus $bus exisit in node $n with event: $evt")
    #         end
    #     end
    # end
    insert_recharge!(len_potential_recharge, recharge_events, final_recharge_index, bus, sol)
    # order_route!(sol.routes[bus], sol.N)
    # last_amount,_ = energy_check(bus, sol)
    # if last_amount > 1e-3
    #     global s_ec = copy(sol)
    #     error("energy check not pass at bus $bus")
    # end
    return true
end

function insert_recharge!(len_potential_recharge::Int64, recharge_events::Vector, final_recharge_index::Vector{Bool}, bus::Int64, sol::Solution)
    N = sol.N
    for id in 1:len_potential_recharge
        if final_recharge_index[id]
            recharge_info = recharge_events[id]
            cgr_dm = recharge_info.cgr_dm
            cgr_amount = recharge_info.cgr_amount
            cgr_time = cgr_amount/N[cgr_dm].α
            pre = recharge_info.pre
            suc = recharge_info.suc
            finish_time = recharge_info.start_time + cgr_time
            cgr_evt = ChargingEvent(
                bus,
                cgr_dm,
                recharge_info.start_time,
                finish_time,
                cgr_amount,
                pre,
                suc
            )
            N[cgr_dm].cgr_evt[(bus, recharge_info.pre)] = cgr_evt
            # println("Bus $bus: Insert $cgr_dm between $pre and $suc with charging time $cgr_time from $(recharge_info.start_time) to $finish_time")
            insertnode!(N[cgr_dm], pre, suc, bus, sol)
        end
    end
    sol.routes[bus].removeE = false # charging node is reinserted
end

# Remove all chargers in the route
function remove_charger!(bus::Int64, sol::Solution)
    route = sol.routes[bus]
    order_route!(route, sol.N)
    pathID = route.pathID[1:route.len]
    cgr_dummies = sol.params.cgr
    suc, pre = route.suc, route.pre
    # cgr_nodes = intersect(pathID, cgr_dummies)
    # if isempty(cgr_nodes) 
    #     return 
    # end
    if route.removeE == true
        return 
    end
    # Remove starts here
    for n in cgr_dummies
        # remove the charging event at the node 
        # for bus in keys(sol.N[n].cgr_evt)
            # if bus[1] == bus 
                filter!(((key,_),) -> key[1] != bus, sol.N[n].cgr_evt)
                # pop!(sol.N[n].cgr_evt, bus)
        #     end
        # end
        
        # remove the charger at the route
        if suc[n] != 0
            pre_n = pre[n]
            suc_n = suc[n]
            suc[pre_n] = suc_n
            pre[suc_n] = pre_n
            suc[n] = 0
            pre[n] = 0
        end
    end
    route.removeE = true
end

function recharge_conflict_check_final(sol::Solution)
    cgr_nodes = sol.params.cgr
    N = sol.N

    for charger in 1:sol.params.num_charger
        recharge_events = []
        for cgr_dm in cgr_nodes[:,charger]
            cgr_node = N[cgr_dm]
            for (key, evt) in cgr_node.cgr_evt
                start = evt.start
                finish = evt.finish
                cgr_time = finish - start
                # println("Charge $charger ($cgr_dm) for bus $(key[1]): $start to $finish with $cgr_time minutes")
                push!(recharge_events, (start, finish, key[1], cgr_dm))
            end
        end
        recharge_events = sort(recharge_events, by = x -> x[1])
        for i in 1:length(recharge_events) - 1
            end_time = recharge_events[i][2]
            next_start_time = recharge_events[i+1][1]
            if end_time > next_start_time
                bus1 = recharge_events[i][3]
                n1 = recharge_events[i][4]
                bus2 = recharge_events[i+1][3]
                n2 = recharge_events[i+1][4]
                @error("Overlap at $charger: bus$bus1 at node$n1 and bus$bus2 and node $n2")
            end
        end
    end
end

function recharge_availabiliity(cgr_info::ChargeInfo, sol::Solution)
    charger = cgr_info.cgr
    cgr_nodes = sol.params.cgr[:, charger]
    N = sol.N
    earliest = cgr_info.start_time
    latest = cgr_info.max_time + earliest
    time_intervals = [(evt.start, evt.finish) for cgr in cgr_nodes for (key, evt) in N[cgr].cgr_evt]
    time_intervals = sort(time_intervals, by = x -> x[1])
    availabilities = [] # the availabilities of time interval
    current_start = earliest # track the start of availability 

    for (start, finish) in time_intervals
        if start > latest
            break
        end
        # Check if there's a gap between `current_start` and the start of this interval
        if start > current_start
            # Add the available interval from `current_start` to `start`
            push!(availabilities, (current_start, start, start - current_start))
        end 
        current_start = max(current_start, finish)
    end
    # Add the last available interval if it extends to the end of `tw`
    if current_start < latest
        push!(availabilities, (current_start, latest, latest - current_start))
    end
    availabilities = sort(availabilities, by = x -> -x[3])
    if !isempty(availabilities) && (availabilities[1][3] > cgr_info.max_time + 1e-5)
        global s_avb = copy(sol)
        # global cgr_info1 = copy(cgr_info)
        error("Available interval $(availabilities) greater than $cgr_info")
    end
    return availabilities
end