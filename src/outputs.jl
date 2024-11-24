"""
Visualize EIDARP instance. 

# Arguments
- `eidarp::EIDARP`: The instance.
"""
function visualize(eidarp::EIDARP, ts_network_noshift; ts_line_width = 3.0, NETWORK = "cross")
    fig = plot(legendfontsize= 8, legend=:true, dpi=500)
    plot!(legend=:bottomright)
    plot!(xlims=(-9,9), ylims=(-9,9))
    
    n_depot = eidarp.n_depot
    n_c = eidarp.n_c
    n_tsline = eidarp.n_tsline
    nnode = eidarp.nnode
    coords = get_coord(eidarp.nodes)
    # plot depots
    scatter!((coords[nnode-n_depot+1:nnode,1], coords[nnode-n_depot+1:nnode,2]), markerstrokecolor=:blue, marker = (:diamond,6,:white), label="Depot")
    # plot charging stations
    scatter!(coords[eidarp.cgr_rng,1], coords[eidarp.cgr_rng,2], marker = (:diamond,5,:skyblue), markerstrokecolor = :white, label="Chargers")
    # plot origins
    scatter!(coords[1:n_c,1], coords[:,2], markershape=:circle, markersize = 3, color = :white, markerstrokecolor=:black, label = "Pickups")
    annotate!([(coords[i,1], coords[i,2],("$i", 10, :top, :black)) for i in 1:n_c])
    # plot destinations
    scatter!(coords[n_c+1:2*n_c,1], coords[n_c+1:2*n_c,2], marker = (:x, 3, :black), label = "Dropoffs")
    annotate!([(coords[i+n_c,1], coords[i+n_c,2],("$i", 10, :top, :black)) for i in 1:n_c])
    # plot transit stations
    scatter!(coords[eidarp.ts_rng,1], coords[eidarp.ts_rng,2], marker = (:star,5,:black), label="Train stop")
    # plot transit lines
    colors = [:darkolivegreen, :navy, :firebrick4]
    for l in 1:n_tsline
        thisline = filter(x->x.lineNo == l, ts_network_noshift.nodes)
        ts_coords = get_coord(thisline)
        ts_coords = unique(ts_coords, dims=1)
        plot!(ts_coords[:,1], ts_coords[:,2], color=colors[l], linewidth= ts_line_width, label=false)
        # plot!(ts_coords[:,1], ts_coords[:,2], color=colors[l], linewidth=3, label="Train line$(l)")
        if NETWORK == "crossring" && l == 3
            plot!([ts_coords[end,1],ts_coords[1,1]], [ts_coords[1,2], ts_coords[end,2]], color=colors[l], linewidth=3, label=false)
        end
    end

    # display(fig)
    return fig
end

"""
Visualize solution
"""
function visualize(sol::Solution, cus_paths::Vector{Vector{Int64}}, eidarp::EIDARP, ts_network::EIDARPInstance.TSNetwork.TransitNetwork, ts_network_noshift, result_folder; save = 1)
    N = sol.N
    # Plot customers
    served_cus = findall(sol.servedcus)
    plot_customer(served_cus, cus_paths, N, eidarp, ts_network, ts_network_noshift, result_folder)

    # Plot buses
    used_bus = findall(sol.usedbus)
    plot_bus(used_bus, sol.routes, N, eidarp,  ts_network_noshift, result_folder)
end

function plot_bus(used_bus::Vector{Int64}, routes::Vector{Route}, N::Vector{Node}, eidarp::EIDARP, ts_network_noshift, result_folder)
    coords = get_coord(N)
    for k in used_bus
        path = get_path(routes[k])
        len = routes[k].len
        # Initialize plot
        fig = visualize(eidarp, ts_network_noshift, ts_line_width = 1.0)
        for i in 1:len-1
            n1 = path[i]; n2 = path[i+1]
            plot!([coords[n1,1],coords[n2,1]],[coords[n1,2],coords[n2,2]], c = :black, label=false)
        end
        savefig(result_folder * "/bus_$k.png")
    end
end

function plot_customer(served_cus::Vector{Int64}, cus_paths::Vector{Vector{Int64}}, N::Vector{Node}, eidarp::EIDARP, ts_network::EIDARPInstance.TSNetwork.TransitNetwork, ts_network_noshift, result_folder)
    coords = get_coord(N)
    for c in served_cus
        path = cus_paths[c]
        # Initialize plot
        fig = visualize(eidarp, ts_network_noshift, ts_line_width = 1.0)
        len = length(path)
        for i in 1:len-1
            n1 = path[i]; n2 = path[i+1]
            if (N[n1].ID == servedts(N[c])) & (N[n2].ID == servedts(N[N[c].dropoffID]))
            # if isa(N[n1], TransitStation) & isa(N[n2], TransitStation)
                # Transit arcs
                ts_path = ts_network.paths[(n1,n2)]
                len_ts = length(ts_path)
                if len_ts > 2
                    for j in 1:len_ts-1
                        ts1 = ts_path[j]; ts2 = ts_path[j+1]
                        plot!([coords[ts1,1], coords[ts2,1]], [coords[ts1,2], coords[ts2,2]], color = :blue, linewidth = 2, label=false)
                    end
                else
                    plot!([coords[n1,1], coords[n2,1]], [coords[n1,2], coords[n2,2]], color = :blue, linewidth = 2, label=false)
                end 
            elseif isa(N[n1], Pickup) & isa(N[n2], TransitStation)
                if N[n1].walk_ts > 0 
                    # Walk arc at firstmile
                    plot!([coords[n1,1], coords[n2,1]], [coords[n1,2], coords[n2,2]], color = :blue, linestyle = :dashdot, label=false)
                else
                    # Bus arc at firstmile
                    plot!([coords[n1,1], coords[n2,1]], [coords[n1,2], coords[n2,2]], color = :blue, label=false)
                end
            elseif isa(N[n1], TransitStation) & isa(N[n2], Droppoff)
                if N[n2].walk_ts > 0
                    # Walk arc at lastmile
                    plot!([coords[n1,1], coords[n2,1]], [coords[n1,2], coords[n2,2]], color = :blue, linestyle = :dashdot, label=false)
                else
                    # Bus arc at lastmile
                    plot!([coords[n1,1], coords[n2,1]], [coords[n1,2], coords[n2,2]], color = :blue, label=false)
                end
            else
                # Bus arc
                plot!([coords[n1,1], coords[n2,1]], [coords[n1,2], coords[n2,2]], color = :blue, label=false)
            end
        end
        savefig(result_folder * "/c$c.png")
    end
end

function save_results(instance, sol::Solution, eidarp::EIDARP, ts_network::EIDARPInstance.TSNetwork.TransitNetwork, ts_network_noshift, iter_num::Int64, t_solve::Float64, n_nbr_change::Int64, result_folder::String; visual = 0)
    # Bus path result
    used_bus_ID = findall(sol.usedbus)
    total_chg_time = 0.0
    for k in used_bus_ID
        # Final check the feasibility of each path
        if !isfeasible(sol.routes[k], sol)
            @error "Bus $k might not (be) feasible at the last check"
        end
        bus = DataFrame()
        route = sol.routes[k]
        bus.path = get_path(route)
        bus.Load = route.y[1:route.len] 
        bus.BoS = route.D[1:route.len]
        bus.A = route.A[1:route.len]
        bus.EaryTW = [get_earlytw(sol.N[n], k) for n in bus.path]
        bus.LateTW = [get_latetw(sol.N[n], k) for n in bus.path]
        amount, _, soc = energy_check(k, sol)
        # if amount > 1e-3
        #     @error("Bus $k needs to be recharged")
        # end
        bus.SOC = soc[1:route.len]
        CHG_time = zeros(route.len)
        for i in 2:route.len
            n = bus.path[i]
            node = sol.N[n]
            if isa(node, Charger)
                evt = node.cgr_evt[(k, bus.path[i-1])]
                CHG_time[i] = evt.finish - evt.start
            end
        end
        bus.CHG_time = CHG_time
        total_chg_time += sum(bus.CHG_time)
        type = Vector{String}(undef, route.len)
        phyID = Vector{Int64}(undef, route.len)
        # get the physical ID of each node
        for (i,n) in enumerate(bus.path)
            if isa(sol.N[n], Pickup)
                type[i] = string(typeof(sol.N[n]))
                phyID[i] = sol.N[n].ID
            elseif isa(sol.N[n], Droppoff)
                type[i] = string(typeof(sol.N[n]))
                phyID[i] = sol.N[n].pickupID
            elseif isa(sol.N[n], TransitStation)
                type[i] = string(typeof(sol.N[n]))
                phyID[i] = sol.N[n].phyID
            elseif isa(sol.N[n], Charger)
                charger = sol.N[n].phyID
                # _, charger = findfirst(x->x==n,sol.params.cgr)
                type[i] = "Charger$charger"
                phyID[i] = charger
            elseif isa(sol.N[n], Depot)
                type[i] = string(typeof(sol.N[n]))
                phyID[i] = sol.N[n].phyID
            end
        end
        bus.type = type
        bus.phyID = phyID
        CSV.write(result_folder * "/bus$k.csv", bus)
    end

    bus_cost, cus_cost, penalty_cost, cus_paths  = objective_value(sol, output = true)

    # visualize the paths 
    if visual == 1 visualize(sol, cus_paths, eidarp, ts_network, ts_network_noshift, result_folder) end

    # Customer results
    served_cus_ID = findall(sol.servedcus)
    # served_cus_ID = sol.servedcus_set
    Customers = DataFrame(ID = Int64[], Path = Vector[], WalkTime = Float64[], InTrainTime = Float64[],
                          WaitTime = Float64[], TripTime = [], noWaitTrip = Float64[], MaxTripTime = Float64[])
    for p in served_cus_ID
        wt, bt, in_train, triptime_nowait = cus_cost[p,:]
        triptime = customer_traveltime(p, sol)
        waitime = triptime - triptime_nowait
        push!(Customers, [p, cus_paths[p], wt, in_train, waitime, triptime, triptime_nowait, sol.N[p].L])
    end
    CSV.write(result_folder * "/customers.csv", Customers)

    
    # Summary of overall results
    summary = DataFrame(Instance = instance,
                        ObjValue = sol.objective_cost,
                        No_reject_customer = sum(.!sol.servedcus),
                        # No_reject_customer = sol.params.num_cus - length(sol.servedcus_set),
                        BusTravelTime = sum(bus_cost),
                        CusTravelTime = sum(cus_cost[:,4]),
                        TrainTravelTime = sum(cus_cost[:,3]),
                        No_Bus = sum(sol.usedbus),
                        Bus_ID = [findall(sol.usedbus)],
                        ChargeTime = total_chg_time,
                        SolveTime = t_solve,
                        neighbor_change = n_nbr_change,
                        No_iter = iter_num)
    CSV.write(result_folder * "/summary.csv", summary)
    return summary
end

function save_result_local_search(result_ls::Matrix{Float64}, result_folder::String)
    columns = [:NO_LS1, :lower_obj_LS1, :NO_LS2, :lower_obj_LS2]
    df = DataFrame(result_ls, columns)
    CSV.write(result_folder * "/LS_summary.csv", df)
end