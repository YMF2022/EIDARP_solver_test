using Plots
using Random
using JSON
using HDF5
using Distributions
using DelimitedFiles

function generate_timetable(n_ts, start_t, end_t, freq, tt, dt, folder; shift = 5.0)
    # sym = collect('A':'Z')
    n_ts_each = n_ts ÷ 2
    max_duration = 0
    n_dep = Int(2*(end_t - start_t)*60 ÷ freq)
    for l in 1:2
        # train_names = split(String(sym[1+(l-1)*n_ts_each:n_ts_each*l]),"")
        train_names = string.(collect((l-1)*n_ts_each+1:(l-1)*n_ts_each+n_ts_each))
        push!(train_names,"Direction")
        train_names = reshape(train_names,1,:)
        
        timetable = Array{Any}(undef, n_dep, n_ts_each+1)
        last_0 = 20.0 + 3*(l-1) # direction 0: right to left/down
        last_1 = last_0 + shift # direction 1: left to right /up
        for dep in 1:n_dep
            if isinteger(dep/2)
                timetable[dep, n_ts_each+1] = 0
                for ts in n_ts_each:-1:1
                    timetable[dep,ts] = last_1 + abs(ts-3)*(tt+dt)
                end
                last_1 += freq
            else
                timetable[dep, n_ts_each+1] = 1
                for ts in 1:n_ts_each
                    timetable[dep,ts] = last_0 + (ts-1)*(tt+dt)
                end
                last_0 += freq
            end
        end
        open("$folder/timetable_line$l.csv", "w") do f
            writedlm(f, train_names, ",")
            writedlm(f, timetable, ",")
        end
        # println(maximum(timetable))
        if maximum(timetable) > max_duration
            max_duration = maximum(timetable)
        end
    end
    return max_duration, n_dep
end

function nearest(t::Float64, m::Matrix{Float64}, trans::Int64, direct::Int64)
    dep = m[:,trans] # find departure time for transfer stop
    trans_t = dep .- t
    s = sortperm(trans_t)
    for d in s
        if trans_t[d] > 0 && trans_t[d] < 10 && m[d,end]==direct
            return d, trans_t[d]
        end
    end
    return 0, 0 
end

function generate_network(folder::String, n_ts::Int64, dt::Float64)
    m1 = readdlm("$folder/timetable_line1.csv",',')
    m2 = readdlm("$folder/timetable_line2.csv",',')
    m1 = Float64.(m1[2:end,:])
    m2 = Float64.(m2[2:end,:])
    n_ts_each = n_ts ÷ 2 # two lines have the same number of transit stops
    # Combine timetable. Only possible when two lines have the same No. stops&direction&departures
    m = hcat(m1[:,1:n_ts_each], m2) 
    n_dep = size(m)[1]
    ts_nodes = collect(1:n_ts*n_dep)
    ts_nodes_struct = permutedims(reshape(ts_nodes,(n_ts,n_dep)))
    trans_1 = n_ts_each ÷ 2 + 1 # the transfer stop at line 1
    trans_2 = trans_1 + n_ts_each # the transfer stop at line 2
    ts_tt = 1000 .* ones(n_ts, n_ts, n_dep) # travel time 3D
    ts_tt_2D = 1000 .* ones(n_ts*n_dep, n_ts*n_dep) # Travel time 2D
    ts_network_arcs = Dict{Int64,Vector{Tuple}}()
    ts_network_arcs_2D = Dict{Int64,Vector{Tuple}}()
    ts_dep_time = 1000 .* ones(n_ts*n_dep, n_ts*n_dep) # departure time
    ts_arr_time = 1000 .* ones(n_ts*n_dep, n_ts*n_dep) # arrival time
    for d = 1:n_dep
        ts_network_arcs[d] = []
        ts_network_arcs_2D[d] = []
        direct = m[d,end]
        # iseven(d) ? direct = 0 : direct = 1
        for ts1 in 1:n_ts
            ts12d = ts_nodes_struct[d,ts1]
            for ts2 in 1:n_ts
                if ts1 == ts2 || (ts1==trans_1 && ts2==trans_2) # same stops
                    continue
                elseif ts1 == trans_1 && ts2 > n_ts_each # trans_1 to the line 2
                    continue
                elseif ts1 <= n_ts_each && ts2 == trans_2 # line1 to trans_2
                    continue
                elseif ts1 > n_ts_each && ts2 == trans_1 # line2 to trans_1
                    continue
                elseif ts1 == trans_2 && ts2 <= n_ts_each # trans_2 to line 1
                    continue
                end

                if ts1 <= n_ts_each && ts2 <= n_ts_each # ts1, ts2 are in line 1
                    ts22d = ts_nodes_struct[d,ts2]
                    if ts2 > ts1 && direct == 1
                        ts_tt[ts1,ts2,d] = m[d,ts2] - m[d,ts1] - dt # dt is dwel_time
                        ts_tt_2D[ts12d,ts22d] = ts_tt[ts1,ts2,d] # travel time
                        ts_arr_time[ts12d,ts22d] = m[d,ts2] - dt # arrival time
                        ts_dep_time[ts12d,ts22d] = m[d,ts1] # departure time
                        push!(ts_network_arcs_2D[d], (ts12d,ts22d))
                        push!(ts_network_arcs[d], (ts1,ts2))  
                    elseif ts2 < ts1 && direct == 0
                        ts_tt[ts1,ts2,d] = m[d,ts2] - m[d,ts1] - dt # dt is dwel_time
                        ts_tt_2D[ts12d,ts22d] = ts_tt[ts1,ts2,d] # travel time
                        ts_arr_time[ts12d,ts22d] = m[d,ts2] - dt # arrival time
                        ts_dep_time[ts12d,ts22d] = m[d,ts1] # departure time
                        push!(ts_network_arcs_2D[d], (ts12d,ts22d))
                        push!(ts_network_arcs[d], (ts1,ts2))
                    end 
                elseif ts1 <= n_ts_each && ts2 > n_ts_each # ts1 in line1 => ts2 in line 2
                    if m[d,trans_1] < m[d,ts1] # already pass the transfer stop
                        continue
                    end
                    # @info "ts1 and ts2" (ts1,ts2)
                    ts2 - trans_2 > 0 ? direct2 = 1 : direct2 = 0 # get line2 direction
                    t1 = m[d,trans_1] - dt # get the arrival time at trans_1 at departure d
                    # println("direct $direct2 and t1 $t1")
                    d2, transfer_t = nearest(t1, m, trans_2, direct2)  # find the nearest departure time at line2
                    # println(d2)                   
                    if d2 != 0 # a transfer is found
                        ts22d = ts_nodes_struct[d2,ts2]
                        ts_tt[ts1,ts2,d] = m[d2,ts2] - m[d,ts1] - dt # dt is dwel_time
                        ts_tt_2D[ts12d,ts22d] = ts_tt[ts1,ts2,d] # travel time
                        ts_arr_time[ts12d,ts22d] = m[d2,ts2] - dt # arrival time
                        ts_dep_time[ts12d,ts22d] = m[d,ts1] # departure time
                        push!(ts_network_arcs_2D[d], (ts12d,ts22d))
                        push!(ts_network_arcs[d], (ts1,ts2))
                    end
                elseif ts1 > n_ts_each && ts2 <= n_ts_each # ts1 in line2 => ts2 in line1
                    if m[d,trans_2] < m[d,ts1] # already pass the transfer stop
                        continue
                    end
                    # @info "ts1 and ts2" (ts1,ts2)
                    ts2 - trans_1 > 0 ? direct1 = 1 : direct1 = 0 # get line1 direction
                    t2 = m[d,trans_2] - dt # get the arrival time at trans_2 at dparture d
                    d1, transfer_t = nearest(t2, m, trans_1, direct1) # find the nearest departure time at line1
                    if d1 != 0
                        ts22d = ts_nodes_struct[d1,ts2]
                        ts_tt[ts1,ts2,d] = m[d1,ts2] - m[d,ts1] - dt 
                        ts_tt_2D[ts12d,ts22d] = ts_tt[ts1,ts2,d] # travel time
                        ts_arr_time[ts12d,ts22d] = m[d1,ts2] - dt # arrival time
                        ts_dep_time[ts12d,ts22d] = m[d,ts1] # departure time
                        push!(ts_network_arcs_2D[d], (ts12d,ts22d))
                        push!(ts_network_arcs[d], (ts1,ts2))
                    end
                else # ts1 and ts2 are both in line 2
                    ts22d = ts_nodes_struct[d,ts2]
                    if ts2 > ts1 && direct == 1
                        ts_tt[ts1,ts2,d] = m[d,ts2] - m[d,ts1] - dt # dt is dwel_time
                        ts_tt_2D[ts12d,ts22d] = ts_tt[ts1,ts2,d] # travel time
                        ts_arr_time[ts12d,ts22d] = m[d,ts2] - dt # arrival time
                        ts_dep_time[ts12d,ts22d] = m[d,ts1] # departure time
                        push!(ts_network_arcs_2D[d], (ts12d,ts22d))
                        push!(ts_network_arcs[d], (ts1,ts2))
                    elseif ts2 < ts1 && direct == 0
                        ts_tt[ts1,ts2,d] = m[d,ts2] - m[d,ts1] - dt # dt is dwel_time
                        ts_tt_2D[ts12d,ts22d] = ts_tt[ts1,ts2,d] # travel time
                        ts_arr_time[ts12d,ts22d] = m[d,ts2] - dt # arrival time
                        ts_dep_time[ts12d,ts22d] = m[d,ts1] # departure time
                        push!(ts_network_arcs_2D[d], (ts12d,ts22d))
                        push!(ts_network_arcs[d], (ts1,ts2))
                    end
                end
            end
        end
    end
    return ts_tt, ts_network_arcs, ts_tt_2D, ts_arr_time, ts_dep_time, ts_network_arcs_2D
end

function generate_customer(n_c, opr_len, opr_width, v_bus, operation_time, folder ; detour_factor = 1.8)
    cus_array = Array{Float64}(undef, n_c, 8)
    opr_width_half = opr_width/2 - 1
    opr_len_half = opr_len/2 - 1
    x_ori = rand(Uniform(-opr_len_half, opr_len_half), n_c)
    y_ori = rand(Uniform(-opr_width_half, opr_width_half), n_c)
    x_des = rand(Uniform(-opr_len_half, opr_len_half), n_c)
    y_des = rand(Uniform(-opr_width_half, opr_width_half), n_c)
    for c in 1:n_c
        x1, x2, y1, y2 = x_ori[c], x_des[c], y_ori[c], y_des[c]
        while sqrt((x1-x2)^2+(y1-y2)^2) < 2.0
            x1, x2, y1, y2 = rand(Uniform(-opr_len_half, opr_len_half), 4)
        end
        x_ori[c], x_des[c], y_ori[c], y_des[c] = x1, x2, y1, y2
    end
    cus_array[:,1] = x_ori
    cus_array[:,2] = y_ori
    cus_array[:,3] = x_des
    cus_array[:,4] = y_des

    # maximum travel time
    direct_dist = sqrt.((x_ori .- x_des).^2 + (y_ori .- y_des).^2)
    max_duration = 2 .*direct_dist./v_bus # ear_dep + 2 * direct travel time = late_arr
    direct_tt = direct_dist./v_bus
    max_tt = detour_factor.*direct_dist./v_bus
    
    # Time window
    dep_time = Vector{Float64}(undef, n_c)
    arr_time = Vector{Float64}(undef, n_c)
    for c in 1:n_c
        dep = operation_time * rand()
        arr = dep + max_duration[c]
        while arr > operation_time
            dep = operation_time * rand()
            arr = dep + max_duration[c]
        end
        dep_time[c] = dep
        arr_time[c] = arr
    end
    cus_array[:,5] = dep_time
    cus_array[:,6] = dep_time .+ 15
    cus_array[:,7] = direct_tt
    cus_array[:,8] = max_tt

    open("$folder/customers.csv", "w") do f
        writedlm(f, ["x_o" "y_o" "x_d" "y_d" "ear_dep_time" "late_dep_time" "direct_ridetime" "max_ridetime"], ",")
        writedlm(f, cus_array, ",")
    end
    return cus_array
end

function generate_trainstop(n_ts::Int64, dist_ts::Float64, n_line::Int64, folder::String)
    ts_coords = Array{Any}(undef, n_ts, 3)
    n_ts_each = n_ts ÷ 2 # number of transit stops for each line
    trans_ts = n_ts_each ÷ 2 + 1 # the transfer stop
    # Line 1
    ts_coords[1:n_ts_each,1] .= [x for x in -((n_ts_each-1)*dist_ts/2):dist_ts:((n_ts_each-1)*dist_ts/2)]
    ts_coords[1:n_ts_each,2] .= zeros(n_ts_each)
    ts_coords[1:n_ts_each,3] .= 1
    # Line 2
    ts_coords[n_ts_each+1:n_ts,1] = zeros(n_ts_each)
    ts_coords[n_ts_each+1:n_ts,2] = [x for x in -((n_ts_each-1)*dist_ts/2):dist_ts:((n_ts_each-1)*dist_ts/2)]
    ts_coords[n_ts_each+1:n_ts,3] .= 2
    open("$folder/trainStops.csv", "w") do f
        writedlm(f, ["x" "y" "line"], ",")
        writedlm(f, ts_coords, ",")
    end
    return ts_coords
end

function generate_charger(ts_coords, cgr_speed::Float64, folder::String)
    # cgr_info = Array{Float64}(undef, size(ts_coords)[1], 3)
    # cgr_info[:,1:2] = ts_coords[:,1:2]
    # cgr_info[:,3] .= cgr_speed
    # open("$folder/chargers.csv", "w") do f
    #     writedlm(f, ["x" "y" "charging_speed"], ",")
    #     writedlm(f, cgr_info, ",")
    # end
    cgr_info = Array{Float64}(undef, 2, 3)
    cgr_info[:,3] .= cgr_speed
    cgr_info[:,1:2] .= 0.0
    open("$folder/chargers.csv", "w") do f
        writedlm(f, ["x" "y" "charging_speed"], ",")
        writedlm(f, cgr_info, ",")
    end
    return cgr_info[:,1:2]
end

function graph(ts_coords, c_array, n_c, opr_width, folder; flag = 1)
    image = plot(title="Scenario",legendfontsize=7, legend=:false)
    ylims!(-opr_width/2-1, opr_width/2+1)
    xlims!(-opr_width/2-1, opr_width/2+1)
    # plot customers
    scatter!(c_array[:,1],c_array[:,2], label="origin", markershape=:circle, markercolor=:black, markersize=4)
    scatter!(c_array[:,3],c_array[:,4], label="destination", markershape=:utriangle, markercolor=:phase, markersize=5, markerstrokewidth=0)
    for c in 1:n_c
        annotate!(c_array[c,1]+0.2, c_array[c,2]+0.3, text("$c",10,:black))
        annotate!(c_array[c,3]+0.2, c_array[c,4]+0.3, text("$c",10,:phase))
    end
    # plot transit stations and line
    scatter!(ts_coords[:,1], ts_coords[:,2], label="Transit stops", 
            markershape=:star5, markercolor=:salmon, markersize=8, markerstrokewidth=0)
    n_ts_each = size(ts_coords)[1] ÷ 2
    plot!(ts_coords[1:n_ts_each,1], ts_coords[1:n_ts_each,2], color=:salmon, linewidth=4)
    plot!(ts_coords[n_ts_each+1:end,1], ts_coords[n_ts_each+1:end,2], color=:salmon, linewidth=4)
    sym = collect('A':'Z')
    if flag == 1
        for ts in 1:size(ts_coords)[1]
            if ts == n_ts_each + ceil(n_ts_each/2)
                annotate!(ts_coords[ts,1]+0.7, ts_coords[ts,2]-0.4, text("($(sym[ts]))",10,:salmon))
            else
                annotate!(ts_coords[ts,1]+0.3, ts_coords[ts,2]-0.4, text("$(sym[ts])",10,:salmon))
            end 
        end
    else
        for ts in 1:size(ts_coords)[1]
            if ts == n_ts_each + ceil(n_ts_each/2)
                annotate!(ts_coords[ts,1]+0.7, ts_coords[ts,2]-0.4, text("($ts)",10,:salmon))
            else
                annotate!(ts_coords[ts,1]+0.3, ts_coords[ts,2]-0.4, text("$ts",10,:salmon))
            end
        end
    end
    display(image)
    savefig(image, "$folder/fig.png")
end

function save_network(folder::String, ts_tt::Array, ts_arcs::Dict)
    # export network OD Matrix
    h5open("$folder/OD-3D.h5", "w") do file
        write(file, "OD", ts_tt)
    end

    #export OD pairs
    open("$folder/OD-pairs.json", "w") do f
        json_string = JSON.json(ts_arcs)
        JSON.print(f, json_string)
    end
end

function main(n_c::Int64; n_ts = 6, dist_ts = 5.0, maxRadius = 3.0, maxWlkdist = 1.0, n_line = 2,
                start_t = 10.0, end_t = 11.0, freq = 30.0, travel_time = 5.0, dwel_time = 1.0,
                depot = (0.0, 0.0), charger_speed = 0.83, charger_dummies = 2,  μ = 0.5,
                v_bus = 40.0, v_mass = 60.0, v_walk = 0.085, detour_factor = 1.8, max_wait_time=10.0,
                bus_capacity = 10)
    Random.seed!(10) 
    # Create a folder name
    folder_name = "cross/c$n_c"
    if !isdir(folder_name)
        mkdir(folder_name)
    end
    # Decide operational area
    opr_len = 2*maxRadius + dist_ts * (n_ts ÷ 2 -1)
    opr_width = 2*maxRadius + dist_ts * (n_ts ÷ 2 -1)
    println("Operational area: $opr_len*$opr_width km")
    # Convert speed to km/min 
    v_bus = v_bus/60.0
    v_mass = v_mass/60.0
    # Generate transit stops and timetable
    ts_coords = generate_trainstop(n_ts, dist_ts, n_line, folder_name)
    max_duration, n_dep = generate_timetable(n_ts, start_t, end_t, freq, travel_time, dwel_time, folder_name)
    
    # global ts_tt, ts_arcs, ts_tt_2D, ts_arr_time, ts_dep_time, ts_arcs_2D = generate_network(folder_name, n_ts, dwel_time)
    # save_network(folder_name, ts_tt, ts_arcs)
    # Generate customers randomly
    operation_time = (end_t-start_t)*60
    cus_array = generate_customer(n_c, opr_len, opr_width, v_bus, max_duration, folder_name, detour_factor = detour_factor)
    # Generate chargers: coordinates the same as transit stops
    cgr_coords = generate_charger(ts_coords, charger_speed, folder_name)
    n_cgr = size(cgr_coords)[1]
    
    # Bus data
    n_bus = Int(ceil(n_c/(bus_capacity*0.7)))
    BUS_β = 0.226 # kWh/km Volswagen ID.BUZZ https://en.wikipedia.org/wiki/Volkswagen_ID._Buzz # needs to be changed
    BUS_battery = 77
    open("$folder_name/buses.csv", "w") do f
        writedlm(f, ["capacity" "num" "speed" "consumption" "maxBattery"], ',')
        writedlm(f, Any[bus_capacity n_bus v_bus BUS_β BUS_battery], ',')
    end

    # Output other parameter
    open("$folder_name/parameters.csv", "w") do f
        writedlm(f, ["NOcustomer" "NOtransit" "NOcharger" "NOTSline" "NOdummyCgr" "depot_x" "depot_y" "service_time" "max_wlk_dist" "wlk_speed" "v_train" "dwel_time" "detour_factor" "max_wait_time" "start_time" "duration"], ',')
        writedlm(f, Any[n_c n_ts n_cgr n_line charger_dummies depot[1] depot[2] μ maxWlkdist v_walk v_mass dwel_time detour_factor max_wait_time 0.0 max_duration+15], ',')
    end
    # Plot the instance: falg = 1(0), label for transit stops will be letters(numbers)
    graph(ts_coords, cus_array, n_c, opr_width, folder_name, flag = 0)
end

# retrive data
# jdata = JSON.parsefile("cross/c10/OD-pairs.json")
# jd = JSON.parse(jdata)

# for i in 1:9
#     main(i)
# end