struct Bus
    type::Int64
    capacity::Int64
    speed::Float64
    β::Float64 # energy consumption rate kWh/km
    maxbattery::Float64
    depot::Int64   # depot of the bus
end

struct EIDARP
    nnode::Int64                        # number of nodes in the network; while nnode is destination depot and nnode-1 is the origin depot
    nodes::Vector{Node}                 # set of nodes
    arcs_bus::Vector{Tuple}             # accessible bus arcs
    arcs_ts::Vector{Tuple}              # accessible transit arcs
    arcs_walk::Vector{Tuple}            # accessible customer walking arcs; (customer_origin/destination, Transit station)
    buses::Vector{Bus}                  # set of buses
    n_k::Int64                          # number of vehicles
    n_c::Int64                          # number of customers
    n_ts::Int64                         # number of transit stations
    n_tsline::Int64                     # number of transit lines
    n_cgr::Int64                        # number of chargers
    n_depot::Int64                      # number of depots
    ts_rng::UnitRange{Int64}            # node range of transit stops
    cgr_rng::UnitRange{Int64}           # node range of chargers
    t_end::Float64                      # end of operational time
    timewindows::Array{Float64}         # two colmns: early and late time at each node; while for TS it represents arrival time and departing time
    μ::Vector{Float64}                  # service time at each node
    L::Vector{Float64}                  # maximum travel time for each customer
    max_waittime::Float64               # maximum waiting time at transit stops
    ts_map::Array{Int64}                # mapping between physical and transit stop dummies
    cgr_map::Array{Int64}               # mapping betweeb physical and transit stop dummies
    dist::Array{Float64}                # bus distance matrix row: depatures; column: phsical transit station
    traveltime::Array{Float64}          # bus traveltime matrix 
    traveltime_ts::Array{Float64}       # transit network travel time
    walktime::Array{Float64}            # customer walk time: customer_origin/destination -> transit stations
    cusnearTS::Vector{Vector}           # customers' origin and destination that close to transit station
    nodes_bus_inout::Dict{Int, Vector}  # σ⁻_B and σ⁺_B: The first vector is incoming nodes, the second vector are outgoing nodes
    nodes_ts_inout::Dict                # σ⁻_G and σ⁺_G
    nodes_walk_inout::Dict              # σ⁻_R and σ⁺_R
end