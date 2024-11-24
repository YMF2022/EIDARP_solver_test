module EIDARPInstance

using CSV, DataFrames, Distances
import ..Node, ..Pickup, ..Droppoff, ..TransitStation, ..Charger, ..Depot
import ..get_coord

include("../tsnetwork/TSNetwork.jl")
include("instance_structure.jl")
include("preprocess.jl")
using .TSNetwork

export preprocess, EIDARP

end

