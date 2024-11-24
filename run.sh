#!/bin/bash -l
#SBATCH -c 16
#SBATCH --time=0-10:00:00
#SBATCH -p batch

julia scr/example.jl 10