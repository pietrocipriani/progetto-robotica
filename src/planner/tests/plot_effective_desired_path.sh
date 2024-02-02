#!/bin/bash


export csv="$1"

cat ./plot_effective_desired_path.plg - | gnuplot
