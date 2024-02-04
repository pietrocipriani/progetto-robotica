#!/bin/bash


export csv="$1"

cat ./plot_desired_path.plg - | gnuplot
