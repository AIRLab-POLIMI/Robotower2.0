#!/usr/bin/gnuplot
reset

set xlabel "time"
#set xrange [0:70]
#set yrange [0:0.1]

#set term png
#set output "time_stats_graph.png"

set datafile separator ","

set key autotitle columnhead
set key reverse Left
set grid
set style data linespoints
set title "External USB camera -- Latency experiment (PCs Unsync, Airlab2 network)"
set xlabel "Frames samples"
set ylabel "Seconds"

plot input_file using 1 title "Point difference", \
	 "" using 2 title "Avg difference (10 samples)", \
	 "" using 3 title "Std. deviation (10 samples)"

pause 0.2
reread
