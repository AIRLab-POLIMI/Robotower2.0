#!/usr/bin/gnuplot

# Legend
set key at 50,112
# Theoretical curve
plot '4.txt' using 1:($2*1000):($4*1000) title 'Power' w yerrorbars ls 2, P(x) title 'Theory' w lines ls 1
