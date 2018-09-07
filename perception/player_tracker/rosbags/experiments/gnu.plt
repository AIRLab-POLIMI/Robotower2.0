# set terminal pngcairo  transparent enhanced font "arial,10" fontscale 1.0 size 600, 400 
# set output 'polar.4.png'
set clip points
unset border
set dummy t, y
set raxis
set key fixed right top vertical Right noreverse enhanced autotitle box lt black linewidth 1.000 dashtype solid
set polar
set samples 160, 160
set style data lines
set xzeroaxis
set yzeroaxis
set zzeroaxis
set xtics axis in scale 1,0.5 nomirror norotate  autojustify
set ytics axis in scale 1,0.5 nomirror norotate  autojustify
unset rtics
set trange [ 0.00000 : 6.28319 ] noreverse nowriteback
DEBUG_TERM_HTIC = 119
DEBUG_TERM_VTIC = 119
plot sin(4*t),cos(4*t)

