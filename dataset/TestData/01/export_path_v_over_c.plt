set title "Dynamic Volume Over Capcity Contour (Path 1 New Path)" 
set xlabel "Time"
set ylabel "Space" offset -3
set xtics (" 7:00" 0 ," 7:10" 10 ," 7:20" 20 ," 7:30" 30 ," 7:40" 40 ," 7:50" 50 ," 8:00" 60 ," 8:10" 70 ," 8:20" 80 ," 8:30" 90 ," 8:40" 100 ," 8:50" 110 ," 9:00" 120 ) 
set ytics ("1" 0, "3" 10, "4" 20, "5" 30, "6" 40, "7" 50, "8" 60, "9" 70, "10" 80, "11" 90, "12" 100, "2" 110)
set xrange [0:121] 
set yrange [0:110] 
set palette defined (0 "white", 0.4 "green", 0.6 "yellow", 1 "red")
set pm3d map
splot 'E:\00-discovery\Module 5_3-Corridor_Network\export_path_v_over_c.txt' matrix notitle
