set title "Dynamic Density Contour (Path 1 New Path)" 
set xlabel "Time"
set ylabel "Space"  offset -3
set xtics (" 7:00" 0 ," 7:30" 30 ," 8:00" 60 ," 8:30" 90 ," 9:00" 120 ," 9:30" 150 ,"10:00" 180 ,"10:30" 210 ,"11:00" 240 ) 
set ytics ("1" 0, "3" 10, "4" 20, "5" 30, "6" 40, "7" 50, "8" 60, "9" 70, "10" 80, "11" 90)
set xrange [0:241] 
set yrange [0:90] 
set palette defined (0 "white", 10 "green", 30 "yellow", 50 "red")
set pm3d map
splot 'C:\Temp\Module 5_3-Corridor_Network-1031\Module 5_3-Corridor_Network\export_path_density.txt' matrix notitle
