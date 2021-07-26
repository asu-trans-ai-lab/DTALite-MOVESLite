set title "Dynamic Speed Contour (Path 1 New Path)" 
set xlabel "Time"
set ylabel "Space" offset -3
set xtics (" 7:00" 0 ," 7:30" 30 ," 8:00" 60 ," 8:30" 90 ," 9:00" 120 ," 9:30" 150 ,"10:00" 180 ,"10:30" 210 ,"11:00" 240 ) 
set ytics ("3" 0, "4" 10, "5" 20, "6" 30, "7" 40, "8" 50, "9" 60, "10" 70, "11" 80)
set xrange [0:241] 
set yrange [0:80] 
set palette defined (0 "white", 0.1 "red", 40 "yellow", 50 "green")
set pm3d map
splot 'C:\NEXTA_OpenSource\Software_release\sample_data_sets\3. Dynamic Network Loading_3-Corridor_Network\export_path_speed.txt' matrix notitle
