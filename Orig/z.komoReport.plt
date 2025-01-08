set key autotitle columnheader
set title 'komo report'
plot 'z.komoData' \
      u (($0+1)/5):1 w l lw 3 lc 1 lt 1 \

