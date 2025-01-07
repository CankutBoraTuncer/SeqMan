set key autotitle columnheader
set title 'komo report'
plot 'z.komoData' \
      u (($0+1)/1):1 w l lw 3 lc 1 lt 1 \
  ,'' u (($0+1)/1):2 w l lw 3 lc 2 lt 1 \
  ,'' u (($0+1)/1):3 w l lw 3 lc 3 lt 1 \
  ,'' u (($0+1)/1):4 w l lw 3 lc 4 lt 1 \
  ,'' u (($0+1)/1):5 w l lw 3 lc 5 lt 1 \
  ,'' u (($0+1)/1):6 w l lw 3 lc 6 lt 1 \

