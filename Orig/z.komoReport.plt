set key autotitle columnheader
set title 'komo report'
plot 'z.komoData' \
      u (($0+1)/100):1 w l lw 3 lc 1 lt 1 \
  ,'' u (($0+1)/100):2 w l lw 3 lc 2 lt 1 \
  ,'' u (($0+1)/100):3 w l lw 3 lc 3 lt 1 \
  ,'' u (($0+1)/100):4 w l lw 3 lc 4 lt 1 \
  ,'' u (($0+1)/100):5 w l lw 3 lc 5 lt 1 \
  ,'' u (($0+1)/100):6 w l lw 3 lc 6 lt 1 \
  ,'' u (($0+1)/100):7 w l lw 3 lc 7 lt 1 \
  ,'' u (($0+1)/100):8 w l lw 3 lc 8 lt 1 \
  ,'' u (($0+1)/100):9 w l lw 3 lc 9 lt 1 \
  ,'' u (($0+1)/100):10 w l lw 3 lc 10 lt 0 \

