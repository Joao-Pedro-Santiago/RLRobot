# sort the logs in all sub-directories from a given root directory

if [ $# -ne 1 ]; then
  echo "Usage:"
  echo "  script rootDir"
  exit
fi

rootDir="$1"
MAXNUMLOGS=80

for expr in '*m_*' '*n_*' '*neighbourhoodAngle_*'; do
  find "$rootDir" -name $expr -type d | while read aline; do
    for n in `seq 0 $MAXNUMLOGS`; do
      if [ ! -f "$aline"/log_$n ]; then
        for ((ii=$MAXNUMLOGS; ii > $n; ii--)); do 
          [ -f "$aline"/log_$ii ] && mv "$aline"/log_$ii "$aline"/log_$n && break
        done
      fi
    done
  done
done
