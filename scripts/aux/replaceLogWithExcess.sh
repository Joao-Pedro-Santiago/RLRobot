#Replace missing log file with an excess log in the directory if it exists from a given root directory.

if [ $# -ne 1 ]; then
  echo "Usage:"
  echo "  $0 rootDir"
  exit
fi


rootDir="$1"
MAXNUMLOGS=80

for expr in '*m_*' '*n_*' '*neighbourhoodAngle_*'; do
  find "$rootDir" -name $expr -type d | while read aline; do
    fulldir="$aline"
    for n in `seq 0 $MAXNUMLOGS`; do
      if [ ! -f "$fulldir"/log_$n ]; then
        excessLog=`ls "$fulldir"/log[0-9]* 2> /dev/null | head -n 1 | tr -d '\n'`
        if [ ! -z "$excessLog" ]; then
          mv "$excessLog" "$fulldir"/log_$n
        fi
      fi
    done
  done 
done 
  
