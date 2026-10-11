
if [ $# -ne 1 ]; then
  echo "Usage:"
  echo "  script rootDir"
  exit
fi


rootDir="$1"
TIMELIMIT=$((3600*1000000))
EPSILON=$((600*1000000))
LINETOCHECK=11

for expr in '*m_*' '*n_*' '*neighbourhoodAngle_*'; do
  find "$rootDir" -name $expr -type d | while read aline; do
    fulldir="$aline"
    for logname in "$fulldir"/log*; do
      value=`head -n $LINETOCHECK "$logname" | tail -n 1`
      if [  $((TIMELIMIT-EPSILON)) -le $value  -a  $value -le $((TIMELIMIT+EPSILON))  ]; then
        echo rm "$logname"
      fi
    done
  done 
done 
  
