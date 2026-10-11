
if [ $# -ne 1 ]; then
  echo "Usage:"
  echo "  script rootDir"
  exit
fi


rootDir="$1"
LOGSPERDIR=39

for expr in '*m_*' '*n_*' '*neighbourhoodAngle_*'; do
  find "$rootDir" -name $expr -type d | while read aline; do
    fulldir="$aline"
      numlogs=`ls "$fulldir"/log* 2> /dev/null | wc -l`
      if [ $numlogs -le $LOGSPERDIR ]; then
        echo $numlogs "$fulldir"
        #~ rm -r "$fulldir"
      fi
  done 
done 
  
