arg=$1
for alg in SQF TRVF; do
  for j in `seq 1 9`; do
    bash experiment.sh conf$alg\_prob$arg\_$j.sh
  done
done
