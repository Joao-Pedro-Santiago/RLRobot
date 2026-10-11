# Run all experiments for m = 1

DEBUGECHO="" 

for hol in `echo nonholo holo | tr " " "\n" | shuf`; do
  for alg in `echo SQF TRVF | tr " " "\n" | shuf`; do
    $DEBUGECHO bash experiment.sh scripts/NoCoordAlt/$hol/conf$alg\_m1.sh;
  done
done
