# Run all experiments for angle of $\gamma_{ad}$-area

DEBUGECHO=""

for hol in `echo nonholo holo | tr " " "\n" | shuf`; do
  for alg in `echo SQF TRVF | tr " " "\n" | shuf`; do
    for numTotal in `seq 100 100 200 | shuf`; do
      for m in `seq 1 4 9 | shuf`; do
        for alt in `echo NoCoordAlt TRVFAlt SQFAlt | tr " " "\n" | shuf`; do
          if [ $alt == TRVFAlt -a $alg == TRVF ] || [ $alt == SQFAlt -a $alg == SQF ] ; then
            continue
          fi
          $DEBUGECHO bash experiment.sh scripts/$alt/$hol/conf$alg\_angle\_$numTotal\_$m.sh;
        done
      done
    done
  done
done
