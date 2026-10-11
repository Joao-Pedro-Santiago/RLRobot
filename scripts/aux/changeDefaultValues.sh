if [ -z $1 ]; then
  echo "Usage:"
  echo "   $0 previous_value value_to_change"
  exit
fi

NEWVALUE=$2
NUMCHANGE=$1

for dire in NoCoordAlt SQFAlt TRVFAlt; do
  for alg in TRVF SQF; do
    if [ "$dire" = SQFAlt -a "$alg" = SQF ] || [ "$dire" = TRVFAlt -a "$alg" = TRVF ]; then
      continue
    fi
    for hol in holo nonholo; do
      for num in `seq 20 20 300`; do
        for prob in `seq 1 9`; do
          sed 's/\(defaultValues=.\+\) '$NUMCHANGE' \(.\+\)/\1 '$NEWVALUE' \2/' $dire/$hol/conf$alg\_prob$num\_$prob.sh  > tmp_file
          mv tmp_file $dire/$hol/conf$alg\_prob$num\_$prob.sh
        done
      done
    done
  done
done

for hol in holo nonholo; do
  for alg in SQF TRVF; do
    for numTotal in 100 200 300 400; do
      for m in 1 5 9; do
        sed 's/\(defaultValues=.\+\) '$NUMCHANGE' \(.\+\)/\1 '$NEWVALUE' \2/' NoCoordAlt/$hol/conf$alg\_angle\_$numTotal\_$m.sh > tmp_file
        mv tmp_file NoCoordAlt/$hol/conf$alg\_angle\_$numTotal\_$m.sh
      done
    done
  done
done

for hol in holo nonholo; do
  for suf in m1 angle; do
    for alg in TRVF SQF; do
      sed 's/\(defaultValues=.\+\) '$NUMCHANGE' \(.\+\)/\1 '$NEWVALUE' \2/' NoCoordAlt/$hol/conf$alg\_$suf.sh > tmp_file
      mv tmp_file NoCoordAlt/$hol/conf$alg\_$suf.sh
    done
  done
  for alg in NoCoord OnlyAdhoc; do
    sed 's/\(defaultValues=.\+\) '$NUMCHANGE' \(.\+\)/\1 '$NEWVALUE' \2/' NoCoordAlt/$hol/conf$alg.sh > tmp_file
    mv tmp_file NoCoordAlt/$hol/conf$alg.sh
  done
done

