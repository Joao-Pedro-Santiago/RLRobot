for alg in TRVF SQF; do
  for hol in holo nonholo; do
    for num in `seq 20 20 300`; do
      for prob in `seq 1 9`; do
        sed 's/folderConf=\"'$hol'\/'$alg'\//folderConf=\"'$hol'\/NoCoordAlt\/'$alg'\//' $hol/conf$alg\_prob$num\_$prob.sh | sed 's/neighbourhoodAngle)/neighbourhoodAngle alternativeAlgorithm)/' | sed 's/'$alg' 90)/'$alg' 90 NoCoord)/' > tmp_file
        mv tmp_file $hol/conf$alg\_prob$num\_$prob.sh
      done
    done
  done
done
