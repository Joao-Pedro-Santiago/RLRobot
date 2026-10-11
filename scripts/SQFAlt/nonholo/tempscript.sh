arg=$1
for j in `seq 1 9`; do
  bash experiment.sh scripts/SQFAlt/nonholo/confTRVF\_prob$arg\_$j.sh
done
