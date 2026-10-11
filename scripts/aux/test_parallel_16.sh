


if [ -z $1 ]; then
  echo "Tests the parallel job in HEC."
  echo "Usage:"
  echo "   $0 script_to_run"
  exit
fi

MYSCRIPT="$1"

for ii in {1..16}; do
  bash $MYSCRIPT  &
done
