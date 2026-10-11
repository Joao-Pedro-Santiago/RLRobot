

if [ $# -ne 1 ]; then
  echo "Incorrect number of arguments"
  echo "Usage: script outputDirectory "
  exit
fi

outputDir="$1"

for img in "$outputDir"/stage*.png; do 
  mogrify -gravity South -chop 0x25 "$img"; 
done

