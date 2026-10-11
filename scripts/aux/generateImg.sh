if [ $# -ne 2 ]; then
  echo "Incorrect number of arguments"
  echo "Usage: script outputDirectory configFile"
  exit
fi

outputDir="$1"
configFile="$2"

bash test.sh "$configFile" video
rm stage-000000.png
mv stage*.png "$outputDir"
