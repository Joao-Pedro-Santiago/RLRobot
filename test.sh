#execute tests and interrupts if execution run over a specified time
#Arguments:
#  1) configuration file
#  2 [optional]) "video" or "gui"

# Finds scenario entry
seekScenario(){
  local fileIni=$1
  while read line; do
    if [[ $line == scenario* ]]; then
      echo "${line/scenario*=/}"
      return
    fi
  done < $fileIni
}

if [ "$#" -lt 1 ]; then
  echo "Wrong number of arguments."
  echo "Arguments:"
  echo "   1) configuration file"
  echo "   optional) \"video\" or \"gui\""
  exit 1
fi

if [ "$2" == "video" ]; then
  video="video"
fi

if [ "$video" != "video" ]; then
  GUI="-g"
fi

if [ "$2" == "gui" ]; then
  GUI=""
fi

export STAGEPATH="$PWD/"
./createScenario $1 $video && automaticFile=$(seekScenario $1).world
if [ $? -eq 0 ]; then
  stage $GUI $automaticFile
fi
