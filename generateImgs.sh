#!/bin/bash
# The optional argument of this script is a file name with override for variables below.
# In this script, if the vector variable samples is defined, it is used for computed the increment for each value of the variable given. Otherwise, it is used the vector increment[i] for each index of variable i.

### Default values ###
# Variables names for test.sh. Angle is measured in degrees
defaultVarNames=(s    d   v   K   n)
# default values for each variable
defaultValues=(  3.0 1.0  1.0 10  200)
# number of repetitions for each variable value
nTests=1
# variables name to do more experiments
varsExp=( v )
# minimum and maximum values for each variable to do more experiments
minExp=( 0.5 )     
maxExp=( 1.0 )  
#increment of above values, since the value minExp until is greater than maxExp 
increment=( 0.1 )   
# Name of the folder where the temporary configuration ini file named in variable configFile will be created
# and where the directories containg the logs will be created.
folderConf=.
#subdirectory to save the imagens
imgDir="img"
# The name of the log for the experiments performed by this script is randomly generated until a not used is found.
experimentsLog="experimentsLog$RANDOM.txt"
while [ -f "$imgDir/$folderConf/$experimentsLog$RANDOM.txt" ]; do experimentsLog="experimentsLog$RANDOM.txt"; done
# The name of the scenario file for stage is random too
scenarioFileName=automatic_$RANDOM
while [ -f "automatic_$RANDOM.world" ]; do experimentsLog="automatic_$RANDOM"; done

# The configuration file passed in first argument overrides the above values
if [ ! -z $1  ]; then
  . $1
fi

# Creates a config file from arguments.
# Argument 1: name of variable.
# Argument 2: value of this variable.
# Argument 3: name of configuration file.
# Argument 4: folder name for save the log file.
# Argument 5: log's filename.
makeConfigFile(){
  if [ $# -ne 5 ]; then
    echo "Wrong number of parameter for makeConfigFile." $# "are given."
    echo "Parameters were: " $@
    return 1
  fi
  local varName=$1
  local varValue=$2
  local configName=$3
  local folderLog=$4
  local logname=$5
  echo "log="$logname > $configName
  echo "folder="$folderLog >> $configName
  size=${#defaultVarNames[@]}
  for (( i=0; i<$size; i++ )); do 
    if [ ${defaultVarNames[$i]} != $varName ]; then
      echo ${defaultVarNames[$i]}"="${defaultValues[$i]} >> $configName 
    else
      echo $varName"="$varValue >> $configName
    fi
  done
  echo "scenario="$scenarioFileName >> $configName
}

# Peform experiments from given paramenters.
# Argument 1: number of experiments for each variable value.
# Argument 2: name of the variable being experimented.
# Argument 3: minimum value of the variable.
# Argument 4: maximum value of the variable.
# Argument 5: increment in variable.
# 
doExperiments(){
  if [ $# -ne 5 ]; then
    echo "Wrong number of parameter for doExperiments. " $# "are given."
    echo "Parameters were: " $@
    return 1
  fi
  local nTests=$1
  local varName=$2
  local minValue=$3
  local maxValue=$4
  local varInc=$5
  varValue=$minValue

  echo "Log for generateImgs.sh $@" &>> $imgDir/$folderConf/$experimentsLog
  # The configuration file of test.sh is random too.
  configFile="configExperiment$RANDOM.ini"
  while [ -f "$imgDir/$folderConf/$configFile" ]; do configFile="configExperiment$RANDOM.ini"; done
  while (( $(echo "$varValue <= $maxValue" | bc -l) ));
  do
    logsFolder=$imgDir/$folderConf/$varName\_$varValue
    local i
    for (( i=0; i<$nTests; i++ )); 
    do
      logsName=log\_$i
      if [ ! -e $imgDir/$logsFolder/*.png ]; then
        makeConfigFile $varName $varValue $imgDir/$folderConf/$configFile "$logsFolder" $logsName
        echo "******* test for "$varName" = "$varValue"  ********" &>> $imgDir/$folderConf/$experimentsLog
        if [ ! -d $logsFolder ]; then
          mkdir $logsFolder;
        fi
        
        bash test.sh $imgDir/$folderConf/$configFile video &>> $imgDir/$folderConf/$experimentsLog
        
        #the first image is badly cropped, so remove it.
        rm stage-000000.png
        mv stage*png $imgDir/$folderConf/
      fi;
    done
    varValue=`echo "$varValue + $varInc" | bc -l`
  done
}

mkdir -p $imgDir/$folderConf
# initalise log for this script
cat /dev/null > $imgDir/$folderConf/$experimentsLog
sizeVars=${#varsExp[@]}
for (( i=0; i<$sizeVars; i++ )); do
  if [ ! -z $samples ]; then
    varInc=`echo "(${maxExp[$i]}-${minExp[$i]})/(${samples[$i]} - 1)" | bc -l`
  else
    varInc=${increment[$i]}
  fi
  doExperiments $nTests ${varsExp[$i]} ${minExp[$i]} ${maxExp[$i]} $varInc 
done

