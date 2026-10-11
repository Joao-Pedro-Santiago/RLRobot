# Plot input versus output.

input=vi
output=vo
logFile=log_0E

# Generates a file containing the values to plot
# Argument 1: name of the variable
# Argument 2: name of the input file containing the variable name and values
# Argument 3, optional: fields to output, separated by commas
# output the results in a text file named $1.txt
filterVariable(){
  local varname=$1
  local logFile=$2
  local fields=3
  if [ ! -z "$3" ]; then
    fields=$3
  fi
  
  awk '/'"$varname"'/ {print $0}' $logFile | sed 's/^\ \ *//;s/,//;s/://;s/'"$varname"'=//' | cut -d" " -f$fields > "$varname".txt
}

filterVariable "$input" "$logFile" 
filterVariable "$output" "$logFile" 
python3 plotVelocities.py
