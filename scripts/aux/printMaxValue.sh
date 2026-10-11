#Print the maximum value at a line over all log files from a given root directory.

if [ $# -ne 1 ]; then
  echo "Usage:"
  echo "  script rootDir"
  exit
fi

rootDir="$1"
lineNumber=11
maxValue=0
find "$rootDir" -name \*log_\* -type f | xargs -d '\n' -I '()' bash -c "head -n "$lineNumber" "\"'()'\""| tail -n 1" | awk 'BEGIN{max=0} {l=$1} l>max {max=l} END{print max}'
