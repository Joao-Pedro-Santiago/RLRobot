#Remove a outlier log file from a list.

if [ $# -ne 1 ]; then
  echo "Usage:"
  echo "  $0 fileList"
  echo "    fileList: list of files of outliers to delete"
  exit
fi

fileList="$1"
while read aline; do
  rm "$aline"
done < "$fileList"

