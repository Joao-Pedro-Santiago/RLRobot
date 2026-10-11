# Purge repeated logs by md5 sum from a root dir.

if [ $# -ne 1 ]; then
  echo "Usage:"
  echo "  $0 rootDir"
  exit
fi

rootDir="$1"

find "$rootDir" -name \*log_[0-9]\* -type f -exec md5sum '{}' \; | sort -s -n -k 1,1 | awk 'BEGIN{Curr=""} {Prev=Curr; Curr=$1; if (Curr==Prev) print $2}' | xargs -d '\n' rm 
