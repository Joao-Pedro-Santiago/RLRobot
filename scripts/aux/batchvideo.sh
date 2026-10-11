# Run experiments and creates videos based on a list of prefixes. The prefix is used for the ini file and for the directory where it will be saved.

ECHODEBUG=""

declare -a LIST=( configVideo100NoCoords7holo  )

for dire in "${LIST[@]}"; do 
  $ECHODEBUG mkdir -p img/"$dire";
  $ECHODEBUG bash test.sh "$dire".ini video; 
  $ECHODEBUG rm stage-000000.png; 
  $ECHODEBUG mv stage*.png img/"$dire"; 
  $ECHODEBUG bash scripts/aux/chopImgs.sh img/"$dire"; 
  $ECHODEBUG pushd img/"$dire"; 
  $ECHODEBUG ffmpeg -r 10 -i stage-%06d.png video.mp4; 
  $ECHODEBUG popd;  
done
