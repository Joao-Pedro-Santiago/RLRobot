#!/bin/bash

# Uso:
#   ./make_video.sh nome_do_video

if [ $# -lt 1 ]; then
    echo "Uso: $0 <nome_do_video>"
    exit 1
fi

OUTPUT="$1.mp4"

ffmpeg -y \
    -framerate 30 \
    -i stage-%06d.png \
    -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" \
    -c:v libx264 \
    -preset slow \
    -crf 18 \
    -pix_fmt yuv420p \
    -movflags +faststart \
    "$OUTPUT"

if [ $? -eq 0 ]; then
    echo "Vídeo criado com sucesso: $OUTPUT"
else
    echo "Erro ao criar o vídeo."
    exit 1
fi