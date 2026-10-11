#!/bin/bash

# Remove todas as imagens geradas pelo Stage.
# Uso:
#   ./delete_frames.sh

echo "Serão removidas todas as imagens 'stage-*.png'."
read -p "Tem certeza? (s/N): " resposta

case "$resposta" in
    [sS]|[sS][iI][mM])
        rm -v stage-*.png
        echo "Imagens removidas."
        ;;
    *)
        echo "Operação cancelada."
        ;;
esac