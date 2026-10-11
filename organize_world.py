import os
import shutil
import time
import re

# ==========================================
# CONFIGURAÇÕES DE PASTAS
# ==========================================
# Pasta onde os arquivos .world são gerados (diretório atual)
SOURCE_DIR = "." 

# Pasta para onde os arquivos serão movidos
TARGET_DIR = "./saved_worlds_experiments"

# Intervalo em segundos entre as checagens do loop
CHECK_INTERVAL = 1.0  

# Padrão do nome do arquivo (procura por arquivos tipo automatic12345.world)
PATTERN = re.compile(r"^automatic\d+\.world$")


def move_world_files():
    # Cria a pasta de destino se ela não existir
    os.makedirs(TARGET_DIR, exist_ok=True)
    
    print(f"🔍 Monitorando '{SOURCE_DIR}' por arquivos automaticXXXXX.world...")
    print(f"📁 Salvando em: '{TARGET_DIR}'")
    print("Pressione CTRL+C para encerrar.\n")

    try:
        while True:
            # Lista todos os arquivos no diretório de origem
            for filename in os.listdir(SOURCE_DIR):
                # Verifica se o nome do arquivo corresponde ao padrão automaticXXXXX.world
                if PATTERN.match(filename):
                    src_path = os.path.join(SOURCE_DIR, filename)
                    dest_path = os.path.join(TARGET_DIR, filename)
                    
                    try:
                        # Tenta mover o arquivo
                        shutil.move(src_path, dest_path)
                        print(f"📦 [Salvo]: {filename} -> {TARGET_DIR}/")
                    except PermissionError:
                        # Caso o Stage ainda esteja escrevendo/usando o arquivo
                        pass
                    except Exception as e:
                        print(f"⚠️ Erro ao mover {filename}: {e}")

            # Aguarda o tempo definido antes da próxima varredura
            time.sleep(CHECK_INTERVAL)

    except KeyboardInterrupt:
        print("\n🛑 Monitoramento encerrado pelo usuário.")

if __name__ == "__main__":
    move_world_files()