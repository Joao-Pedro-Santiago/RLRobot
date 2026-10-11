"""
inference_server.py

Servidor de INFERENCIA (nao de treino) para o RLFollower. Carrega uma politica
PPO ja treinada e a serve para VARIOS robos (RLRobot) simultaneamente, ao
contrario do StageEnv de treino, que so aceita 1 conexao por vez.

Diferenca fundamental em relacao ao stage_env.py usado no treino:
  - Nao ha calculo de reward, nao ha nocao de "episodio" (phase 0/1), nao ha
    model.learn(). E so: recebe estado -> normaliza -> model.predict() ->
    devolve acao. Isso e tudo que o RLRobot.cpp precisa (o `phase` do treino
    nunca faz parte da observacao de 16 valores, entao a politica nao perde
    nada rodando assim).
  - Cada RLRobot que conectar vira uma thread propria. O modelo (rede neural)
    e compartilhado (read-only) entre as threads -- model.predict() e seguro
    para chamadas concorrentes de leitura.

Uso:
  python inference_server.py --model models/PPO_Robot/ppo_robot_curriculum_final.zip

Depois disso, basta subir o Stage com quantos robos "ingenuos" (RLRobot)
quiser no cenario de teste -- todos vao conectar na mesma porta 5000 e serem
atendidos concorrentemente por esta mesma politica.
"""

import argparse
import socket
import struct
import threading

import numpy as np
from stable_baselines3 import PPO

MAX_DIST = 15.0
MAX_LASER = 3.0
MAX_VEL = 1.0


def normalize_state(state: np.ndarray) -> np.ndarray:
    state = np.array(state, dtype=np.float32)
    state[0:2] = state[0:2] / MAX_DIST
    state[2:4] = state[2:4] / MAX_VEL
    state[4:16] = state[4:16] / MAX_LASER
    return np.clip(state, -5, 5)


def handle_client(conn: socket.socket, addr, model: PPO, robot_id: int):
    print(f"[robô {robot_id}] conectado de {addr}")
    try:
        while True:
            data = conn.recv(16 * 8)
            if not data or len(data) < 16 * 8:
                break

            state = np.array(struct.unpack("16d", data))
            if abs(state[0]) > MAX_DIST * 10 or abs(state[1]) > MAX_DIST * 10:
                state[0] = 0.0
                state[1] = 0.0
            state = np.nan_to_num(state, nan=0.0, posinf=MAX_LASER, neginf=0.0)

            obs = normalize_state(state)
            action, _ = model.predict(obs, deterministic=True)

            conn.sendall(struct.pack("i", int(action)))
    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        conn.close()
        print(f"[robô {robot_id}] desconectado")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, required=True, help="Caminho do .zip do modelo PPO treinado")
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    print(f"Carregando modelo: {args.model}")
    model = PPO.load(args.model)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    # backlog alto o suficiente para aceitar varios robos conectando quase
    # ao mesmo tempo quando o Stage sobe o cenario inteiro de uma vez
    server.listen(128)
    print(f"Servidor de inferência escutando em {args.host}:{args.port}")
    print("Suba o Stage com quantos robôs RLRobot quiser -- todos serão atendidos.")

    robot_counter = 0
    try:
        while True:
            conn, addr = server.accept()
            robot_counter += 1
            t = threading.Thread(
                target=handle_client,
                args=(conn, addr, model, robot_counter),
                daemon=True,
            )
            t.start()
    except KeyboardInterrupt:
        print("\n🛑Encerrando servidor de inferência...")
    finally:
        server.close()


if __name__ == "__main__":
    main()
