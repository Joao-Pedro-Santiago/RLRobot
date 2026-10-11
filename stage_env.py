import gymnasium as gym
from gymnasium import spaces
import socket
import struct
import numpy as np

MAX_DIST = 15.0  # Distância máxima para normalização
MAX_LASER = 3.0  # Distância máxima dos sensores laser para normalização
MAX_VEL = 1.0    # Velocidade máxima para normalização


class StageEnv(gym.Env):
    def __init__(self, host='127.0.0.1', port=5000):
    
        super(StageEnv, self).__init__()
        self.adress = (host, port)

        self.action_space = spaces.Discrete(4)  # 4 ações: 0 - Parar, 1 - Ir para o alvo, 2 - Seguir um vizinho, 3 Sair do alvo
        self.phase = 0  # Fase inicial do experimento 0 = Buscando Alvo, 1 = Fugindo da área
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(16,), dtype=np.float64
        )

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(self.adress)
        self.server.listen(1)
        print(f"Servidor StageEnv rodando em {host}:{port}")
        self.conn = None
        self.last_dist = 0
        self.last_action = 0
    
    def normalize_state(self, state):

        state = np.array(state, dtype=np.float32)
        state[0:2] = state[0:2] / MAX_DIST 
        state[2:4] = state[2:4] / MAX_VEL
        state[4:16] = state[4:16] / MAX_LASER
    
        # Clip para garantir que nenhum valor saia de um range seguro
        return np.clip(state, -5, 5)

    def step(self, action):
        # 1. Enviar ação para o C++ RLRobot
        self.conn.send(struct.pack('i', action))

        data = self.conn.recv(16 * 8)  # Recebe 16 doubles (128 bytes)
        if not data: return np.zeros(16), 0, True, False, {}

        state = np.array(struct.unpack('16d', data))  # Desempacota os dados recebidos
        
        if abs(state[0]) > MAX_DIST * 10 or abs(state[1]) > MAX_DIST * 10:
            state[0] = 0.0
            state[1] = 0.0
        
        state = np.nan_to_num(state, nan=0.0, posinf=MAX_LASER, neginf=0.0)  # Substitui NaN por 0 e inf por limites máximos
        
        obs = self.normalize_state(state)  # Normaliza o estado
 
        dist_to_target = np.hypot(state[0], state[1])
        reward = 0.0 # Inicializa a recompensa

        min_laser = np.min(state[-12:])  # Distâncias dos sensores laser
        
        done = False  # Inicializa a flag de término do episódio

        # ==========================================
        # FASE 0: BUSCANDO O ALVO
        # ==========================================
        if self.phase == 0:
            # Recompensa por se aproximar do alvo
            reward = np.clip((self.last_dist - dist_to_target) * 10, -10, 10) 
            
            if min_laser < 0.8:
                reward -= 3  # Penalidade por risco de colisão
            if action != self.last_action:
                reward -= 0.8 # Penalidade por mudar de ação (incentiva consistência)
            if state[2] < 0.1: 
                reward -= 1.5 # Penalidade por reduzir a velocidade

            reward -= 0.2  # Penalidade de tempo
            
            # Transição de Fase (Atingiu o alvo)
            if dist_to_target < 3.15:
                reward += 50  # Bônus por chegar
                self.phase = 1  # Muda para a fase de fuga
                # NÃO setamos done = True aqui!

        # ==========================================
        # FASE 1: FUGINDO DO CONGESTIONAMENTO
        # ==========================================
        elif self.phase == 1:
            # Recompensa INVERTIDA: Ganha pontos por se AFASTAR do alvo (aumentar a distância)
            reward = np.clip((dist_to_target - self.last_dist) * 10, -10, 10) 
            
            # Punição SEVERA por ficar no engarrafamento
            if min_laser < 0.8:
                reward -= 10  
            
            # Penalidade de tempo maior para forçar a saída rápida
            reward -= 0.5 
            
            # Término do Episódio: Saiu da zona de confusão (ajuste esse valor conforme o tamanho do seu mapa)
            if dist_to_target > 8.0:
                reward += 50  # Bônus por conseguir fugir com sucesso
                done = True

        self.last_dist = dist_to_target
        self.last_action = action

        return obs, reward, done, False, {}
    
    def reset(self, seed=None, options=None):
        # Se houver conexão antiga aberta, fecha
        if self.conn:
            self.conn.close()
            self.conn = None
        
        # O reset agora apenas aguarda o início do próximo processo pelo experiment.sh
        #print("Aguardando inicialização...")
        self.conn, addr = self.server.accept()
        
        data = self.conn.recv(16 * 8)  # Recebe o estado inicial do C++
        state = np.array(struct.unpack('d' * 16, data))
        state = np.nan_to_num(state, nan=0.0, posinf=3.0, neginf=0.0)
        
        self.phase = 0
        self.last_dist = np.hypot(state[0], state[1])
        return self.normalize_state(state), {}