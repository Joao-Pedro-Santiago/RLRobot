from stage_env import StageEnv
from stable_baselines3 import PPO
import time

#1 Carregando o ambiente
env = StageEnv()

#2. Carregando o modelo treinado
model_path = "ppo_robot_final.zip"
model = PPO.load(model_path)
print(f"Modelo {model_path} carregado com sucesso!")

obs, _ = env.reset()

while True:
    action, _ = model.predict(obs, deterministic=True)
    try:
        obs, reward, done, truncated, info = env.step(action)
        
        if done:
            print("Robô chegou no alvo! Agora ele iniciará a saída automática.")
            # NÃO chame env.reset() aqui. Deixe o C++ assumir a saída.
            
    except ConnectionError:
        print("Simulação encerrada pelo Stage. Reiniciando ambiente...")
        obs, _ = env.reset() # Reset real apenas quando o Stage fechar