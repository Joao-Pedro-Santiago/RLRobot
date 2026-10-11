from stable_baselines3 import PPO
from stage_env import StageEnv
import os

# Pasta para salvar os modelos e logs
models_dir = "models/PPO_Robot"
logdir = "logs"

if not os.path.exists(models_dir): os.makedirs(models_dir)
if not os.path.exists(logdir): os.makedirs(logdir)

#Criar o ambiente
env = StageEnv()

#Definir o modelo PPO
model = PPO("MlpPolicy", env, verbose=1, learning_rate=1e-4, tensorboard_log=logdir)

#Treinar o modelo
print("Iniciando o treinamento...")
model.learn(total_timesteps=300000, tb_log_name="PPO_Robot_log")
print("Treinamento concluído.")

#Salvar o modelo treinado
model.save("ppo_robot_teste")
print("Modelo salvo como ppo_robot_teste.zip")