"""
train_curriculum.py

Principais diferencas em relacao ao train.py original:
  1) Salva checkpoints periodicos do modelo (nao so no final), essencial
     quando o treino leva horas passando por dezenas de cenarios.
  2) Permite retomar de um checkpoint (--resume), caso o processo do Stage
     trave/precise ser reiniciado no meio do curriculo.
  3) total_timesteps escalado para o numero de cenarios x repeticoes que
     o run_curriculum.sh vai gerar, em vez de um valor fixo arbitrario.
  4) Mantem o log do TensorBoard para acompanhar a curva de recompensa por
     cenario (o "tag" de cada cenario aparece nos prints do RLRobot/CPP,
     olhe os logs em training_logs/<tag>/ para cruzar com o tempo de treino).

Uso tipico:
  # Terminal 1: sobe o servidor RL e comeca a esperar conexoes
  python train_curriculum.py

  # Terminal 2 (mesma pasta do RLFollower): dispara os cenarios
  bash run_curriculum.sh
"""

import argparse
import os
import signal

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stage_env import StageEnv


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--total-timesteps",
        type=int,
        default=160_000,
        help=(
            "Total de passos de treino. Regra pratica: "
            "(numero_de_cenarios * N_REPEATS * passos_medios_por_episodio). "
            "ajuste observando o TensorBoard."
        ),
    )
    p.add_argument("--learning-rate", type=float, default=1e-4)
    p.add_argument("--checkpoint-every", type=int, default=20_000)
    p.add_argument("--models-dir", type=str, default="models/PPO_Robot")
    p.add_argument("--logdir", type=str, default="logs")
    p.add_argument(
        "--resume",
        type=str,
        default=None,
        help="Caminho para um .zip de checkpoint para retomar o treino.",
    )
    p.add_argument("--host", type=str, default="127.0.0.1")
    p.add_argument("--port", type=int, default=5000)
    return p.parse_args()



def main():
    args = parse_args()
 
    os.makedirs(args.models_dir, exist_ok=True)
    os.makedirs(args.logdir, exist_ok=True)
 
    env = StageEnv(host=args.host, port=args.port)
 
    if args.resume:
        print(f"Retomando treino a partir de: {args.resume}")
        model = PPO.load(args.resume, env=env, tensorboard_log=args.logdir)
    else:
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=args.learning_rate,
            tensorboard_log=args.logdir,
        )
 
    checkpoint_callback = CheckpointCallback(
        save_freq=args.checkpoint_every,
        save_path=args.models_dir,
        name_prefix="ppo_robot_curriculum",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )
 
    def _raise_keyboard_interrupt(signum, frame):
        raise KeyboardInterrupt()
 
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
 
    print("Iniciando o treinamento (aguardando o Stage/RLRobot conectar)...")
    print(f"-> total_timesteps={args.total_timesteps}, checkpoint a cada {args.checkpoint_every}")
    print("Dica: Ctrl+C a qualquer momento salva o modelo com o progresso ate ali.")
    try:
        model.learn(
            total_timesteps=args.total_timesteps,
            tb_log_name="PPO_Robot_curriculum_log",
            callback=checkpoint_callback,
            reset_num_timesteps=(args.resume is None),
        )
        print("Treinamento concluido (bateu o total_timesteps).")
        final_path = os.path.join(args.models_dir, "ppo_robot_curriculum_final")
    except KeyboardInterrupt:
        print(f"\nTreinamento interrompido manualmente em {model.num_timesteps} timesteps.")
        final_path = os.path.join(args.models_dir, "ppo_robot_curriculum_interrupted")
    finally:
        model.save(final_path)
        print(f"Modelo salvo em {final_path}.zip")
 
 
if __name__ == "__main__":
    main()