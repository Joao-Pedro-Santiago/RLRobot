#!/bin/bash
# run_curriculum.sh
#
# Orquestra o treinamento do RLFollower expondo o agente PPO (train.py) a um
# subconjunto REPRESENTATIVO dos cenarios encontrados em logs.tar.gz, em vez
# de rodar todos os milhares de combinacoes.
#
# IMPORTANTE (leia antes de rodar):
#   train.py NAO le os arquivos de log como dataset. Ele abre um servidor TCP
#   (StageEnv, porta 5000) e espera o C++ (RLRobot, dentro do Stage) se
#   conectar. Quem gera essas conexoes/episodios e o experiment.sh -> test.sh
#   -> stage, executando os controladores SQF/TRVF/NoCoord (e variantes Alt)
#   para os robos "informados" (n) e RLRobot para os robos "ingenuos"
#   restantes. Ou seja: os logs.tar.gz sao a SAIDA de rodadas anteriores
#   (baseline, sem RL) e servem aqui apenas como MAPA de quais combinacoes de
#   parametros (holo/nonholo, algoritmo, quantidade de robos, m,
#   neighbourhoodAngle) existem e valem a pena treinar. Este script usa esse
#   mapa para escolher cenarios, nao para ler dados de dentro deles.
#
# Uso:
#   1) Rode `python train.py` num terminal (ele fica escutando a porta 5000).
#   2) Rode `bash run_curriculum.sh` neste diretorio do RLFollower.
#
# O script cicla por combinacoes de:
#   - holo / nonholo               (cinematica holonomica vs diferencial)
#   - algoritmo do robo informado  (SQF, TRVF, NoCoord)
#   - porte do enxame              (pequeno / medio / grande)
#   - neighbourhoodAngle           (fechado / medio / aberto)
# repetindo cada combinacao N_REPEATS vezes (episodios) para dar ao PPO
# amostras suficientes por cenario antes de trocar de cenario (curriculo).

set -u

# ---------------------------------------------------------------------------
# CONFIGURACAO DO CURRICULO
# ---------------------------------------------------------------------------

HOLO_ORDER=(0 1)                 # 1=holo primeiro, 0=nonholo
ALGOS=(NoCoord SQF TRVF)         # algoritmo do(s) robo(s) informado(s)
ROBOT_COUNTS=(20 100 250)        # tamanho total do enxame (aware + RL)
ANGLES=(0 37.5 90)               # angulo de vizinhanca não usado pelo RLFollower, mas mantido para manter a compatibilidade com o mapa de logs.tar.gz
N_REPEATS=15                     # episodios por combinacao, no tcc está escrito com 45 pois como os angulos sao 3 e não são usados pelo RLFollower, isso só multiplica os episodios por 3, então 15*3=45

# Parametros fixos do experimento
S_VAL=3
V_VAL=1
K_VAL=5
D_VAL=3
D_MAJ=13

TRAIN_LOG_ROOT="training_logs"
mkdir -p "$TRAIN_LOG_ROOT"

echo "=== Iniciando curriculo de treinamento do RLFollower ==="
echo "Repeticoes por cenario: $N_REPEATS | Configuracao: 1 robo RL (m=1) vs (total-1) robos aware (n)"
echo

for holo in "${HOLO_ORDER[@]}"; do
  kinem="nonholo"; [ "$holo" -eq 1 ] && kinem="holo"

  for algo in "${ALGOS[@]}"; do
    for total in "${ROBOT_COUNTS[@]}"; do

      # AJUSTE DA PARAMETRIZACAO DO STAGE:
      # n = quantidade de robos informados/aware (SQF, TRVF, NoCoord)
      # m = quantidade de robos RL (sempre 1)
      n_aware=$(( total - 1 ))
      m_rl=1

      for angle in "${ANGLES[@]}"; do

        tag="${kinem}_${algo}_n${total}_a${angle}"
        logdir="${TRAIN_LOG_ROOT}/${tag}"
        mkdir -p "$logdir"

        echo ">>> Cenario: holo=$holo algoritmo=$algo total=$total (aware=$n_aware, RL=$m_rl) angle=$angle"

        for rep in $(seq 1 $N_REPEATS); do
          cfg="curriculum_conf_${tag}_${rep}.ini"

          cat > "$cfg" <<EOF
log=log_${rep}
folder=${logdir}
s=${S_VAL}
v=${V_VAL}
K=${K_VAL}
n=${n_aware}
m=${m_rl}
d=${D_VAL}
D=${D_MAJ}
holo=${holo}
algorithm=${algo}
neighbourhoodAngle=${angle}
alternativeAlgorithm=${algo}
scenario=automatic${RANDOM}${rep}
EOF

          # Executa o Stage: apenas 1 conexao TCP sera aberta e consumida pelo train_curriculum.py
          bash test.sh "${cfg}" >> "${logdir}/experiment_stdout.log" 2>&1

          rm -f "$cfg"
        done
      done
    done
  done
done

echo
echo "=== Curriculo concluido. Verifique os checkpoints do PPO e os logs em ${TRAIN_LOG_ROOT}/ ==="