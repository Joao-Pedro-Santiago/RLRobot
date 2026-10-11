#!/bin/bash
# run_rlfollower_experiments.sh
#
# Reproduz os experimentos da tese (Passos, 2023, Secoes 5.4.1.3 e 5.4.2)
# substituindo o AHF pelo RLFollower como algoritmo dos robos "ad hoc" (m).
#
# n = robos aware (SQF/TRVF/NoCoord)   -- ver createScenario.cpp
# m = robos ad hoc (RLFollower aqui)   -- total = n + m
#
# Diferente do AHF, o RLFollower NAO usa neighbourhoodAngle nem
# alternativeAlgorithm (a politica aprendida ja decide tudo sozinha a
# partir da observacao), entao a matriz de experimentos e mais simples que
# a da tese: nao precisamos varrer o angulo.
#
# PRE-REQUISITO:
#   Suba o servidor de inferencia com o modelo treinado ANTES de rodar este
#   script (ele aceita varias conexoes simultaneas, uma por robo RLFollower):
#     python inference_server.py --model models/PPO_Robot/ppo_robot_curriculum_parallel_final.zip
#
# Uso:
#   bash run_rlfollower_experiments.sh SET_A     # espelha a Secao 5.4.1.3
#   bash run_rlfollower_experiments.sh SET_B     # espelha a Secao 5.4.2.1
#   bash run_rlfollower_experiments.sh SET_A SET_B   # roda os dois

set -u

N_REPEATS=40           
S_VAL=3
V_VAL=1
K_VAL=5
D_VAL=3
D_MAJ=13
HOLO_ORDER=(1 0)

TRAIN_LOG_ROOT="rlfollower_experiments"
mkdir -p "$TRAIN_LOG_ROOT"

run_scenario() {
  local holo=$1 algo=$2 total=$3 pct=$4 timeout_s=$5
  local kinem="nonholo"; [ "$holo" -eq 1 ] && kinem="holo"

  local m_rl=$(python3 -c "import math; print(round($total * $pct / 100))")
  local n_aware=$(( total - m_rl ))

  local tag="${kinem}/RLFollower/${algo}/robots_${total}/m_${m_rl}"
  local logdir="${TRAIN_LOG_ROOT}/${tag}"
  mkdir -p "$logdir"

  echo ">>> [$kinem] aware=$algo total=$total (n=$n_aware aware, m=$m_rl RL, ${pct}%)"

  for rep in $(seq 1 $N_REPEATS); do
    local cfg="rlf_conf_$(echo "$tag" | tr '/' '_')_${rep}.ini"

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
neighbourhoodAngle=0
alternativeAlgorithm=${algo}
scenario=automatic${RANDOM}${rep}
EOF

    # timeout externo, igual ao limite de 60min/20min usado na tese
    timeout "${timeout_s}" bash test.sh "${cfg}" >> "${logdir}/experiment_stdout.log" 2>&1
    if [ $? -eq 124 ]; then
      echo "  [rep ${rep}] excedeu ${timeout_s}s -- descartado (igual a tese faz com runs que nao terminam)" >> "${logdir}/experiment_stdout.log"
    fi

    rm -f "$cfg"
  done
}

# ---------------------------------------------------------------------------
# SET A -- espelha a Secao 5.4.1.3 (AHF/MT): 100 e 200 robos, 10/50/90% ad hoc
# ---------------------------------------------------------------------------
run_set_a() {
  echo "=== SET A: reproducao da Secao 5.4.1.3 (AHF individual/MT -> RLFollower) ==="
  local TOTALS=(100 200)
  local PCTS=(10 50 90)
  local ALGOS=(SQF TRVF)   # a tese usa SQF e TRVF aqui; adicione NoCoord se quiser

  for holo in "${HOLO_ORDER[@]}"; do
    for algo in "${ALGOS[@]}"; do
      for total in "${TOTALS[@]}"; do
        for pct in "${PCTS[@]}"; do
          run_scenario "$holo" "$algo" "$total" "$pct" 3600   # timeout 60 min
        done
      done
    done
  done
}

# ---------------------------------------------------------------------------
# SET B -- espelha a Secao 5.4.2.1 (comparacao com estado da arte):
# 20..300 robos (passo 20), 20/40/60/80% ad hoc
# ---------------------------------------------------------------------------
run_set_b() {
  echo "=== SET B: reproducao da Secao 5.4.2.1 (comparacao com estado da arte) ==="
  local TOTALS=(20 40 60 80 100 120 140 160 180 200)
  local PCTS=(20 40 60 80)
  local ALGOS=(SQF TRVF)   # a tese usa SQF e TRVF aqui; adicione NoCoord se quiser

  for holo in "${HOLO_ORDER[@]}"; do
    for algo in "${ALGOS[@]}"; do
      for total in "${TOTALS[@]}"; do
        for pct in "${PCTS[@]}"; do
          run_scenario "$holo" "$algo" "$total" "$pct" 3600   # timeout 60 min
        done
      done
    done
  done
}

for arg in "$@"; do
  case "$arg" in
    SET_A) run_set_a ;;
    SET_B) run_set_b ;;
    *) echo "Argumento desconhecido: $arg (use SET_A e/ou SET_B)" ;;
  esac
done

echo "=== Concluido. Logs em ${TRAIN_LOG_ROOT}/ com a mesma estrutura de logs.tar.gz ==="
