"""
plot_lines_ahf_vs_rlfollower.py

Graficos de LINHA (nao de barra) comparando AHF (Alt NC) vs RLFollower,
usando os dados de:

  baseline_logs/original root/<holo|nonholo>/NoCoordAlt/<SQF|TRVF>/robots_<100|200>/m_<X>
  rlfollower_experiments/<holo|nonholo>/RLFollower/<SQF|TRVF>/robots_<100|200>/m_<X>

Eixo X: percentual de robos ad hoc (10%, 50%, 90%)
Eixo Y: uma das 4 metricas abaixo (uma figura por metrica):

  1) Throughput (1/s)                        -- indice 19 (derivado)
  2) Total time of the simulation (s)         -- indice 10
  3) Sum of total time (s)                    -- indice 10, somado entre
                                                  robots_100 e robots_200
                                                  (mesmo estilo da Fig. 5.37
                                                  da tese, que agrega os
                                                  totais de robos numa unica
                                                  curva por percentual)
  4) Last robot's reaching time (s)           -- indice 9

Para as metricas 1, 2 e 4: uma figura por (cinematica, algoritmo aware),
JUNTANDO robots_100 e robots_200 no mesmo grafico -- 4 curvas: AHF(100),
AHF(200), RLFollower(100), RLFollower(200). Mesma cor por fonte (AHF/RL),
estilo de linha diferente por total (solido=100, tracejado=200).

Para a metrica 3 (soma): uma figura por (cinematica, algoritmo aware), com
2 curvas (AHF vs RLFollower) vs percentual, ja somando robots_100+robots_200.

Como rodar (dentro da pasta com baseline_logs/ e rlfollower_experiments/):
    python plot_lines_ahf_vs_rlfollower.py
"""

import glob

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.rcParams.update({'font.size': 14})

# ---------------------------------------------------------------------------
# AJUSTE CONFORME SEU AMBIENTE
# ---------------------------------------------------------------------------
BASELINE_PREFIX = "baseline_logs/original root"
RLFOLLOWER_PREFIX = "rlfollower_experiments"
BASELINE_ALT_DIR = "NoCoordAlt"     # AHF com NC como alternativa (default da tese)
RLFOLLOWER_ALT_DIR = "RLFollower"

KINEMATICS = ["nonholo", "holo"]
AWARE_ALGOS = ["SQF", "TRVF"]
TOTALS = [100, 200]
PCTS = [10, 50, 90]

IDX_LAST_REACH = 9
IDX_TOTAL_TIME = 10

# Liga/desliga a remocao de outliers. Deixe False se quiser ver os dados
# "crus" (sem filtro) para comparar/auditar.
FILTER_OUTLIERS = True
IQR_MULTIPLIER = 1.5   # 1.5 = padrao estatistico (Tukey); aumente p/ ser menos agressivo


def remove_outliers_iqr(values, multiplier=IQR_MULTIPLIER):
    """Remove outliers pelo metodo do intervalo interquartil (IQR/Tukey):
    descarta valores fora de [Q1 - m*IQR, Q3 + m*IQR]. Retorna (valores_filtrados,
    valores_removidos). E o mesmo metodo usado, por exemplo, para desenhar os
    'whiskers' de um boxplot -- robusto a poucos valores extremos sem exigir
    suposicao de distribuicao normal."""
    if len(values) < 4:
        # com poucas amostras o IQR fica instavel demais para ser confiavel;
        # melhor nao filtrar
        return values, []
    arr = np.array(values)
    q1, q3 = np.percentile(arr, [25, 75])
    iqr = q3 - q1
    if iqr == 0:
        return values, []
    lower = q1 - multiplier * iqr
    upper = q3 + multiplier * iqr
    mask = (arr >= lower) & (arr <= upper)
    return arr[mask].tolist(), arr[~mask].tolist()


def read_metric_mean(prefix, alt_dir, aware_algo, kinem, total, pct, metric):
    """Le todos os log_* de um cenario e retorna (media, erro_95ci, valores_brutos)
    de uma metrica. metric: 'throughput' | 'total_time' | 'last_reach'
    Nao assume numeracao fixa dos arquivos log_* (0-based ou 1-based)."""
    m_val = round(total * pct / 100)
    folder = f"{prefix}/{kinem}/{alt_dir}/{aware_algo}/robots_{total}/m_{m_val}"
    log_paths = glob.glob(f"{folder}/log_*")

    values = []
    for path in log_paths:
        try:
            with open(path) as f:
                lines = f.readlines()
        except (FileNotFoundError, IsADirectoryError):
            continue
        if len(lines) <= max(IDX_LAST_REACH, IDX_TOTAL_TIME):
            continue
        try:
            first_us = float(lines[8])
            last_us = float(lines[9])
            total_time_us = float(lines[IDX_TOTAL_TIME])
        except ValueError:
            continue

        if metric == "throughput":
            denom = (last_us - first_us) / 1e6
            values.append((total - 1) / denom if denom > 0 else np.nan)
        elif metric == "total_time":
            values.append(total_time_us / 1e6)
        elif metric == "last_reach":
            values.append(last_us / 1e6)

    values = [v for v in values if not np.isnan(v)]
    if not values:
        print(f"    [aviso] sem dados: {alt_dir}/{aware_algo} {kinem} "
              f"robots_{total} pct={pct}% (pasta: '{folder}')")
        return np.nan, 0.0, []

    label = f"{alt_dir}/{aware_algo} {kinem} robots_{total} pct={pct}%"

    if FILTER_OUTLIERS:
        filtered, removed = remove_outliers_iqr(values)
        if removed:
            print(f"    [outlier] {label}: removidos {len(removed)}/{len(values)} "
                  f"valores fora do IQR -> {[round(v, 1) for v in removed]} "
                  f"(mantidos: min={min(filtered):.1f}, max={max(filtered):.1f})")
        values = filtered if filtered else values

    mean = float(np.mean(values))
    se = np.std(values, ddof=1) / np.sqrt(len(values)) if len(values) > 1 else 0.0
    return mean, 1.96 * se, values




def plot_combined_totals(x, series, ylabel, filename):
    """series: dict {label: (y_values, y_errs, color, marker, linestyle)}"""
    fig, ax = plt.subplots(figsize=(7.5, 5.5))
    for label, (y, yerr, color, marker, linestyle) in series.items():
        ax.errorbar(x, y, yerr=yerr, marker=marker, linestyle=linestyle,
                    color=color, label=label, capsize=5)
    ax.set_xlabel("Porcentagem de robôs ad hoc (%)")
    ax.set_ylabel(ylabel)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(filename, bbox_inches="tight", pad_inches=0.05)
    print(f"{filename} gerado")
    plt.close(fig)


def make_simple_metric_plots(metric, ylabel, label_tag):
    """Metricas 1, 2 e 4: uma figura por (kinem, algo), juntando robots_100
    e robots_200 no MESMO grafico (4 curvas: AHF-100, AHF-200, RL-100, RL-200)."""
    # cores fixas por fonte (AHF/RL), estilo de linha diferenciando o total
    colors = {"AHF": "#4C72B0", "RL": "#DD8452"}
    linestyles = {100: "-", 200: "--"}
    markers = {100: "o", 200: "s"}

    for kinem in KINEMATICS:
        for algo in AWARE_ALGOS:
            series = {}
            for total in TOTALS:
                ahf_results = [read_metric_mean(BASELINE_PREFIX, BASELINE_ALT_DIR,
                                                 algo, kinem, total, pct, metric)
                               for pct in PCTS]
                rl_results = [read_metric_mean(RLFOLLOWER_PREFIX, RLFOLLOWER_ALT_DIR,
                                                algo, kinem, total, pct, metric)
                              for pct in PCTS]
                y_ahf = [r[0] for r in ahf_results]
                err_ahf = [r[1] for r in ahf_results]
                y_rl = [r[0] for r in rl_results]
                err_rl = [r[1] for r in rl_results]

                series[f"AHF ({total} robôs)"] = (
                    y_ahf, err_ahf, colors["AHF"], markers[total], linestyles[total])
                series[f"RLFollower ({total} robôs)"] = (
                    y_rl, err_rl, colors["RL"], markers[total], linestyles[total])

            filename = f"{label_tag}_{kinem}_{algo}.pdf"
            plot_combined_totals(PCTS, series, ylabel, filename)


def make_sum_total_time_plots():
    """Metrica 3: soma do tempo total entre robots_100 e robots_200,
    uma figura por (kinem, algo). Erro propagado em quadratura (soma de
    variancias independentes), assumindo robots_100 e robots_200
    estatisticamente independentes entre si."""
    for kinem in KINEMATICS:
        for algo in AWARE_ALGOS:
            y_ahf_sum = np.zeros(len(PCTS))
            y_rl_sum = np.zeros(len(PCTS))
            err_ahf_sum_sq = np.zeros(len(PCTS))
            err_rl_sum_sq = np.zeros(len(PCTS))

            for total in TOTALS:
                ahf_results = [read_metric_mean(BASELINE_PREFIX, BASELINE_ALT_DIR,
                                                 algo, kinem, total, pct, "total_time")
                               for pct in PCTS]
                rl_results = [read_metric_mean(RLFOLLOWER_PREFIX, RLFOLLOWER_ALT_DIR,
                                                algo, kinem, total, pct, "total_time")
                              for pct in PCTS]

                y_ahf = np.nan_to_num(np.array([r[0] for r in ahf_results]))
                err_ahf = np.array([r[1] for r in ahf_results])
                y_rl = np.nan_to_num(np.array([r[0] for r in rl_results]))
                err_rl = np.array([r[1] for r in rl_results])

                y_ahf_sum += y_ahf
                y_rl_sum += y_rl
                err_ahf_sum_sq += err_ahf ** 2
                err_rl_sum_sq += err_rl ** 2

            err_ahf_sum = np.sqrt(err_ahf_sum_sq)
            err_rl_sum = np.sqrt(err_rl_sum_sq)

            filename = f"sum_total_time_{kinem}_{algo}.pdf"
            series = {
                "AHF (soma 100+200)": (y_ahf_sum, err_ahf_sum, "#4C72B0", "o", "-"),
                "RLFollower (soma 100+200)": (y_rl_sum, err_rl_sum, "#DD8452", "s", "-"),
            }
            ylabel = f"Soma dos tempos totais = {TOTALS} (s)"
            plot_combined_totals(PCTS, series, ylabel, filename)


def main():
    print("=== Grafico 1: Throughput (1/s) ===")
    make_simple_metric_plots("throughput", "Vazão/Throughput(1/s)", "throughput")

    print("\n=== Grafico 2: Total time of the simulation (s) ===")
    make_simple_metric_plots("total_time", "Tempo total da simulação (s)", "total_time")

    print("\n=== Grafico 3: Sum of total time (s) ===")
    make_sum_total_time_plots()

    print("\n=== Grafico 4: Last robot's reaching time (s) ===")
    make_simple_metric_plots("last_reach", "Tempo de chegada do último robô (s)", "last_reach")


if __name__ == "__main__":
    main()