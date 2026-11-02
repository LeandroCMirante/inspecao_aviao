"""Exemplo de MISSÃO (roda dentro da simulação, a cada quadro).

    ./executar.sh --missao exemplos/missao_inspecao.py
    ./executar.sh --headless --missao exemplos/missao_inspecao.py --gravar-video saidas/missao.mp4 --log-csv saidas/missao.csv

O que faz, em paralelo:
  * drone_arl: decola até 7,5 m, percorre o lado esquerdo da fuselagem do 747 tirando uma foto em cada ponto, volta
    pelo alto e pousa no ponto de partida;
  * ridgeback_franka: sai da fila, vai até embaixo do motor interno esquerdo, aponta a garra para cima e fotografa.

As fotos vão para saidas/fotos/. Copie este arquivo e altere os pontos para criar a sua própria missão.
A API disponível está em scripts/api_externa.py (classe API) e no DESENVOLVIMENTO.md.
"""
import os

PASTA = os.path.join(os.path.dirname(__file__), "..", "saidas", "fotos")

# pontos do drone (x, y, z) no mundo; o 747 tem o nariz em +X e o eixo da fuselagem em y = 0
PONTOS_DRONE = [(17, -6.5, 7.5), (8, -6.5, 7.5), (-2, -6.5, 7.5), (-12, -6.5, 7.5)]
# volta pela mesma altura (para não passar embaixo da asa) e pousa onde decolou
VOLTA_DRONE = [(17, -6.5, 7.5), (17, -6, 0.4)]
# caminho do Ridgeback: primeiro sai da fila (y = -10,5), depois anda até o motor (x = 6, y = -10,8)
PONTOS_FRANKA = [(30, -10.5), (6, -10.8)]

estado = {"drone": 0, "franka": 0, "espera": 0.0}


def inicio(api):
    print("[missao] robôs:", [r["nome"] for r in api.robos()], flush=True)
    api.selecionar("drone_arl")
    api.camera(1)  # câmera seguindo o drone


def passo(api, t):
    """Chamado a cada quadro. Retorne True quando a missão acabar."""
    drone_ok = passo_drone(api, t)
    franka_ok = passo_franka(api, t)
    return drone_ok and franka_ok


def passo_drone(api, t):
    i = estado["drone"]
    nav = api.estado("drone_arl")["navegando"]
    if i < len(PONTOS_DRONE):
        if i == 0 or not nav:
            if i > 0:  # chegou no ponto anterior: foto
                api.foto("drone_arl", os.path.join(PASTA, f"drone_ponto_{i}.png"))
            api.ir_para("drone_arl", *PONTOS_DRONE[i], vel=2.0)
            estado["drone"] += 1
        return False
    if i == len(PONTOS_DRONE) and not nav:
        api.foto("drone_arl", os.path.join(PASTA, f"drone_ponto_{i}.png"))
    j = i - len(PONTOS_DRONE)
    if j < len(VOLTA_DRONE) and not nav:
        pouso = j == len(VOLTA_DRONE) - 1
        api.ir_para("drone_arl", *VOLTA_DRONE[j], vel=2.0, tol=0.03 if pouso else 0.15)  # no pouso, chega até o chão
        estado["drone"] += 1
        return False
    return j >= len(VOLTA_DRONE) and not nav


def passo_franka(api, t):
    i = estado["franka"]
    if i < len(PONTOS_FRANKA):
        if i == 0 or not api.estado("ridgeback_franka")["navegando"]:
            api.ir_para("ridgeback_franka", *PONTOS_FRANKA[i], vel=1.0)
            estado["franka"] += 1
        return False
    if i == len(PONTOS_FRANKA):
        if api.estado("ridgeback_franka")["navegando"]:
            return False
        if api.girar_para("ridgeback_franka", -90):  # vira de frente para o motor
            api.pose_braco("ridgeback_franka", "inspecao")
            estado["espera"] = t
            estado["franka"] += 1
        return False
    if i == len(PONTOS_FRANKA) + 1 and t - estado["espera"] > 3.0:  # espera o braço chegar
        api.foto("ridgeback_franka", os.path.join(PASTA, "franka_motor.png"))
        print("[missao] Franka:", api.estado("ridgeback_franka")["garra_pos"], flush=True)
        estado["franka"] += 1
    return estado["franka"] > len(PONTOS_FRANKA) + 1
