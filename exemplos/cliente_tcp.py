"""Exemplo de controle EXTERNO pela rede (TCP + JSON). Não depende do Isaac Sim: roda com qualquer Python 3.

1) Em um terminal, abra a simulação com o servidor:
       ./executar.sh --servidor            (porta 5555)
2) Em outro terminal:
       python3 exemplos/cliente_tcp.py

Protocolo: cada mensagem é UMA linha JSON {"cmd": "<comando>", ...argumentos}; a resposta é uma linha JSON
{"ok": true, "resultado": ...} ou {"ok": false, "erro": "..."}. Dá para usar de C++, MATLAB, Node, ROS etc.
"""
import json
import socket
import time


class Simulacao:
    def __init__(self, host="127.0.0.1", porta=5555):
        self.s = socket.create_connection((host, porta), timeout=60)
        self.buf = b""

    def __getattr__(self, cmd):  # sim.estado(nome="drone_arl") -> {"cmd": "estado", "nome": "drone_arl"}
        def chamar(**kw):
            self.s.sendall((json.dumps({"cmd": cmd, **kw}) + "\n").encode())
            while b"\n" not in self.buf:
                self.buf += self.s.recv(65536)
            linha, self.buf = self.buf.split(b"\n", 1)
            r = json.loads(linha)
            if not r["ok"]:
                raise RuntimeError(r["erro"])
            return r["resultado"]
        return chamar


def esperar_chegar(sim, nome):
    while sim.estado(nome=nome)["navegando"]:
        time.sleep(0.2)


def esperar(sim, segundos):
    """Espera `segundos` de TEMPO DE SIMULAÇÃO. A simulação pode rodar mais devagar que o relógio (depende do PC),
    e as durações dos comandos (`duracao`) são contadas no tempo da simulação."""
    fim = sim.tempo() + segundos
    while sim.tempo() < fim:
        time.sleep(0.05)


if __name__ == "__main__":
    sim = Simulacao()
    print("Robôs:", [r["nome"] for r in sim.robos()])

    # 1) drone Ingenuity: sobe ~3 m com comando de velocidade (1 m/s por 3 s de simulação, reenviado sempre)
    sim.selecionar(nome="drone_ingenuity")
    sim.camera(modo=1)
    fim = sim.tempo() + 3.0
    while sim.tempo() < fim:
        sim.velocidade(nome="drone_ingenuity", vz=1.0, duracao=0.3)
        time.sleep(0.05)
    esperar(sim, 0.5)  # para de subir e estabiliza
    print("Ingenuity:", [round(v, 2) for v in sim.estado(nome="drone_ingenuity")["pos"]])

    # 2) drone ARL: navegação por ponto (o controle roda dentro da simulação)
    sim.ir_para(nome="drone_arl", x=17, y=-6.5, z=7.0, vel=2.0)
    esperar_chegar(sim, "drone_arl")
    print("Foto salva em:", sim.foto(alvo="drone_arl", arquivo="saidas/fotos/cliente_tcp_drone.png"))

    # 3) braço do UR5: juntas diretamente (radianos)
    sim.juntas(nome="ridgeback_ur5", alvos={"ur_arm_shoulder_lift_joint": -1.0, "ur_arm_elbow_joint": 0.8})
    esperar(sim, 2.0)
    print("UR5 garra em:", [round(v, 2) for v in sim.estado(nome="ridgeback_ur5")["garra_pos"]])

    # 4) Spot anda 2 m para a frente
    sim.velocidade(nome="spot_braco", vx=1.0, duracao=2.0)
    esperar(sim, 3.0)
    print("Spot:", [round(v, 2) for v in sim.estado(nome="spot_braco")["pos"]])
