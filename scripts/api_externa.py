"""Interface para controlar a simulação por código: API Python, servidor TCP/JSON, gravação de vídeo e log CSV.

Usado pelo inspecao_aviao.py. Veja o DESENVOLVIMENTO.md para exemplos.

  * API ............ métodos para comandar/consultar os robôs (usados pelas "missões" e pelo servidor TCP)
  * ServidorTCP .... recebe comandos JSON (um por linha) de qualquer programa externo, em qualquer linguagem
  * GravadorVideo .. grava um MP4 de uma câmera da cena
  * LogCSV ......... grava as poses de todos os robôs num CSV
"""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import os
import socket

import numpy as np


class API:
    """Comandos de alto nível. Todos os nomes de robô são os da lista `robos()` (ex.: "drone_arl")."""

    def __init__(self, ctx):
        # ctx: objeto do inspecao_aviao.py com robots, select(i), set_camera_mode(m), sim_time(), find_link(), stage
        self.ctx = ctx
        self.ext = {}        # comandos de velocidade externos: nome -> dict(base, vz, ee, ate)
        self.alvos = {}      # navegação: nome -> (x, y, z|None, vel, tol)
        self.fotos = []      # fotos pendentes
        self._ee = {}

    # ------------------------------------------------------------------ consulta
    def _robo(self, nome):
        for r in self.ctx.robots:
            if r.s.nome == nome:
                return r
        raise KeyError(f"robô desconhecido: {nome!r}. Use robos() para ver a lista.")

    def robos(self):
        return [{"nome": r.s.nome, "tipo": r.s.tipo, "descricao": r.s.rotulo,
                 "juntas": r.joint_names_for_ui(), "camera": r.s.cam_path} for r in self.ctx.robots]

    def tempo(self):
        return self.ctx.sim_time()

    def estado(self, nome):
        r = self._robo(nome)
        p, q = r.pose()
        roll, pitch, _ = _rpy(q)
        out = {"nome": nome, "pos": [float(v) for v in p], "rumo_graus": math.degrees(r.heading()),
               "roll_graus": math.degrees(roll), "pitch_graus": math.degrees(pitch),
               "navegando": nome in self.alvos}
        if r.dofs:
            qs = r.art.get_dof_positions().numpy()[0]
            out["juntas"] = {r.dofs[i]: float(qs[i]) for i in r.jlist}
        if r.s.ee:
            if nome not in self._ee:
                from isaacsim.core.experimental.prims import XformPrim

                self._ee[nome] = XformPrim(str(self.ctx.find_link(r.s.path, r.s.ee)))
            out["garra_pos"] = [float(v) for v in self._ee[nome].get_world_poses()[0].numpy()[0]]
        return out

    # ------------------------------------------------------------------ movimento
    def velocidade(self, nome, vx=0.0, vy=0.0, wz=0.0, vz=0.0, duracao=0.5):
        """Velocidade no referencial do robô (vx frente, vy esquerda, m/s; wz giro, rad/s; vz subida, só drones).
        Vale por `duracao` segundos de simulação; reenvie periodicamente para manter."""
        self._robo(nome)
        e = self.ext.get(nome, {})
        e.update(base=(float(vx), float(vy), float(wz)), vz=float(vz), ate=self.tempo() + float(duracao))
        e.setdefault("ee", (0.0, 0.0, 0.0))
        self.ext[nome] = e

    def garra_velocidade(self, nome, vx=0.0, vy=0.0, vz=0.0, duracao=0.5):
        """Move a garra (Ridgeback+Franka/UR5) em m/s no referencial da base, por IK diferencial."""
        e = self.ext.get(nome, {"base": (0.0, 0.0, 0.0), "vz": 0.0})
        e.update(ee=(float(vx), float(vy), float(vz)), ate=self.tempo() + float(duracao))
        self.ext[nome] = e

    def parar(self, nome):
        self.ext.pop(nome, None)
        self.alvos.pop(nome, None)

    def ir_para(self, nome, x, y, z=None, vel=1.0, tol=0.15):
        """Navega até (x, y[, z]) no mundo; o controle é feito a cada quadro. Veja estado(nome)["navegando"]."""
        self._robo(nome)
        self.alvos[nome] = (float(x), float(y), None if z is None else float(z), float(vel), float(tol))

    def girar_para(self, nome, rumo_graus, wmax=0.8):
        """Gira no lugar até o rumo (graus, 0 = +X do mundo). Retorna True quando chegou."""
        r = self._robo(nome)
        err = math.atan2(math.sin(math.radians(rumo_graus) - r.heading()), math.cos(math.radians(rumo_graus) - r.heading()))
        if abs(err) < math.radians(3):
            self.velocidade(nome, duracao=0.05)
            return True
        self.velocidade(nome, wz=max(-wmax, min(wmax, 2.0 * err)), duracao=0.2)
        return False

    # ------------------------------------------------------------------ braço / juntas
    def juntas(self, nome, alvos: dict):
        """Define alvos de posição (rad) por nome de junta, ex.: {"panda_joint1": 0.5}."""
        r = self._robo(nome)
        idx = []
        for j, v in alvos.items():
            i = r.dofs.index(j)
            if i not in r.jlist:
                raise ValueError(f"a junta {j} não é comandável neste robô")
            r.q_target[i] = float(np.clip(v, r.lo[i], r.hi[i]))
            idx.append(i)
        if r.s.tipo != "spot":  # no Spot o controlador escreve o braço a cada passo
            r.art.set_dof_position_targets(r.q_target[None, idx], dof_indices=idx)

    def pose_braco(self, nome, pose="inspecao"):
        """'inspecao' (garra para cima) ou 'inicial'. Só Ridgeback+Franka / Ridgeback+UR5."""
        r = self._robo(nome)
        alvo = r.s.pose_inspecao if pose == "inspecao" else r.q_home[r.arm_idx]
        self.juntas(nome, {r.dofs[i]: v for i, v in zip(r.arm_idx, alvo)})

    def garra(self, nome, abrir=True):
        r = self._robo(nome)
        if not getattr(r, "grip_idx", None):
            raise ValueError(f"{nome} não tem garra")
        r.grip_open = bool(abrir)
        self.juntas(nome, {r.dofs[i]: 0.04 if abrir else 0.0 for i in r.grip_idx})

    def resetar(self, nome):
        self.parar(nome)
        self._robo(nome).reset()

    # ------------------------------------------------------------------ câmeras / fotos
    def selecionar(self, nome):
        self.ctx.select([r.s.nome for r in self.ctx.robots].index(nome))

    def camera(self, modo):
        """0 = panorâmica, 1 = seguir robô ativo, 2 = câmera de bordo do robô ativo."""
        self.ctx.set_camera_mode(int(modo))

    def foto(self, alvo, arquivo, largura=1280, altura=720, ao_terminar=None):
        """Salva um PNG da câmera de bordo de um robô (alvo = nome) ou de qualquer câmera (alvo = caminho USD).
        A imagem fica pronta alguns quadros depois; `ao_terminar(caminho)` é chamado quando salvar."""
        import omni.replicator.core as rep

        cam = alvo if str(alvo).startswith("/") else self._robo(alvo).s.cam_path
        if not cam:
            raise ValueError(f"{alvo} não tem câmera de bordo")
        rp = rep.create.render_product(cam, (int(largura), int(altura)))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        self.fotos.append({"rp": rp, "ann": ann, "arquivo": os.path.abspath(arquivo), "quadros": 0,
                           "cb": ao_terminar})

    # ------------------------------------------------------------------ chamado pela simulação
    def comando_para(self, nome, padrao):
        """Mescla o comando externo (se houver e não expirou) com o comando do teclado/ocioso."""
        e = self.ext.get(nome)
        if e is None:
            return padrao
        if e["ate"] < self.tempo():
            del self.ext[nome]
            return padrao
        c = dict(padrao)
        c["base"], c["vz"], c["ee"] = e["base"], e["vz"], e["ee"]
        return c

    def atualizar(self):
        """Uma vez por quadro: navegação (ir_para) e fotos pendentes."""
        for nome, (x, y, z, vel, tol) in list(self.alvos.items()):
            r = self._robo(nome)
            p, _ = r.pose()
            ex, ey = x - p[0], y - p[1]
            ez = 0.0 if z is None else z - p[2]
            dist = math.hypot(ex, ey)
            if dist < tol and abs(ez) < tol:
                del self.alvos[nome]
                self.velocidade(nome, duracao=0.05)
                continue
            h = r.heading()
            fx, fy = math.cos(h) * ex + math.sin(h) * ey, -math.sin(h) * ex + math.cos(h) * ey
            k = min(vel, 1.2 * dist) / max(dist, 1e-6)  # velocidade constante, desacelera perto do alvo
            if r.s.tipo == "spot":  # o Spot anda melhor virando para o alvo e indo para a frente
                ang = math.atan2(fy, fx)
                self.velocidade(nome, vx=min(vel, dist) if abs(ang) < 0.5 else 0.0, wz=max(-0.8, min(0.8, 1.5 * ang)),
                                duracao=0.2)
            else:
                self.velocidade(nome, vx=fx * k, vy=fy * k, vz=max(-vel, min(vel, 1.5 * ez)), duracao=0.2)
        for f in list(self.fotos):
            f["quadros"] += 1
            if f["quadros"] < 8:
                continue
            img = f["ann"].get_data()
            if img is None or not np.asarray(img).size:
                continue
            from PIL import Image

            os.makedirs(os.path.dirname(f["arquivo"]), exist_ok=True)
            Image.fromarray(np.asarray(img)[:, :, :3]).save(f["arquivo"])
            f["ann"].detach([f["rp"]])
            f["rp"].destroy()
            self.fotos.remove(f)
            if f["cb"]:
                f["cb"](f["arquivo"])


def _rpy(q):
    w, x, y, z = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class ServidorTCP:
    """Servidor de comandos: cada linha recebida é um JSON {"cmd": "<método da API>", ...argumentos}.

    Resposta (uma linha JSON): {"ok": true, "resultado": ...} ou {"ok": false, "erro": "..."}.
    Exemplo:  {"cmd": "velocidade", "nome": "drone_arl", "vz": 1.0, "duracao": 2}
    """

    COMANDOS = {"robos", "tempo", "estado", "velocidade", "garra_velocidade", "parar", "ir_para", "juntas",
                "pose_braco", "garra", "resetar", "selecionar", "camera", "foto"}

    def __init__(self, api: API, porta=5555, host="127.0.0.1"):
        self.api = api
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, porta))
        self.sock.listen(8)
        self.sock.setblocking(False)
        self.clientes = {}  # socket -> buffer
        print(f"[servidor] escutando em {host}:{porta} (JSON, um comando por linha)", flush=True)

    def _responder(self, c, msg):
        try:
            c.sendall((json.dumps(msg, ensure_ascii=False) + "\n").encode())
        except OSError:
            pass

    def poll(self):
        try:
            while True:
                c, _ = self.sock.accept()
                c.setblocking(False)
                self.clientes[c] = b""
        except BlockingIOError:
            pass
        for c in list(self.clientes):
            try:
                data = c.recv(65536)
                if not data:
                    raise ConnectionResetError
                self.clientes[c] += data
            except BlockingIOError:
                pass
            except OSError:
                self.clientes.pop(c, None)
                c.close()
                continue
            while b"\n" in self.clientes.get(c, b""):
                linha, self.clientes[c] = self.clientes[c].split(b"\n", 1)
                if linha.strip():
                    self._executar(c, linha)

    def _executar(self, c, linha):
        try:
            msg = json.loads(linha)
            cmd = msg.pop("cmd")
            if cmd not in self.COMANDOS:
                raise ValueError(f"comando desconhecido {cmd!r}; disponíveis: {sorted(self.COMANDOS)}")
            if cmd == "foto":  # resposta só quando o arquivo estiver salvo
                self.api.foto(ao_terminar=lambda p: self._responder(c, {"ok": True, "resultado": p}), **msg)
                return
            self._responder(c, {"ok": True, "resultado": getattr(self.api, cmd)(**msg)})
        except Exception as e:  # noqa: BLE001
            self._responder(c, {"ok": False, "erro": f"{type(e).__name__}: {e}"})

    def fechar(self):
        for c in self.clientes:
            c.close()
        self.sock.close()


class GravadorVideo:
    """Grava os quadros renderizados de uma câmera num MP4 (H.264, via imageio-ffmpeg do ambiente do Isaac Sim)."""

    def __init__(self, camera_path, arquivo, fps=30, largura=1280, altura=720):
        import imageio_ffmpeg
        import omni.replicator.core as rep

        self.arquivo = os.path.abspath(arquivo)
        os.makedirs(os.path.dirname(self.arquivo), exist_ok=True)
        self.rp = rep.create.render_product(camera_path, (largura, altura))
        self.ann = rep.AnnotatorRegistry.get_annotator("rgb")
        self.ann.attach([self.rp])
        self.writer = imageio_ffmpeg.write_frames(self.arquivo, (largura, altura), fps=fps, quality=8,
                                                  macro_block_size=1)
        self.writer.send(None)
        self.fps = fps
        self.n = 0
        print(f"[video] gravando {camera_path} -> {self.arquivo}", flush=True)

    def quadro(self, t):
        """`t` = tempo de simulação (s). Grava só os quadros necessários para o vídeo ter a duração real."""
        faltam = int(t * self.fps) + 1 - self.n
        if faltam <= 0:
            return
        img = self.ann.get_data()
        if img is None or not np.asarray(img).size:
            return
        img = np.ascontiguousarray(np.asarray(img)[:, :, :3])
        for _ in range(faltam):
            self.writer.send(img)
        self.n += faltam

    def fechar(self):
        self.writer.close()
        self.ann.detach([self.rp])
        self.rp.destroy()
        print(f"[video] {self.n} quadros salvos em {self.arquivo}", flush=True)


class LogCSV:
    """Grava tempo, posição e atitude de todos os robôs a cada `periodo` segundos de simulação."""

    def __init__(self, api: API, arquivo, periodo=0.1):
        self.api, self.periodo, self.ultimo = api, periodo, -1e9
        self.arquivo = os.path.abspath(arquivo)
        os.makedirs(os.path.dirname(self.arquivo), exist_ok=True)
        self.f = open(self.arquivo, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["tempo_s", "robo", "x", "y", "z", "rumo_graus", "roll_graus", "pitch_graus"])

    def quadro(self):
        t = self.api.tempo()
        if t - self.ultimo < self.periodo:
            return
        self.ultimo = t
        for r in self.api.ctx.robots:
            e = self.api.estado(r.s.nome)
            self.w.writerow([f"{t:.3f}", r.s.nome, *(f"{v:.4f}" for v in e["pos"]), f"{e['rumo_graus']:.2f}",
                             f"{e['roll_graus']:.2f}", f"{e['pitch_graus']:.2f}"])

    def fechar(self):
        self.f.close()
        print(f"[log] poses salvas em {self.arquivo}", flush=True)


def carregar_missao(caminho):
    """Carrega um arquivo Python com `passo(api, t)` (chamado a cada quadro) e, opcionalmente, `inicio(api)`.
    Se `passo` retornar True, a missão terminou."""
    spec = importlib.util.spec_from_file_location("missao_usuario", os.path.abspath(caminho))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, "passo"):
        raise AttributeError(f"{caminho} precisa ter uma função passo(api, t)")
    return mod
