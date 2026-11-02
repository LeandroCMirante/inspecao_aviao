"""Simulação de inspeção de aeronave no Isaac Sim 6.1: robôs terrestres com manipulador + drones.

Cena: pátio (apron) com quatro aviões estáticos lado a lado (NASA SOFIA / Boeing 747SP, Global Hawk, X-57 e
Space Shuttle), em fila ao lado do 747: 4 robôs terrestres (Ridgeback+Franka, Ridgeback+UR5, Spot com braço, Galbot G1) e 4 drones
(ARL-Robot-1, Crazyflie, Quadcopter, Ingenuity). Todos são controláveis pelo teclado (um de cada vez).

Exemplos (com o ambiente do Isaac Sim ativado, a partir da pasta inspecao_aviao/):
    python scripts/inspecao_aviao.py                     # cena completa, com janela
    python scripts/inspecao_aviao.py --cena leve         # só 747 + Ridgeback/Franka + drone ARL (PC mais fraco)
    python scripts/inspecao_aviao.py --salvar cenas/cena_inspecao_aviao.usda   # grava a cena em USD
    python scripts/inspecao_aviao.py --headless --capturas imagens      # gera as imagens da pasta imagens/

Teclas (ver também a janela "Controles" e o GUIA_ISAAC_SIM.md):
    TAB / 1..8   troca o robô ativo           C   troca a câmera (panorâmica / seguir / câmera do robô)
    Setas        anda (frente/trás/lados)      Q/E gira          R/V (ou PageUp/PageDown) sobe/desce (drones)
    I/K J/L U/O  move a garra (X / Y / Z)      G   abre/fecha a garra         H   volta o braço à pose inicial
    P            braço na pose de inspeção (garra apontando para cima, para olhar motor/asa)
    [ / ]        escolhe uma junta             - / =  move a junta escolhida (modo junta, serve para todo robô)
    SHIFT        segurado = mais rápido        BACKSPACE  reposiciona o robô ativo
"""
from __future__ import annotations

import argparse
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--cena", choices=["completa", "leve"], default="completa")
parser.add_argument("--headless", action="store_true", help="sem janela (para servidores / gerar imagens)")
parser.add_argument("--sem-spot-ia", action="store_true", help="Spot fica parado (sem a política de locomoção)")
parser.add_argument("--salvar", metavar="ARQUIVO.usd", help="grava a cena montada em um arquivo USD")
parser.add_argument("--capturas", metavar="PASTA", help="renderiza imagens do ambiente nessa pasta e sai")
parser.add_argument("--frames", type=int, default=0, help="roda N quadros e fecha (0 = até fechar a janela)")
parser.add_argument("--robos", metavar="A,B,...", help="carrega só estes robôs (nomes da lista abaixo)")
parser.add_argument("--autoteste", action="store_true", help="simula teclas em cada robô e mostra se ele se moveu")
g = parser.add_argument_group("controle por código e gravação (ver DESENVOLVIMENTO.md)")
g.add_argument("--servidor", type=int, nargs="?", const=5555, metavar="PORTA",
               help="abre um servidor TCP/JSON para controlar os robôs de outro programa (padrão 5555)")
g.add_argument("--missao", metavar="ARQUIVO.py", help="roda um script de missão com passo(api, t) a cada quadro")
g.add_argument("--gravar-video", metavar="ARQUIVO.mp4", help="grava um vídeo MP4 da simulação")
g.add_argument("--camera-video", choices=["panoramica", "seguir", "robo"], default="seguir",
               help="câmera usada no vídeo (robo = câmera de bordo do robô ativo no início)")
g.add_argument("--fps-video", type=int, default=30)
g.add_argument("--log-csv", metavar="ARQUIVO.csv", help="grava as poses de todos os robôs (10 Hz) num CSV")
g.add_argument("--ativo", metavar="NOME", help="robô ativo no início (ex.: drone_arl)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": args.headless, "width": 1600, "height": 900}
)

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.appwindow  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.experimental.prims import Articulation, XformPrim  # noqa: E402
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402

PHYSICS_DT = 1.0 / 500.0  # a política de locomoção do Spot foi treinada a 500 Hz (decimação 10 -> 50 Hz)
ASSETS = get_assets_root_path()
if not ASSETS:
    carb.log_error("Não encontrei os assets do Isaac Sim (precisa de internet na 1ª execução).")
    simulation_app.close()
    sys.exit(1)
ROBOTS_URL = ASSETS + "/Isaac/Robots/"
NVIDIA = ASSETS.split("/Assets/")[0] + "/Assets/"
AVIOES_USD = os.path.join(ROOT, "assets", "avioes", "usd")


# ---------------------------------------------------------------------------------------------------------------------
# Definição dos robôs
# ---------------------------------------------------------------------------------------------------------------------
class Spec:
    def __init__(self, nome, rotulo, usd, tipo, pos, yaw=0.0, escala=1.0, ee=None, braco=None, garra=None,
                 pose_inspecao=None, cam_link=None, cam_pos=(0, 0, 0), cam_eixo="x", rotores=(), seguir=5.0,
                 leve=False):
        self.nome, self.rotulo, self.usd, self.tipo = nome, rotulo, usd, tipo
        self.pos, self.yaw, self.escala = tuple(pos), yaw, escala
        self.ee, self.braco, self.garra = ee, braco or [], garra or []
        self.pose_inspecao = pose_inspecao
        self.cam_link, self.cam_pos, self.cam_eixo = cam_link, cam_pos, cam_eixo
        self.rotores, self.seguir, self.leve = rotores, seguir, leve
        self.path = f"/World/Robos/{nome}"


FRANKA = [f"panda_joint{i}" for i in range(1, 8)]
UR5 = ["ur_arm_shoulder_pan_joint", "ur_arm_shoulder_lift_joint", "ur_arm_elbow_joint",
       "ur_arm_wrist_1_joint", "ur_arm_wrist_2_joint", "ur_arm_wrist_3_joint"]
SPECS = [
    Spec("ridgeback_franka", "Ridgeback + Franka (terrestre c/ manipulador)",
         "Clearpath/RidgebackFranka/ridgeback_franka.usd", "ridgeback", (30.0, -6.0, 0.0), yaw=90,
         ee="endeffector", braco=FRANKA, garra=["panda_finger_joint1", "panda_finger_joint2"],
         pose_inspecao=[0.0, 0.0, 0.0, -0.2, 2.9, 3.0, 0.785],
         cam_link="panda_hand", cam_pos=(0.07, 0, 0.04), cam_eixo="z", leve=True),
    Spec("ridgeback_ur5", "Ridgeback + UR5 (terrestre c/ manipulador)",
         "Clearpath/RidgebackUr/ridgeback_ur5.usd", "ridgeback", (27.0, -6.0, 0.0), yaw=90,
         ee="ur_arm_wrist_3_link", braco=UR5, pose_inspecao=[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
         cam_link="ur_arm_wrist_3_link", cam_pos=(0, 0.12, 0), cam_eixo="y"),
    Spec("spot_braco", "Boston Dynamics Spot com braço (quadrúpede)",
         "BostonDynamics/spot/spot_with_arm.usd", "spot", (24.0, -6.0, 0.8), yaw=0,
         cam_link="base", cam_pos=(0.45, 0, 0.05), seguir=4.0),
    Spec("galbot_g1", "Galbot G1 (humanoide de rodas, dois braços)",
         "Galbot/galbot_g1/galbot_g1.usda", "galbot", (20.5, -6.0, 0.0), yaw=90, seguir=5.0),
    Spec("drone_arl", "Drone ARL-Robot-1 (NTNU) - drone de inspeção",
         "NTNU/ARL-Robot-1/arl_robot_1.usd", "drone", (17.0, -6.0, 0.4), yaw=90, cam_link="base_link",
         cam_pos=(0.28, 0, 0), seguir=3.0, leve=True),
    Spec("drone_crazyflie", "Drone Crazyflie 2.x (Bitcraze, 9 cm)",
         "Bitcraze/Crazyflie/cf2x.usd", "drone", (15.5, -6.0, 0.3), yaw=90, cam_link="body", cam_pos=(0.04, 0, 0),
         rotores=("m1_joint", "m2_joint", "m3_joint", "m4_joint"), seguir=0.8),
    Spec("drone_quadcopter", "Drone Quadcopter (modelo genérico do Isaac Sim)",
         "IsaacSim/Quadcopter/quadcopter.usd", "drone", (14.0, -6.0, 0.6), yaw=90, escala=0.05, cam_link="chassis",
         cam_pos=(6.0, 0, 0), seguir=3.0),
    Spec("drone_ingenuity", "Helicóptero Ingenuity (NASA, coaxial)",
         "NASA/Ingenuity/ingenuity.usd", "drone", (12.0, -6.0, 0.3), yaw=90, cam_link="chassis", cam_pos=(0.1, 0, 0.1),
         rotores=("rotor_visual_0", "rotor_visual_1"), seguir=3.0),
]
if args.cena == "leve":
    SPECS = [s for s in SPECS if s.leve]
if args.robos:
    SPECS = [s for s in SPECS if s.nome in args.robos.split(",")]


# ---------------------------------------------------------------------------------------------------------------------
# Utilitários de cena
# ---------------------------------------------------------------------------------------------------------------------
def _precision(prim, name):
    """Usa a mesma precisão (float/double) de um xformOp que o asset referenciado já tenha declarado."""
    attr = prim.GetAttribute(name)
    if attr and attr.GetTypeName() in (Sdf.ValueTypeNames.Double3, Sdf.ValueTypeNames.Double):
        return UsdGeom.XformOp.PrecisionDouble
    if attr and attr.GetTypeName() in (Sdf.ValueTypeNames.Float3, Sdf.ValueTypeNames.Float):
        return UsdGeom.XformOp.PrecisionFloat
    return None


def place(prim, pos, yaw_deg=0.0, scale=None):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    p = _precision(prim, "xformOp:translate") or UsdGeom.XformOp.PrecisionDouble
    xf.AddTranslateOp(p).Set(Gf.Vec3d(*pos) if p == UsdGeom.XformOp.PrecisionDouble else Gf.Vec3f(*pos))
    p = _precision(prim, "xformOp:rotateZ") or UsdGeom.XformOp.PrecisionFloat
    xf.AddRotateZOp(p).Set(float(yaw_deg))
    if scale is not None:
        p = _precision(prim, "xformOp:scale") or UsdGeom.XformOp.PrecisionFloat
        xf.AddScaleOp(p).Set(Gf.Vec3d(scale, scale, scale) if p == UsdGeom.XformOp.PrecisionDouble
                             else Gf.Vec3f(scale, scale, scale))


def preview_material(stage, path, color, roughness=0.6):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def quad_mesh(stage, path, x0, x1, y0, y1, z, color, collide=False):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateNormalsAttr([(0, 0, 1)] * 4)
    m.CreateExtentAttr([(x0, y0, z), (x1, y1, z)])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(preview_material(stage, path + "_mat", color))
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("none")
    return m


def build_environment(stage):
    UsdGeom.Xform.Define(stage, "/World/Ambiente")
    # céu (HDR) + sol
    dome = UsdLux.DomeLight.Define(stage, "/World/Ambiente/Ceu")
    dome.CreateTextureFileAttr(NVIDIA + "Skies/Clear/noon_grass_4k.hdr")
    dome.CreateTextureFormatAttr("latlong")
    dome.CreateIntensityAttr(700.0)
    sol = UsdLux.DistantLight.Define(stage, "/World/Ambiente/Sol")
    sol.CreateIntensityAttr(1600.0)
    sol.CreateAngleAttr(0.8)
    UsdGeom.Xformable(sol).AddRotateXYZOp().Set(Gf.Vec3f(35, 25, 40))
    # pátio: chão de concreto 400 x 400 m com colisão + faixa amarela de taxi
    quad_mesh(stage, "/World/Ambiente/Patio", -200, 200, -200, 200, 0.0, (0.16, 0.16, 0.16))
    # colisão do chão: plano infinito do PhysX (mais robusto que uma malha fina)
    plano = UsdGeom.Plane.Define(stage, "/World/Ambiente/ChaoColisao")
    plano.CreateAxisAttr("Z")
    plano.CreateWidthAttr(400.0)
    plano.CreateLengthAttr(400.0)
    plano.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(plano.GetPrim())
    quad_mesh(stage, "/World/Ambiente/FaixaTaxi", -80, 80, -0.15, 0.15, 0.005, (0.95, 0.75, 0.1))
    quad_mesh(stage, "/World/Ambiente/FaixaParada", 33.0, 33.3, -6, 6, 0.005, (0.95, 0.75, 0.1))
    # material de atrito para o chão
    pm = UsdShade.Material.Define(stage, "/World/Ambiente/AtritoChao")
    api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    api.CreateStaticFrictionAttr(1.0)
    api.CreateDynamicFrictionAttr(1.0)
    api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath("/World/Ambiente/ChaoColisao")).Bind(
        pm, UsdShade.Tokens.weakerThanDescendants, "physics")


def build_aircraft(stage):
    """Aviões lado a lado, todos com o nariz para +X; o 747 fica na origem e os outros em fila no lado -Y."""
    UsdGeom.Xform.Define(stage, "/World/Avioes")
    layout = [("sofia_747sp", (0.0, 0.0, 0.0), 0.0)]
    if args.cena == "completa":
        layout += [("global_hawk", (0.0, -55.0, 0.0), 0.0),      # envergadura 35,4 m
                   ("x57_maxwell", (0.0, -84.0, 0.0), 0.0),      # envergadura 9,6 m
                   ("space_shuttle", (-14.0, -106.0, 0.0), 0.0)]  # largura 26,5 m
    for key, pos, yaw in layout:
        prim = stage_utils.add_reference_to_stage(os.path.join(AVIOES_USD, key, key + ".usd"), f"/World/Avioes/{key}")
        place(prim, pos, yaw)


def fix_spot_articulation_root(stage, root_path):
    """No spot_with_arm.usd a ArticulationRootAPI fica no prim de topo e o PhysX acaba usando o link do ombro do
    braço como raiz. A política de locomoção lê velocidade/orientação da raiz, então movemos a raiz para o corpo."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    base = stage.GetPrimAtPath(root_path + "/base")
    UsdPhysics.ArticulationRootAPI.Apply(base)
    PhysxSchema.PhysxArticulationAPI.Apply(base)


def lighten_spot_arm(stage, root_path, massa=0.05):
    """A política de locomoção da NVIDIA foi treinada no Spot SEM braço. Com os 7,5 kg do braço ele cai; deixando
    os links do braço quase sem massa/inércia a marcha fica estável e o braço continua controlável."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if prim.GetName().startswith("arm0_link") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            mapi = UsdPhysics.MassAPI.Apply(prim)
            m0 = mapi.GetMassAttr().Get() or 0.0
            i0 = mapi.GetDiagonalInertiaAttr().Get()
            mapi.CreateMassAttr(massa)
            if i0 is not None and m0 > 0 and any(i0):
                mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*(float(v) * massa / m0 for v in i0)))


def disable_gravity(stage, root_path):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)


def add_camera(stage, path, pos, eixo="x", focal=18.0):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 2000.0))
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    # câmera USD olha para -Z com Y para cima; giramos para ela olhar ao longo do eixo +X, +Y ou +Z do link
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*{"x": (90, 0, -90), "y": (-90, 180, 0), "z": (180, 0, 0)}[eixo]))
    return cam


def look_at_matrix(eye, target):
    eye, target = Gf.Vec3d(*eye), Gf.Vec3d(*target)
    m = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1))
    return m.GetInverse()


def set_camera_pose(stage, path, eye, target):
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    if not ops or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform:
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
    else:
        op = ops[0]
    op.Set(look_at_matrix(eye, target))


# ---------------------------------------------------------------------------------------------------------------------
# Montagem da cena
# ---------------------------------------------------------------------------------------------------------------------
stage_utils.create_new_stage()
stage = omni.usd.get_context().get_stage()
stage_utils.set_stage_up_axis("Z")
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(int(round(1 / PHYSICS_DT)))

build_environment(stage)
build_aircraft(stage)
UsdGeom.Xform.Define(stage, "/World/Robos")
UsdGeom.Xform.Define(stage, "/World/Cameras")

spot_runner = None
for s in SPECS:
    if s.tipo == "spot" and not args.sem_spot_ia:
        from isaacsim.robot.policy.examples import RobotPolicyRunner, get_spot_spec

        stage_utils.add_reference_to_stage(ROBOTS_URL + s.usd, s.path)
        fix_spot_articulation_root(stage, s.path)
        spot_runner = RobotPolicyRunner(get_spot_spec(usd_path=ROBOTS_URL + s.usd), prim_path=s.path,
                                        position=list(s.pos))  # orientação padrão (o runner fixa a identidade)
        spot_runner.spawn()
        lighten_spot_arm(stage, s.path)
        # sem autocolisão: as pernas dianteiras batiam no braço durante o trote/giro
        PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath(s.path + "/base")).CreateEnabledSelfCollisionsAttr(False)
    else:
        prim = stage_utils.add_reference_to_stage(ROBOTS_URL + s.usd, s.path)
        place(prim, s.pos, s.yaw, s.escala if s.escala != 1.0 else None)
    if s.tipo == "drone":
        disable_gravity(stage, s.path)
    if s.tipo == "ridgeback":
        # a base anda por juntas virtuais; os cilindros de colisão das rodas do UR5 ficam 2,6 cm abaixo do chão e
        # travariam o robô, então desligamos a colisão das rodas
        for prim in Usd.PrimRange(stage.GetPrimAtPath(s.path)):
            if prim.IsA(UsdGeom.Cylinder) and prim.HasAPI(UsdPhysics.CollisionAPI) and "base_link" in str(prim.GetPath()):
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)

# deixa os assets da nuvem carregarem antes de mexer nos filhos
for _ in range(30):
    simulation_app.update()
# o Galbot traz um controlador OmniGraph próprio (ROS/holonômico) que não usamos: desativamos para não gerar erros
for prim in list(Usd.PrimRange(stage.GetPrimAtPath("/World/Robos"))):
    if prim.IsValid() and prim.GetTypeName() == "OmniGraph":
        prim.SetActive(False)


def find_link(root, name):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.GetName() == name and (prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.IsA(UsdGeom.Xform)):
            return prim.GetPath()
    return None


for s in SPECS:
    s.cam_path = None
    if s.cam_link:
        link = find_link(s.path, s.cam_link)
        if link is not None:
            s.cam_path = str(link) + "/CameraInspecao"
            add_camera(stage, s.cam_path, s.cam_pos, s.cam_eixo)

UsdGeom.Camera.Define(stage, "/World/Cameras/Seguir").CreateFocalLengthAttr(20.0)
UsdGeom.Camera.Define(stage, "/World/Cameras/Panoramica").CreateFocalLengthAttr(16.0)
set_camera_pose(stage, "/World/Cameras/Panoramica", (48, -48, 22), (0, 0, 4))
UsdGeom.Camera(stage.GetPrimAtPath("/World/Cameras/Seguir")).CreateClippingRangeAttr(Gf.Vec2f(0.05, 3000))
UsdGeom.Camera(stage.GetPrimAtPath("/World/Cameras/Panoramica")).CreateClippingRangeAttr(Gf.Vec2f(0.1, 3000))

if args.salvar:
    out = os.path.abspath(args.salvar)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    stage.GetRootLayer().Export(out)
    if out.endswith(".usda"):  # caminhos relativos: a pasta pode ser copiada para outro computador
        txt = open(out, encoding="utf-8").read()
        rel = os.path.relpath(AVIOES_USD, os.path.dirname(out)).replace(os.sep, "/")
        open(out, "w", encoding="utf-8").write(txt.replace(AVIOES_USD.replace(os.sep, "/"), "./" + rel
                                                           if not rel.startswith(".") else rel))
    print(f"[inspecao] cena gravada em {out}", flush=True)


# ---------------------------------------------------------------------------------------------------------------------
# Controladores
# ---------------------------------------------------------------------------------------------------------------------
def quat_to_rpy(q):  # q = (w, x, y, z)
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def up_vector(q):  # eixo Z do corpo no mundo, q = (w, x, y, z)
    w, x, y, z = q
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


class Robot:
    """Estado/controle comum: modo junta (qualquer robô) e reposicionamento."""

    def __init__(self, spec: Spec):
        self.s = spec
        self.art = spot_runner.articulation if (spec.tipo == "spot" and spot_runner) else Articulation(spec.path)
        self.joint_sel = 0
        self.q_target = None

    def init_physics(self):
        self.dofs = list(self.art.dof_names)
        self.art.set_sleep_thresholds([0.0])  # nunca "dormir": senão comandos de velocidade não acordam o robô
        if self.dofs:
            lo, hi = self.art.get_dof_limits()
            self.lo, self.hi = lo.numpy()[0], hi.numpy()[0]
            self.q_target = self.art.get_dof_positions().numpy()[0].copy()
            self.q_home = self.q_target.copy()
        pos, quat = self.art.get_world_poses()
        self.home_pos, self.home_quat = pos.numpy()[0].copy(), quat.numpy()[0].copy()

    def pose(self):
        pos, quat = self.art.get_world_poses()
        return pos.numpy()[0], quat.numpy()[0]

    def heading(self):
        return quat_to_rpy(self.pose()[1])[2]

    jlist: list = []  # índices das juntas acessíveis no "modo junta"

    def joint_names_for_ui(self):
        return [self.dofs[i] for i in self.jlist]

    def joint_step(self, direction, dt, fast):
        if not self.jlist:
            return
        i = self.jlist[self.joint_sel % len(self.jlist)]
        self.q_target[i] = float(np.clip(self.q_target[i] + direction * (1.2 if fast else 0.5) * dt,
                                         max(self.lo[i], -1e3), min(self.hi[i], 1e3)))
        self.art.set_dof_position_targets(self.q_target[None, i:i + 1], dof_indices=[i])

    def reset(self):
        self.art.set_world_poses(self.home_pos[None], self.home_quat[None])
        self.art.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
        if self.dofs:
            self.q_target = self.q_home.copy()
            self.art.set_dof_positions(self.q_target[None])
            self.art.set_dof_position_targets(self.q_target[None])

    def control(self, cmd, dt):
        pass


class Ridgeback(Robot):
    """Base omnidirecional (juntas virtuais x, y, yaw) + braço por IK diferencial (mínimos quadrados amortecidos)."""

    def init_physics(self):
        super().init_physics()
        n = len(self.dofs)
        self.base_idx = [0, 1, 2]
        self.arm_idx = [self.dofs.index(j) for j in self.s.braco]
        self.grip_idx = [self.dofs.index(j) for j in self.s.garra]
        self.jlist = self.arm_idx + self.grip_idx
        self.base_link = XformPrim(str(find_link(self.s.path, "base_link")))
        kp, kd = self.art.get_dof_gains()
        kp, kd = kp.numpy()[0].copy(), kd.numpy()[0].copy()
        kp[self.base_idx], kd[self.base_idx] = 0.0, 2.0e5  # base: controle de velocidade
        for i in self.arm_idx:  # braço: controle de posição firme
            kp[i], kd[i] = 4.0e4 if "panda" in self.dofs[i] else 2.0e4, 2.0e3
        self.art.set_dof_gains(kp[None], kd[None])
        self.art.set_dof_max_efforts(np.full((1, n), 1.0e7))
        self.ee_link = list(self.art.link_names).index(self.s.ee)
        self.grip_open = True

    def pose(self):
        pos, quat = self.base_link.get_world_poses()
        return pos.numpy()[0], quat.numpy()[0]

    def heading(self):
        return math.radians(self.s.yaw) + float(self.art.get_dof_positions().numpy()[0][2])

    def control(self, cmd, dt):
        q = self.art.get_dof_positions().numpy()[0]
        yaw = q[2]  # as juntas virtuais x/y estão no referencial da raiz (já girada pelo yaw inicial)
        vx, vy, wz = cmd["base"]
        c, s = math.cos(yaw), math.sin(yaw)
        v = np.array([[c * vx - s * vy, s * vx + c * vy, wz]])
        self.art.set_dof_velocity_targets(v, dof_indices=self.base_idx)
        ee = np.array(cmd["ee"], float)
        if np.any(ee):
            # comando do efetuador no referencial da base (frente do robô = +X)
            ee_w = np.array([c * ee[0] - s * ee[1], s * ee[0] + c * ee[1], ee[2], 0, 0, 0])
            J = self.art.get_jacobian_matrices().numpy()[0][self.ee_link - 1][:, self.arm_idx]  # base fixa
            lam = 0.05
            dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), ee_w)
            self.q_target[self.arm_idx] = np.clip(self.q_target[self.arm_idx] + dq * dt,
                                                  self.lo[self.arm_idx], self.hi[self.arm_idx])
            self.art.set_dof_position_targets(self.q_target[None, self.arm_idx], dof_indices=self.arm_idx)
        if cmd["inspect_arm"] and self.s.pose_inspecao:
            self.q_target[self.arm_idx] = np.clip(self.s.pose_inspecao, self.lo[self.arm_idx], self.hi[self.arm_idx])
            self.art.set_dof_position_targets(self.q_target[None, self.arm_idx], dof_indices=self.arm_idx)
        if cmd["home_arm"]:
            self.q_target[self.arm_idx] = self.q_home[self.arm_idx]
            self.art.set_dof_position_targets(self.q_target[None, self.arm_idx], dof_indices=self.arm_idx)
        if cmd["toggle_grip"] and self.grip_idx:
            self.grip_open = not self.grip_open
            self.q_target[self.grip_idx] = 0.04 if self.grip_open else 0.0
            self.art.set_dof_position_targets(self.q_target[None, self.grip_idx], dof_indices=self.grip_idx)

    def reset(self):
        q = self.q_home.copy()
        self.q_target = q
        self.art.set_dof_positions(q[None])
        self.art.set_dof_position_targets(q[None])
        self.art.set_dof_velocity_targets(np.zeros((1, 3)), dof_indices=self.base_idx)


class Spot(Robot):
    """Locomoção pela política de RL oficial da NVIDIA (comando [vx, vy, wz]); braço no modo junta."""

    def init_physics(self):
        super().init_physics()
        self.ready = False
        self.failed = spot_runner is None
        self.leg = [i for i, n in enumerate(self.dofs) if not n.startswith("arm0")]
        self.arm = [i for i, n in enumerate(self.dofs) if n.startswith("arm0")]
        self.jlist = self.arm

    def physics(self, cmd, dt):
        if self.failed:
            return
        try:
            if self.ready:
                spot_runner.step(dt, list(cmd))
            else:
                spot_runner.restart_from_default_state(list(cmd))
                self.ready = True
                self.q_target = self.art.get_dof_positions().numpy()[0].copy()
                self.q_home = self.q_target.copy()
                # A política da NVIDIA foi treinada no Spot SEM braço: deixamos o braço quase sem massa e com
                # ganhos moderados para não desequilibrar a marcha (massa/inércia ajustadas em lighten_spot_arm).
                kp, kd = self.art.get_dof_gains()
                kp, kd = kp.numpy()[0].copy(), kd.numpy()[0].copy()
                kp[self.arm], kd[self.arm] = 300.0, 20.0
                self.art.set_dof_gains(kp[None], kd[None])

            if self.arm:  # segura o braço onde o usuário deixou
                self.art.set_dof_position_targets(self.q_target[None, self.arm], dof_indices=self.arm)
        except Exception as e:  # noqa: BLE001
            self.failed = True
            carb.log_error(f"Spot: política de locomoção falhou ({e}); o robô ficará parado.")

    def reset(self):
        super().reset()
        self.ready = False


class Galbot(Robot):
    """Galbot G1: base com rodas omnidirecionais movida cinematicamente (o asset traz um controlador OmniGraph
    próprio); braços, tronco e cabeça no modo junta."""

    def init_physics(self):
        super().init_physics()
        self.jlist = [i for i, n in enumerate(self.dofs) if not n.startswith(("wheel", "gripper"))]
        self.dyaw = 0.0  # o link raiz do Galbot não está alinhado com Z; giramos em torno do Z do mundo

    def heading(self):
        return math.radians(self.s.yaw) + self.dyaw

    def control(self, cmd, dt):
        vx, vy, wz = cmd["base"]
        if vx or vy or wz:
            pos, _ = self.pose()
            yaw = self.heading()
            c, s = math.cos(yaw), math.sin(yaw)
            pos = pos + np.array([c * vx - s * vy, s * vx + c * vy, 0.0]) * dt
            self.dyaw += wz * dt
            h = self.dyaw / 2
            qz = Gf.Quatd(math.cos(h), 0, 0, math.sin(h))
            w, x, y, z = self.home_quat
            q = qz * Gf.Quatd(float(w), float(x), float(y), float(z))
            qa = np.array([q.GetReal(), *q.GetImaginary()])
            self.art.set_world_poses(pos[None], qa[None])
            self.art.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))

    def reset(self):
        super().reset()
        self.dyaw = 0.0


class Drone(Robot):
    """Controlador de voo simplificado em modo "position hold" (como o modo Loiter/Position de uma controladora):
    o operador comanda velocidades; o controlador mantém altitude/posição/atitude nivelada. A gravidade do drone
    está desligada e o voo é feito por velocidades do corpo; colisões com o avião e o chão continuam valendo."""

    def init_physics(self):
        super().init_physics()
        self.target = self.home_pos.copy()
        self.target_yaw = quat_to_rpy(self.home_quat)[2]
        self.rot_idx = [self.dofs.index(r) for r in self.s.rotores if r in self.dofs]
        self.spin = 0.0

    def control(self, cmd, dt):
        pos, quat = self.pose()
        roll, pitch, yaw = quat_to_rpy(quat)
        vx, vy, wz = cmd["base"]
        vz = cmd["vz"]
        c, s = math.cos(yaw), math.sin(yaw)
        self.target += np.array([c * vx - s * vy, s * vx + c * vy, vz]) * dt
        self.target[2] = max(self.target[2], 0.05)
        err = self.target - pos
        if np.linalg.norm(err) > 0.6:  # bateu em algo: não acumula erro
            self.target = pos + err / np.linalg.norm(err) * 0.6
            err = self.target - pos
        self.target_yaw += wz * dt
        eyaw = math.atan2(math.sin(self.target_yaw - yaw), math.cos(self.target_yaw - yaw))
        lin = 3.0 * err + np.array([c * vx - s * vy, s * vx + c * vy, vz])
        ang = np.array([-6.0 * roll, -6.0 * pitch, 4.0 * eyaw + wz])
        # converte velocidades angulares de roll/pitch (no corpo) para o mundo, aproximação de pequenos ângulos
        ang_w = np.array([c * ang[0] - s * ang[1], s * ang[0] + c * ang[1], ang[2]])
        self.art.set_velocities(lin[None], ang_w[None])
        if self.rot_idx:
            flying = pos[2] > self.home_pos[2] + 0.05 or np.any(cmd["base"]) or vz
            self.spin = min(self.spin + 2 * dt, 1.0) if flying else max(self.spin - dt, 0.0)
            vel = np.array([[(1 if k % 2 else -1) * 150.0 * self.spin for k in range(len(self.rot_idx))]])
            self.art.set_dof_velocities(vel, dof_indices=self.rot_idx)

    def reset(self):
        super().reset()
        self.target = self.home_pos.copy()
        self.target_yaw = quat_to_rpy(self.home_quat)[2]


CLASSES = {"ridgeback": Ridgeback, "spot": Spot, "galbot": Galbot, "drone": Drone}
robots = [CLASSES[s.tipo](s) for s in SPECS]


# ---------------------------------------------------------------------------------------------------------------------
# Teclado
# ---------------------------------------------------------------------------------------------------------------------
KI = carb.input.KeyboardInput
pressed: set = set()
events: list = []
state = {"active": 0, "cam": 0, "msg": ""}
CAM_MODES = ["Panorâmica", "Seguir robô", "Câmera do robô"]


def on_key(event, *_):
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        pressed.add(event.input)
        events.append(event.input)
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        pressed.discard(event.input)
    return True


def axis(pos_keys, neg_keys):
    return float(any(k in pressed for k in pos_keys)) - float(any(k in pressed for k in neg_keys))


def read_command():
    fast = KI.LEFT_SHIFT in pressed or KI.RIGHT_SHIFT in pressed
    r = robots[state["active"]]
    vmax = {"drone": 2.5, "spot": 1.0, "galbot": 0.8}.get(r.s.tipo, 1.0) * (2.0 if fast else 1.0)
    if r.s.nome == "drone_crazyflie":
        vmax *= 0.4
    wmax = (1.6 if fast else 0.8) * (1.25 if r.s.tipo == "spot" else 1.0)
    cmd = {
        "base": (vmax * axis([KI.UP], [KI.DOWN]), vmax * axis([KI.LEFT], [KI.RIGHT]), wmax * axis([KI.Q], [KI.E])),
        "vz": vmax * 0.6 * axis([KI.R, KI.PAGE_UP], [KI.V, KI.PAGE_DOWN]),
        "ee": tuple((0.25 if fast else 0.1) * a for a in (axis([KI.I], [KI.K]), axis([KI.J], [KI.L]),
                                                          axis([KI.U], [KI.O]))),
        "joint": axis([KI.EQUAL, KI.NUMPAD_ADD], [KI.MINUS, KI.NUMPAD_SUBTRACT]),
        "fast": fast, "toggle_grip": False, "home_arm": False, "inspect_arm": False,
    }
    return cmd


NUM_KEYS = [KI.KEY_1, KI.KEY_2, KI.KEY_3, KI.KEY_4, KI.KEY_5, KI.KEY_6, KI.KEY_7, KI.KEY_8]


def handle_events(cmd):
    while events:
        k = events.pop(0)
        if k == KI.TAB:
            select((state["active"] + 1) % len(robots))
        elif k in NUM_KEYS and NUM_KEYS.index(k) < len(robots):
            select(NUM_KEYS.index(k))
        elif k == KI.C:
            state["cam"] = (state["cam"] + 1) % len(CAM_MODES)
            apply_camera()
        elif k == KI.G:
            cmd["toggle_grip"] = True
        elif k == KI.H:
            cmd["home_arm"] = True
        elif k == KI.P:
            cmd["inspect_arm"] = True
        elif k in (KI.LEFT_BRACKET, KI.RIGHT_BRACKET):
            r = robots[state["active"]]
            r.joint_sel += 1 if k == KI.RIGHT_BRACKET else -1
        elif k == KI.BACKSPACE:
            robots[state["active"]].reset()


def select(i):
    state["active"] = i
    apply_camera()
    print(f"[inspecao] robô ativo: {i + 1} - {robots[i].s.rotulo}", flush=True)


# ---------------------------------------------------------------------------------------------------------------------
# Câmera / viewport / janela de ajuda
# ---------------------------------------------------------------------------------------------------------------------
viewport = None
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()


def apply_camera():
    if viewport is None:
        return
    r = robots[state["active"]]
    mode = state["cam"]
    if mode == 2 and r.s.cam_path is None:
        mode = 1
    viewport.camera_path = ["/World/Cameras/Panoramica", "/World/Cameras/Seguir", r.s.cam_path][mode]


def update_follow_camera():
    r = robots[state["active"]]
    try:
        pos, quat = r.pose()
    except Exception:  # noqa: BLE001
        return
    yaw = r.heading()
    d = r.s.seguir
    eye = pos + np.array([-math.cos(yaw) * d, -math.sin(yaw) * d, 0.45 * d + 0.3])
    set_camera_pose(stage, "/World/Cameras/Seguir", eye.tolist(), (pos + np.array([0, 0, 0.3])).tolist())


ui_labels = {}


def build_ui():
    import omni.ui as ui

    win = ui.Window("Controles - Inspeção de Avião", width=430, height=560, position_x=1480, position_y=420)
    with win.frame:
        with ui.VStack(spacing=4):
            ui_labels["ativo"] = ui.Label("", height=22, style={"font_size": 18, "color": 0xFF33CCFF})
            ui_labels["camera"] = ui.Label("", height=18)
            ui_labels["junta"] = ui.Label("", height=18)
            ui.Separator(height=6)
            for i, r in enumerate(robots):
                ui.Button(f"{i + 1}  {r.s.rotulo}", height=24, clicked_fn=lambda i=i: select(i))
            ui.Button("C  trocar câmera", height=24,
                      clicked_fn=lambda: (state.update(cam=(state["cam"] + 1) % 3), apply_camera()))
            ui.Separator(height=6)
            ui.Label(
                "Setas: andar   Q/E: girar   SHIFT: rápido\n"
                "R/V ou PgUp/PgDn: subir/descer (drones)\n"
                "I/K, J/L, U/O: garra frente/trás, esq/dir, sobe/desce\n"
                "G: abre/fecha garra   H: braço na pose inicial\n"
                "P: braço na pose de inspeção (garra para cima)\n"
                "[ ]: escolhe junta   - =: move a junta\n"
                "TAB ou 1..8: troca robô   C: câmera\n"
                "BACKSPACE: reposiciona o robô ativo\n"
                "Clique no viewport antes de usar o teclado.",
                word_wrap=True)
    return win


def refresh_ui():
    if not ui_labels:
        return
    r = robots[state["active"]]
    names = r.joint_names_for_ui()
    ui_labels["ativo"].text = f"Ativo: {state['active'] + 1} - {r.s.rotulo}"
    ui_labels["camera"].text = f"Câmera: {CAM_MODES[state['cam']]}"
    ui_labels["junta"].text = f"Junta [{r.joint_sel % len(names) + 1}/{len(names)}]: {names[r.joint_sel % len(names)]}" \
        if names else "Junta: (sem juntas)"


# ---------------------------------------------------------------------------------------------------------------------
# Loop principal
# ---------------------------------------------------------------------------------------------------------------------
SimulationManager.setup_simulation(dt=PHYSICS_DT, device="cpu")
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(10):
    simulation_app.update()
for r in robots:
    r.init_physics()

cmd_holder = {"cmd": read_command()}
idle = {"base": (0.0, 0.0, 0.0), "vz": 0.0, "ee": (0, 0, 0), "joint": 0.0, "fast": False,
        "toggle_grip": False, "home_arm": False, "inspect_arm": False}
demo = {"on": False, "t": 0.0}  # demo["t"] também é o relógio da simulação (s)
api = None


def on_physics(dt, _ctx):
    cmd = cmd_holder["cmd"]
    for i, r in enumerate(robots):
        c = cmd if i == state["active"] else idle
        if demo["on"]:
            c = demo_command(r, demo["t"])
        elif api is not None:
            c = api.comando_para(r.s.nome, c)
        if isinstance(r, Spot):
            r.physics(c["base"], dt)
        else:
            r.control(c, dt)
        if c["joint"]:
            r.joint_step(c["joint"], dt, c["fast"])
    cmd["toggle_grip"] = cmd["home_arm"] = cmd["inspect_arm"] = False
    demo["t"] += dt


phys_cb = SimulationManager.register_callback(on_physics, IsaacEvents.POST_PHYSICS_STEP)


def demo_command(r, t):
    """Roteiro usado no modo --capturas: drones decolam até a altura do avião e os braços se esticam."""
    c = dict(idle)
    if r.s.tipo == "drone":
        alvo = {"drone_arl": 7.5, "drone_crazyflie": 3.2, "drone_quadcopter": 4.6, "drone_ingenuity": 2.4}[r.s.nome]
        c["vz"] = 1.5 if r.pose()[0][2] < alvo else 0.0
    if isinstance(r, Ridgeback) and t < 0.01:
        c["inspect_arm"] = True
    return c


win = None
if not args.headless:
    win = build_ui()
    input_iface = carb.input.acquire_input_interface()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    key_sub = input_iface.subscribe_to_keyboard_events(keyboard, on_key)
    state["cam"] = 0
    apply_camera()
print("[inspecao] pronto. Robôs: " + ", ".join(f"{i + 1}={r.s.nome}" for i, r in enumerate(robots)), flush=True)


def capture_images(folder):
    import omni.replicator.core as rep
    from PIL import Image

    os.makedirs(folder, exist_ok=True)
    demo["on"] = True
    for _ in range(int(6.0 / (1 / 60))):  # 6 s de roteiro (drones subindo)
        simulation_app.update()
    demo["on"] = False
    for r in robots:
        p, q = r.pose()
        print(f"[inspecao] pose {r.s.nome}: pos={np.round(p, 2).tolist()} rpy={np.round(np.degrees(quat_to_rpy(q)), 1).tolist()}",
              flush=True)
    views = {
        "01_visao_geral.png": ((105, -48, 62), (0, -48, 0)),
        "02_robos_em_fila.png": ((21, -17, 3.5), (21, -6, 1.2)),
        "03_drone_inspecionando_fuselagem.png": ((26, -22, 9), (17, -5, 7)),
        "04_frota_de_drones.png": ((14.5, -14, 3.6), (14.5, -6, 3.8)),
        "05_spot_com_braco.png": ((26.5, -10, 1.6), (24, -6, 0.5)),
        "06_galbot_g1.png": ((22.5, -10.5, 1.6), (20.5, -6, 0.9)),
        "07_avioes_lado_a_lado.png": ((55, -30, 18), (0, -60, 3)),
        "08_vista_aerea.png": ((0.01, -46, 210), (0, -46, 0)),
        "09_space_shuttle.png": ((28, -85, 12), (-14, -106, 8)),
    }
    cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Captura")
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 3000))
    set_camera_pose(stage, "/World/Cameras/Captura", *views["01_visao_geral.png"])

    def shoot(camera_path, fname, setup=None):
        rp = rep.create.render_product(camera_path, (1600, 900))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])
        if setup:
            setup()
        for _ in range(45):  # estabiliza iluminação / streaming de texturas
            simulation_app.update()
        img = rgb.get_data()
        if img is not None and np.asarray(img).size:
            Image.fromarray(np.asarray(img)[:, :, :3]).save(os.path.join(folder, fname))
            print(f"[inspecao] imagem: {os.path.join(folder, fname)}", flush=True)
        rgb.detach([rp])
        rp.destroy()

    for fname, view in views.items():
        shoot("/World/Cameras/Captura", fname, lambda v=view: set_camera_pose(stage, "/World/Cameras/Captura", *v))


frame = 0


# ---------------------------------------------------------------------------------------------------------------------
# Controle por código / gravação (scripts/api_externa.py)
# ---------------------------------------------------------------------------------------------------------------------
from types import SimpleNamespace  # noqa: E402

import api_externa  # noqa: E402


def set_camera_mode(m):
    state["cam"] = m % len(CAM_MODES)
    apply_camera()


api = api_externa.API(SimpleNamespace(robots=robots, select=select, set_camera_mode=set_camera_mode,
                                      sim_time=lambda: demo["t"], find_link=find_link, stage=stage))
if args.ativo:
    api.selecionar(args.ativo)
servidor = api_externa.ServidorTCP(api, args.servidor) if args.servidor else None
missao = api_externa.carregar_missao(args.missao) if args.missao else None
missao_ok = {"fim": False}
if missao is not None and hasattr(missao, "inicio"):
    missao.inicio(api)
gravador = None
if args.gravar_video:
    r0 = robots[state["active"]]
    cam_video = {"panoramica": "/World/Cameras/Panoramica", "seguir": "/World/Cameras/Seguir",
                 "robo": r0.s.cam_path or "/World/Cameras/Seguir"}[args.camera_video]
    gravador = api_externa.GravadorVideo(cam_video, args.gravar_video, fps=args.fps_video)
log_csv = api_externa.LogCSV(api, args.log_csv) if args.log_csv else None


def tick():
    """Um quadro do laço interativo: lê teclado, aplica comandos, atualiza câmera/painel e renderiza."""
    global frame
    cmd = read_command()
    handle_events(cmd)
    cmd_holder["cmd"] = cmd
    if servidor is not None:
        servidor.poll()
    if missao is not None and not missao_ok["fim"]:
        if missao.passo(api, demo["t"]):
            missao_ok["fim"] = True
            print(f"[missao] terminou em t = {demo['t']:.1f} s", flush=True)
    api.atualizar()
    if state["cam"] == 1 or (gravador is not None and args.camera_video == "seguir"):
        update_follow_camera()
    if frame % 10 == 0:
        refresh_ui()
    simulation_app.update()
    if gravador is not None:
        gravador.quadro(demo["t"])
    if log_csv is not None:
        log_csv.quadro()
    frame += 1


def screenshot_window(path):
    """Salva a janela inteira do Isaac Sim (viewport + painéis) em PNG."""
    import omni.kit.renderer_capture

    cap = omni.kit.renderer_capture.acquire_renderer_capture_interface()
    cap.capture_next_frame_swapchain(path)
    for _ in range(5):
        simulation_app.update()
    cap.wait_async_capture()


def autotest():
    """Aperta teclas virtualmente em cada robô e confere se a base / braço / drone respondem."""
    def run(keys, secs):
        pressed.clear()
        pressed.update(keys)
        for _ in range(int(secs * 60)):
            tick()
        pressed.clear()
        cmd_holder["cmd"] = read_command()
        for _ in range(20):
            simulation_app.update()
        if os.environ.get("AUTOTESTE_DEBUG"):
            pz, pq = robots[state["active"]].pose()
            print(f"[autoteste]   apos {[k.name for k in keys]}: z={pz[2]:.2f} rpy={np.round(np.degrees(quat_to_rpy(pq)), 0)}",
                  flush=True)

    for _ in range(int(float(os.environ.get("AUTOTESTE_ESPERA", "0")) * 60)):
        simulation_app.update()
    events.append(KI.C)  # câmera "seguir robô" durante o teste
    for i, r in enumerate(robots):
        select(i)
        if os.environ.get("AUTOTESTE_DEBUG"):
            pz, pq = r.pose()
            print(f"[autoteste]   inicio: z={pz[2]:.2f} rpy={np.round(np.degrees(quat_to_rpy(pq)), 0)}", flush=True)
        p0, _ = r.pose()
        h0 = r.heading()
        run({KI.UP}, 2.0)
        p1, _ = r.pose()
        run({KI.Q}, 1.5)
        h1 = r.heading()
        extra = ""
        if isinstance(r, Ridgeback):
            ee = XformPrim(str(find_link(r.s.path, r.s.ee)))
            e0 = ee.get_world_poses()[0].numpy()[0]
            run({KI.U}, 1.5)
            e1 = ee.get_world_poses()[0].numpy()[0]
            extra = f" garra subiu {e1[2] - e0[2]:+.2f} m"
        if isinstance(r, Drone):
            z0 = r.pose()[0][2]
            run({KI.R}, 2.0)
            extra = f" subiu {r.pose()[0][2] - z0:+.2f} m"
        if r.jlist:
            j = r.jlist[r.joint_sel % len(r.jlist)]
            q0 = float(r.art.get_dof_positions().numpy()[0][j])
            run({KI.EQUAL}, 1.0)
            q1 = float(r.art.get_dof_positions().numpy()[0][j])
            extra += f" junta '{r.dofs[j]}' moveu {q1 - q0:+.2f} rad"
        pz, pq = r.pose()
        up0, up1 = up_vector(r.home_quat if not isinstance(r, Ridgeback) else (1, 0, 0, 0)), up_vector(pq)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(up0, up1))))))
        extra += f" | altura {pz[2]:.2f} m" + ("" if isinstance(r, Galbot) else f", inclinação {tilt:.0f} graus")
        if not args.headless and os.environ.get("AUTOTESTE_PRINT") and r.s.nome == os.environ["AUTOTESTE_PRINT"]:
            screenshot_window(os.path.abspath(os.path.join(ROOT, "imagens", "11_janela_isaac_sim.png")))
        print(f"[autoteste] {r.s.nome}: andou {np.linalg.norm((p1 - p0)[:2]):.2f} m, girou "
              f"{math.degrees(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))):+.0f} graus,{extra}", flush=True)


if args.autoteste:
    autotest()
elif args.capturas:
    capture_images(os.path.abspath(args.capturas))
else:
    while simulation_app.is_running():
        tick()
        if args.frames and frame >= args.frames:
            break
        if args.headless and missao_ok["fim"] and not api.fotos:
            break  # sem janela: fecha quando a missão termina

for obj in (gravador, log_csv, servidor):
    if obj is not None:
        obj.fechar()
SimulationManager.deregister_callback(phys_cb)
timeline.stop()
simulation_app.close()
