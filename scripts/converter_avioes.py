"""Converte os modelos de avião da NASA (assets/avioes/originais) para USD prontos para o Isaac Sim.

Os .glb da NASA usam compressão Draco, que o conversor do Isaac Sim (omni.kit.asset_converter) não decodifica
(gera USD vazio). Por isso a leitura é feita com o Open3D (já instalado no ambiente do Isaac Sim) e o USD é escrito
diretamente com a API pxr.

Cada avião é convertido para:
  * metros, eixo Z para cima, nariz apontando para +X, centrado em X/Y e apoiado no chão (z = 0);
  * escala real (envergadura/comprimento oficiais);
  * malha visual + malha de colisão simplificada (colisor estático, triangle mesh) para os robôs não atravessarem.

Uso (com o ambiente do Isaac Sim ativado):
    python scripts/converter_avioes.py
"""
import glob
import io
import json
import os
import struct

import numpy as np
import open3d as o3d
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ORIG = os.path.join(ROOT, "assets", "avioes", "originais")
OUT = os.path.join(ROOT, "assets", "avioes", "usd")

# Matriz R (linhas = novos eixos X, Y, Z escritos no sistema do arquivo original):
#   X novo = frente (nariz), Y novo = esquerda, Z novo = cima.
AVIOES = {
    "sofia_747sp": dict(
        nome="NASA SOFIA (Boeing 747SP)",
        arquivos=sorted(glob.glob(os.path.join(ORIG, "sofia_747sp", "*.stl"))),
        R=[[-1, 0, 0], [0, -1, 0], [0, 0, 1]],
        medida=("comprimento", 56.31),
        cor=(0.92, 0.92, 0.94),
        altura_chao=1.4,  # o modelo de impressão 3D não tem trem de pouso: suspendemos e criamos um trem simples
        max_tris_visual=None,
    ),
    "global_hawk": dict(
        nome="NASA Global Hawk (RQ-4A)",
        arquivos=[os.path.join(ORIG, "global_hawk", "Global_Hawk.glb")],
        R=[[0, 0, 1], [1, 0, 0], [0, 1, 0]],
        medida=("envergadura", 35.4),
        cor=None,
        altura_chao=0.0,
        max_tris_visual=None,
    ),
    "x57_maxwell": dict(
        nome="NASA X-57 Maxwell",
        arquivos=[os.path.join(ORIG, "x57", "X-57.glb")],
        R=[[-1, 0, 0], [0, 0, 1], [0, 1, 0]],
        medida=("envergadura", 9.6),
        cor=(0.85, 0.86, 0.88),
        altura_chao=0.0,
        max_tris_visual=300_000,
    ),
    "space_shuttle": dict(
        nome="Space Shuttle (orbitador + tanque + boosters)",
        arquivos=[os.path.join(ORIG, "space_shuttle", "Space_Shuttle.glb")],
        R=[[0, 0, -1], [-1, 0, 0], [0, 1, 0]],
        medida=("comprimento", 56.1),
        cor=(0.88, 0.88, 0.86),
        altura_chao=0.0,
        max_tris_visual=None,
    ),
}
MAX_TRIS_COLISAO = 40_000


def glb_textures(path):
    """Devolve {índice do material glTF: imagem PIL} para materiais com baseColorTexture (inclui webp)."""
    if not path.endswith(".glb"):
        return {}, []
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[12:16])[0]
    j = json.loads(b[20 : 20 + n])
    binoff = 20 + n + 8
    out, factors = {}, []
    for i, m in enumerate(j.get("materials", [])):
        pbr = m.get("pbrMetallicRoughness", {})
        factors.append(pbr.get("baseColorFactor", [1, 1, 1, 1]))
        if "baseColorTexture" not in pbr:
            continue
        tex = j["textures"][pbr["baseColorTexture"]["index"]]
        src = tex.get("source", tex.get("extensions", {}).get("EXT_texture_webp", {}).get("source"))
        bv = j["bufferViews"][j["images"][src]["bufferView"]]
        o = binoff + bv.get("byteOffset", 0)
        out[i] = Image.open(io.BytesIO(b[o : o + bv["byteLength"]])).convert("RGB")
    return out, factors


def load(cfg):
    """Lê todos os arquivos do avião e devolve lista de (vértices, triângulos, normais, uvs, cor, textura)."""
    parts = []
    for f in cfg["arquivos"]:
        model = o3d.io.read_triangle_model(f)
        texs, factors = glb_textures(f)
        for g in model.meshes:
            mesh = g.mesh
            mat = model.materials[g.material_idx]
            cor = tuple(float(c) for c in mat.base_color[:3])
            tex = None
            # o Open3D (assimp) mantém a ordem dos materiais do glTF; casamos pelo índice e conferimos a cor
            if g.material_idx in texs:
                tex = texs[g.material_idx]
            if cfg["cor"] is not None and tex is None:
                cor = cfg["cor"]
            if cfg["max_tris_visual"] and len(mesh.triangles) > cfg["max_tris_visual"]:
                mesh = mesh.simplify_quadric_decimation(cfg["max_tris_visual"])
                tex = None
            mesh.compute_vertex_normals()
            uv = np.asarray(mesh.triangle_uvs) if tex is not None and mesh.has_triangle_uvs() else None
            parts.append([np.asarray(mesh.vertices), np.asarray(mesh.triangles), np.asarray(mesh.vertex_normals), uv, cor, tex])
    return parts


def material(stage, path, cor, tex_file):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.25)
    if tex_file:
        st = UsdShade.Shader.Define(stage, path + "/st")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(stage, path + "/Tex")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex_file)
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*cor))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def write_mesh(stage, path, v, t, n=None, uv=None):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(t), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(t.reshape(-1).astype(np.int32)))
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    if n is not None:
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(n.astype(np.float32)))
        m.SetNormalsInterpolation("vertex")
    if uv is not None:
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        uv = uv.copy()
        uv[:, 1] = 1.0 - uv[:, 1]  # glTF tem V para baixo; USD para cima
        pv.Set(Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
    lo, hi = v.min(0), v.max(0)
    m.CreateExtentAttr([Gf.Vec3f(*lo.tolist()), Gf.Vec3f(*hi.tolist())])
    return m


def trem_de_pouso(stage, root, pts):
    """Trem de pouso simplificado (cilindros) para o modelo SOFIA, que veio sem rodas."""
    UsdGeom.Xform.Define(stage, root)
    bmin, bmax = pts.min(0), pts.max(0)
    L = bmax[0] - bmin[0]
    cinza = material(stage, root + "/Metal", (0.55, 0.56, 0.58), None)
    preto = material(stage, root + "/Pneu", (0.05, 0.05, 0.05), None)
    gear = [(bmax[0] - 0.12 * L, 0.0)]  # bequilha (nariz em +X)
    for fx in (0.50, 0.56):  # 4 conjuntos principais (747: 2 na asa, 2 na fuselagem)
        for y in (-1.9, 1.9):
            gear.append((bmax[0] - fx * L, y))
    for i, (x, y) in enumerate(gear):
        # altura da barriga da fuselagem logo acima da perna
        perto = pts[(np.abs(pts[:, 0] - x) < 1.0) & (np.abs(pts[:, 1] - y) < 1.0)]
        topo = float(perto[:, 2].min()) + 0.2 if len(perto) else 3.0
        h = topo - 0.6
        perna = UsdGeom.Cylinder.Define(stage, f"{root}/perna_{i}")
        perna.CreateRadiusAttr(0.18)
        perna.CreateHeightAttr(h)
        perna.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.6 + h / 2))
        UsdShade.MaterialBindingAPI.Apply(perna.GetPrim()).Bind(cinza)
        for dy in (-0.45, 0.45):
            roda = UsdGeom.Cylinder.Define(stage, f"{root}/roda_{i}_{'e' if dy > 0 else 'd'}")
            roda.CreateRadiusAttr(0.62)
            roda.CreateHeightAttr(0.45)
            roda.CreateAxisAttr("Y")
            roda.AddTranslateOp().Set(Gf.Vec3d(x, y + dy, 0.62))
            UsdShade.MaterialBindingAPI.Apply(roda.GetPrim()).Bind(preto)
            UsdPhysics.CollisionAPI.Apply(roda.GetPrim())


def convert(key, cfg):
    print(f"[{key}] lendo {len(cfg['arquivos'])} arquivo(s)...", flush=True)
    parts = load(cfg)
    R = np.array(cfg["R"], float)
    allv = np.concatenate([p[0] for p in parts]) @ R.T
    ext = allv.max(0) - allv.min(0)
    tipo, valor = cfg["medida"]
    s = valor / (ext[0] if tipo == "comprimento" else ext[1])
    center = (allv.max(0) + allv.min(0)) / 2
    off = np.array([-center[0], -center[1], -allv.min(0)[2]]) * s + np.array([0, 0, cfg["altura_chao"]])

    out_dir = os.path.join(OUT, key)
    os.makedirs(os.path.join(out_dir, "texturas"), exist_ok=True)
    usd_path = os.path.join(out_dir, key + ".usd")
    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Aviao")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    root.GetPrim().SetCustomDataByKey("descricao", cfg["nome"] + " - modelo NASA 3D Resources")
    UsdGeom.Scope.Define(stage, "/Aviao/Materiais")
    UsdGeom.Xform.Define(stage, "/Aviao/Visual")

    col_v, col_t, base = [], [], 0
    for i, (v, t, n, uv, cor, tex) in enumerate(parts):
        v2 = v @ R.T * s + off
        n2 = n @ R.T if len(n) == len(v) else None
        tex_file = None
        if tex is not None:
            tex_file = f"./texturas/{key}_{i}.png"
            tex.save(os.path.join(out_dir, tex_file))
        mesh = write_mesh(stage, f"/Aviao/Visual/parte_{i:03d}", v2, t, n2, uv)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(stage, f"/Aviao/Materiais/mat_{i:03d}", cor, tex_file))
        col_v.append(v2)
        col_t.append(t + base)
        base += len(v2)

    # malha de colisão: junta tudo e simplifica
    cm = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.concatenate(col_v)), o3d.utility.Vector3iVector(np.concatenate(col_t)))
    cm.remove_duplicated_vertices()
    if len(cm.triangles) > MAX_TRIS_COLISAO:
        cm = cm.simplify_quadric_decimation(MAX_TRIS_COLISAO)
    col = write_mesh(stage, "/Aviao/Colisao", np.asarray(cm.vertices), np.asarray(cm.triangles))
    col.CreatePurposeAttr(UsdGeom.Tokens.guide)  # invisível na renderização normal
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(col.GetPrim()).CreateApproximationAttr("none")

    allv2 = allv * s + off
    bmin, bmax = allv2.min(0), allv2.max(0)
    if cfg["altura_chao"] > 0:
        trem_de_pouso(stage, "/Aviao/TremDePouso", allv2)
    root.GetPrim().SetCustomDataByKey("dimensoes_m", Gf.Vec3d(*(bmax - bmin + [0, 0, 0]).tolist()))
    stage.GetRootLayer().Save()
    print(f"[{key}] OK -> {usd_path}  dimensões (C x E x A) = {np.round(bmax - bmin, 2)} m, z = {bmin[2]:.2f}..{bmax[2]:.2f}", flush=True)


if __name__ == "__main__":
    import sys

    for k in sys.argv[1:] or AVIOES:
        convert(k, AVIOES[k])
