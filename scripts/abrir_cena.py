"""Abre o Isaac Sim com a interface completa e carrega um arquivo USD (só visualização, sem controladores).

Uso:  python scripts/abrir_cena.py cenas/cena_inspecao_aviao.usda
"""
import os
import sys

from isaacsim import SimulationApp

path = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "cenas/cena_inspecao_aviao.usda")
app = SimulationApp({"headless": False, "width": 1600, "height": 900})

import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402

ok, _ = stage_utils.open_stage(path)
print(f"[abrir_cena] {'aberto' if ok else 'FALHA ao abrir'}: {path}", flush=True)
while app.is_running():
    app.update()
app.close()
