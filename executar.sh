#!/usr/bin/env bash
# Abre a simulação de inspeção do avião (robôs + drones controláveis pelo teclado).
#
#   ./executar.sh                 cena completa
#   ./executar.sh --cena leve     só 747 + Ridgeback/Franka + drone ARL (PC mais fraco)
#   ./executar.sh --help          todas as opções
#
# Se o Isaac Sim estiver instalado em outro lugar, informe o ambiente Python dele:
#   ISAACSIM_ENV=/caminho/para/env_isaacsim ./executar.sh
set -euo pipefail
cd "$(dirname "$0")"
ISAACSIM_ENV="${ISAACSIM_ENV:-$HOME/isaacsim-projeto/env_isaacsim61}"
if [[ ! -f "$ISAACSIM_ENV/bin/activate" ]]; then
    echo "Não encontrei o ambiente do Isaac Sim em: $ISAACSIM_ENV"
    echo "Veja a seção 2 do GUIA_ISAAC_SIM.md ou rode:  ISAACSIM_ENV=/seu/env ./executar.sh"
    exit 1
fi
# shellcheck disable=SC1091
source "$ISAACSIM_ENV/bin/activate"
export OMNI_KIT_ACCEPT_EULA=YES
exec python scripts/inspecao_aviao.py "$@"
