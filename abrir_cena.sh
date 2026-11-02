#!/usr/bin/env bash
# Abre o Isaac Sim (interface completa) já com um arquivo USD carregado, só para olhar/explorar os modelos.
#
#   ./abrir_cena.sh                                           cena completa (cenas/cena_inspecao_aviao.usda)
#   ./abrir_cena.sh assets/avioes/usd/sofia_747sp/sofia_747sp.usd   um avião sozinho
#
# Obs.: abrindo assim os robôs NÃO têm controlador (é só visualização). Para controlar use ./executar.sh
set -euo pipefail
cd "$(dirname "$0")"
ISAACSIM_ENV="${ISAACSIM_ENV:-$HOME/isaacsim-projeto/env_isaacsim61}"
if [[ ! -f "$ISAACSIM_ENV/bin/activate" ]]; then
    echo "Não encontrei o ambiente do Isaac Sim em: $ISAACSIM_ENV  (veja o GUIA_ISAAC_SIM.md)"
    exit 1
fi
# shellcheck disable=SC1091
source "$ISAACSIM_ENV/bin/activate"
export OMNI_KIT_ACCEPT_EULA=YES
exec python scripts/abrir_cena.py "${1:-cenas/cena_inspecao_aviao.usda}"
