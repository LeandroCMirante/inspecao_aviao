# Desenvolvimento: controlar por código, gravar e modificar a simulação

Este documento é para quem vai **programar em cima** da simulação: mandar os robôs fazerem uma inspeção
automaticamente, controlá-los de outro programa, gravar vídeo e dados, e tirar ou colocar coisas na cena.
Para instalar o Isaac Sim, usar a interface e o teclado, veja o [GUIA_ISAAC_SIM.md](GUIA_ISAAC_SIM.md).

## Sumário

1. [Três jeitos de controlar os robôs](#1-três-jeitos-de-controlar-os-robôs)
2. [Missão: um script Python que roda dentro da simulação](#2-missão-um-script-python-que-roda-dentro-da-simulação)
3. [Referência da API](#3-referência-da-api)
4. [Servidor TCP/JSON: controle de outro programa](#4-servidor-tcpjson-controle-de-outro-programa)
5. [Gravar: vídeo, fotos e poses em CSV](#5-gravar-vídeo-fotos-e-poses-em-csv)
6. [Tirar, colocar e alterar coisas na cena](#6-tirar-colocar-e-alterar-coisas-na-cena)
7. [Como o código está organizado](#7-como-o-código-está-organizado)
8. [Adicionar um robô novo](#8-adicionar-um-robô-novo)

---

## 1. Três jeitos de controlar os robôs

| Jeito | Quando usar | Como |
|---|---|---|
| **Teclado** | explorar a cena, testar à mão | `./executar.sh` (ver o guia, seção 5) |
| **Missão** (Python dentro da simulação) | roteiros de inspeção automáticos, repetíveis, gravados | `./executar.sh --missao meu_script.py` |
| **Servidor TCP/JSON** | controlar de outro processo ou linguagem (C++, MATLAB, Node, outro Python, um nó ROS seu) | `./executar.sh --servidor` |

Os três podem ser usados juntos. Um comando por código vale por um tempo curto (`duracao`). Quando ele expira, o
robô volta a obedecer ao teclado (o robô ativo) ou fica parado (os outros).

Todas as opções:

```bash
./executar.sh --help
```

| Opção | O que faz |
|---|---|
| `--missao ARQ.py` | roda uma missão (seção 2) |
| `--servidor [PORTA]` | abre o servidor TCP; a porta padrão é 5555 (seção 4) |
| `--gravar-video ARQ.mp4` | grava um vídeo MP4 (seção 5) |
| `--camera-video {panoramica,seguir,robo}` | câmera usada no vídeo (padrão `seguir`) |
| `--fps-video N` | quadros por segundo do vídeo (padrão 30) |
| `--log-csv ARQ.csv` | grava as poses de todos os robôs a 10 Hz |
| `--ativo NOME` | robô ativo no início (ex.: `drone_arl`) |
| `--robos A,B,...` | carrega só esses robôs |
| `--cena leve` | só o 747, o Ridgeback+Franka e o drone ARL |
| `--headless` | sem janela; com `--missao`, a simulação fecha sozinha quando a missão termina |
| `--frames N` | fecha depois de N quadros |

Nomes dos robôs: `ridgeback_franka`, `ridgeback_ur5`, `spot_braco`, `galbot_g1`, `drone_arl`, `drone_crazyflie`,
`drone_quadcopter` e `drone_ingenuity`.

---

## 2. Missão: um script Python que roda dentro da simulação

Uma missão é um arquivo `.py` comum com duas funções:

```python
def inicio(api):          # opcional: chamada uma vez, antes do primeiro quadro
    ...

def passo(api, t):        # obrigatória: chamada a cada quadro; t = tempo de simulação em segundos
    ...
    return True           # quando a missão acabar (senão retorne False ou None)
```

`api` é o objeto descrito na [seção 3](#3-referência-da-api). A função `passo` precisa ser **rápida e não pode
bloquear**: nada de `time.sleep` nem de laços esperando o robô chegar. A simulação só avança depois que `passo`
retorna. O jeito certo é guardar em que etapa a missão está e verificar a cada quadro (uma "máquina de estados"):

```python
etapa = {"n": 0}

def passo(api, t):
    if etapa["n"] == 0:
        api.ir_para("drone_arl", 17, -6.5, 7.5)            # decola e vai até o ponto
        etapa["n"] = 1
    elif etapa["n"] == 1 and not api.estado("drone_arl")["navegando"]:
        api.foto("drone_arl", "saidas/fotos/ponto1.png")   # chegou: tira a foto
        etapa["n"] = 2
    return etapa["n"] == 2
```

O exemplo completo está em [`exemplos/missao_inspecao.py`](exemplos/missao_inspecao.py). Nele:

* o **drone ARL** decola até 7,5 m, percorre o lado esquerdo da fuselagem do 747 tirando 4 fotos, volta pelo alto e
  pousa;
* ao mesmo tempo, o **Ridgeback+Franka** sai da fila, vai até embaixo do motor interno esquerdo, vira de frente para
  ele, põe o braço na pose de inspeção (garra para cima) e fotografa.

Para rodar e gravar tudo:

```bash
./executar.sh --headless --cena leve --missao exemplos/missao_inspecao.py \
    --gravar-video saidas/missao.mp4 --log-csv saidas/missao.csv
```

O resultado fica em `saidas/`: `missao.mp4`, `missao.csv` e `fotos/*.png`. A missão leva cerca de 44 s de
simulação. Sem `--headless`, a janela continua aberta no final e dá para seguir com o teclado.

Os caminhos relativos (`saidas/...`) partem da pasta `inspecao_aviao/`, porque o `executar.sh` entra nela.

**Coordenadas.** O mundo é em metros, com Z para cima. O 747 tem o nariz em +X e o eixo da fuselagem em y = 0. A
fila de robôs fica em y = −6, de x = 30 (Ridgeback+Franka) a x = 12 (Ingenuity). O motor interno esquerdo fica em
torno de x = 6, y = −10,8. O rumo é medido em graus a partir de +X, no sentido anti-horário: 90° aponta para +Y, a
fuselagem do avião.

---

## 3. Referência da API

A API está em `scripts/api_externa.py` (classe `API`). Os mesmos métodos são usados pelas missões e pelo servidor
TCP.

### Consulta

| Método | Retorna |
|---|---|
| `robos()` | lista de `{"nome", "tipo", "descricao", "juntas", "camera"}` dos robôs carregados |
| `tempo()` | tempo de simulação (s) |
| `estado(nome)` | `{"pos": [x,y,z], "rumo_graus", "roll_graus", "pitch_graus", "navegando", "juntas": {...}, "garra_pos": [x,y,z]}` (`garra_pos` só nos robôs com braço) |

### Movimento

| Método | O que faz |
|---|---|
| `velocidade(nome, vx=0, vy=0, wz=0, vz=0, duracao=0.5)` | velocidade no referencial do robô: `vx` para a frente, `vy` para a esquerda (m/s), `wz` giro (rad/s), `vz` subida (m/s, só drones). Vale por `duracao` segundos de simulação; para manter, reenvie. |
| `ir_para(nome, x, y, z=None, vel=1.0, tol=0.15)` | navega até o ponto no mundo (`z` só para drones). Acompanhe com `estado(nome)["navegando"]`. |
| `girar_para(nome, rumo_graus)` | gira no lugar até o rumo pedido. Chame a cada quadro até retornar `True`. |
| `parar(nome)` | cancela a navegação e os comandos de velocidade |
| `resetar(nome)` | volta o robô à posição e à pose iniciais |

O `ir_para` é um controle proporcional em linha reta, sem planejamento de caminho: ele **não desvia de
obstáculos**. Se houver algo no caminho (trem de pouso, motor, outro robô), divida a rota em pontos intermediários,
como faz o exemplo. O Spot anda virando para o alvo e indo para a frente, porque a política dele funciona melhor
assim.

### Braço, juntas e garra

| Método | O que faz |
|---|---|
| `juntas(nome, {"junta": rad, ...})` | alvos de posição por nome de junta. Funciona em todo robô com juntas (braço do Spot, braços e tronco do Galbot, etc.). Os nomes aparecem em `robos()` e em `estado(nome)["juntas"]`. |
| `pose_braco(nome, "inspecao" \| "inicial")` | Ridgeback+Franka e Ridgeback+UR5: garra apontando para cima, ou a pose inicial |
| `garra_velocidade(nome, vx, vy, vz, duracao=0.5)` | move a garra em m/s (referencial da base), por cinemática inversa diferencial; Ridgebacks |
| `garra(nome, abrir=True)` | abre ou fecha a garra do Franka |

### Câmeras e fotos

| Método | O que faz |
|---|---|
| `foto(alvo, arquivo, largura=1280, altura=720)` | salva um PNG da câmera de bordo do robô (`alvo` = nome) ou de qualquer câmera da cena (`alvo` = caminho USD, ex. `"/World/Cameras/Panoramica"`). O arquivo fica pronto alguns quadros depois. |
| `selecionar(nome)` | troca o robô ativo (o do teclado e da câmera "seguir") |
| `camera(modo)` | câmera do viewport: 0 = panorâmica, 1 = seguir o robô ativo, 2 = câmera de bordo |

As câmeras de bordo ficam na garra (Franka, UR5), na frente do corpo (Spot) e na frente dos drones. O Galbot não tem.

---

## 4. Servidor TCP/JSON: controle de outro programa

```bash
./executar.sh --servidor            # escuta em 127.0.0.1:5555
./executar.sh --servidor 6000       # outra porta
```

**Protocolo:** cada mensagem é **uma linha** de JSON terminada em `\n`, com o nome do método da API em `"cmd"` e os
argumentos pelo nome. Cada comando recebe uma linha de resposta:

```
→ {"cmd": "estado", "nome": "drone_arl"}
← {"ok": true, "resultado": {"nome": "drone_arl", "pos": [17.0, -6.0, 0.4], "rumo_graus": 90.0, ...}}

→ {"cmd": "velocidade", "nome": "drone_arl", "vz": 1.0, "duracao": 2}
← {"ok": true, "resultado": null}

→ {"cmd": "estado", "nome": "nao_existe"}
← {"ok": false, "erro": "KeyError: \"robô desconhecido: 'nao_existe'. Use robos() para ver a lista.\""}
```

Comandos aceitos: `robos`, `tempo`, `estado`, `velocidade`, `garra_velocidade`, `parar`, `ir_para`, `juntas`,
`pose_braco`, `garra`, `resetar`, `selecionar`, `camera` e `foto`. A resposta de `foto` só chega depois que o arquivo
é salvo, com o caminho completo dele.

Dá para testar à mão com o `nc` (netcat):

```bash
echo '{"cmd": "robos"}' | nc -q 1 127.0.0.1 5555
```

O exemplo [`exemplos/cliente_tcp.py`](exemplos/cliente_tcp.py) usa só a biblioteca padrão do Python (não precisa do
ambiente do Isaac Sim). Ele faz o Ingenuity subir, o drone ARL navegar e fotografar, o braço do UR5 se mover e o Spot
andar 2 m. Como usa `drone_ingenuity`, `drone_arl`, `ridgeback_ur5` e `spot_braco`, rode a cena completa ou carregue
só esses robôs:

```bash
# terminal 1
./executar.sh --robos drone_ingenuity,drone_arl,ridgeback_ur5,spot_braco --servidor
# terminal 2 (depois de aparecer "[servidor] escutando em ...")
python3 exemplos/cliente_tcp.py
```

**Atenção ao tempo.** A simulação pode rodar **mais devagar que o relógio**, dependendo do PC. As durações (`duracao`) são contadas no tempo da simulação. Por isso o cliente de exemplo espera com o comando
`tempo` (função `esperar`), e não com `time.sleep`. Faça o mesmo nos seus programas.

**Segurança.** O servidor escuta só em `127.0.0.1` (o próprio PC). Para aceitar conexões de outra máquina, troque
`host="127.0.0.1"` por `"0.0.0.0"` em `ServidorTCP`, sabendo que não há autenticação.

**ROS 2.** O Isaac Sim tem uma ponte oficial para ROS 2 (extensão `isaacsim.ros2.bridge`). Com ela, câmeras, juntas,
TF e `cmd_vel` viram tópicos ROS, configurados por OmniGraph. Ela não está ligada nesta simulação. Se o plano for
integrar com uma pilha ROS 2 (Nav2, MoveIt), esse é o caminho; até lá, um nó ROS seu pode falar com o servidor TCP.

---

## 5. Gravar: vídeo, fotos e poses em CSV

### Vídeo MP4

```bash
./executar.sh --gravar-video saidas/video.mp4                          # câmera que segue o robô ativo
./executar.sh --gravar-video saidas/video.mp4 --camera-video panoramica
./executar.sh --gravar-video saidas/video.mp4 --camera-video robo --ativo drone_arl   # câmera de bordo do drone
```

O vídeo sai em 1280x720, H.264, e tem a **duração do tempo de simulação**, mesmo que o PC rode mais devagar. O arquivo
é finalizado quando a simulação fecha (fim da missão, `--frames` ou fechamento da janela). Se o processo for morto à
força, o MP4 fica corrompido.

Na interface também dá para gravar pelo menu **Window → Movie Capture**: escolha a câmera, a resolução e a pasta, e
clique em *Capture*. Funciona com qualquer cena, inclusive com o `abrir_cena.sh`.

### Fotos

Use `api.foto(...)` numa missão ou pelo servidor (seção 3). Para imagens fixas do ambiente, `--capturas PASTA`
regenera as imagens da pasta `imagens/` (guia, seção 8).

### Poses em CSV

```bash
./executar.sh --log-csv saidas/poses.csv
```

O CSV tem uma linha por robô a cada 0,1 s de simulação:

```
tempo_s,robo,x,y,z,rumo_graus,roll_graus,pitch_graus
0.016,ridgeback_franka,30.0000,-6.0131,0.0000,90.00,-0.00,-0.00
0.016,drone_arl,17.0000,-6.0084,0.4282,90.00,0.00,0.00
```

Abre no Excel, no LibreOffice ou com `pandas.read_csv`. Para gravar outras coisas (juntas, posição da garra), copie a
classe `LogCSV` em `scripts/api_externa.py` e use `api.estado(nome)`.

---

## 6. Tirar, colocar e alterar coisas na cena

### Sem mexer no código

| Quero... | Como |
|---|---|
| menos robôs | `--robos ridgeback_franka,drone_arl` |
| cena mínima (PC fraco) | `--cena leve`: só o 747, o Ridgeback+Franka e o drone ARL |
| sem a política do Spot | `--sem-spot-ia` (o Spot fica parado) |

### No código (`scripts/inspecao_aviao.py`)

| Quero... | Onde |
|---|---|
| tirar um robô de vez, mudar a posição inicial ou o rumo | lista `SPECS`: apague a linha ou mude `pos=(x, y, z)` e `yaw=` |
| escolher quem entra na `--cena leve` | `leve=True` no `Spec` |
| tirar ou trocar aviões, mudar a posição deles | `build_aircraft`: lista `layout` (o 747 fica sempre; os outros só na cena completa) |
| mudar chão, céu, sol, faixas pintadas | `build_environment` |
| mudar a pose de inspeção do braço | `pose_inspecao=[...]` no `Spec` (radianos, na ordem das juntas do braço) |
| mudar a câmera de bordo | `cam_link`, `cam_pos` e `cam_eixo` no `Spec` |

Para colocar um avião novo, veja o guia, seção 9.

### Pela interface do Isaac Sim

Com a simulação aberta (`./executar.sh`), no painel **Stage** (à direita):

* **Apagar:** selecione o prim (ex.: `/World/Avioes/global_hawk`) e aperte **Delete**.
* **Esconder sem apagar:** clique no ícone de olho ao lado do prim.
* **Desativar** (some e deixa de colidir): botão direito → *Deactivate*, ou desmarque *Active* no painel *Property*.
* **Mover:** selecione e use as ferramentas de mover e girar (teclas W e E no viewport) ou edite *Transform* no
  painel *Property*.

Essas mudanças valem só enquanto a janela estiver aberta. Para que fiquem permanentes, altere o código como na tabela
acima. Para guardar a cena montada em USD:

```bash
./executar.sh --headless --frames 1 --salvar cenas/minha_cena.usda
```

Também dá para editar a cena salva pela interface (`./abrir_cena.sh cenas/minha_cena.usda`, depois *File → Save*).
O arquivo salvo serve para visualizar e editar, mas o controle dos robôs (teclado, missão, servidor) vem do
`inspecao_aviao.py`, que monta a cena do zero a cada execução.

---

## 7. Como o código está organizado

| Arquivo | Papel |
|---|---|
| `scripts/inspecao_aviao.py` | monta a cena, cria os controladores, lê o teclado e roda o laço principal |
| `scripts/api_externa.py` | `API`, `ServidorTCP`, `GravadorVideo`, `LogCSV` e `carregar_missao` |
| `scripts/converter_avioes.py` | converte os modelos da NASA para USD (só quando trocar um avião) |
| `scripts/abrir_cena.py` | abre um USD no Isaac Sim, sem controle |

O `inspecao_aviao.py` é lido de cima para baixo, nesta ordem:

1. **Opções de linha de comando** (`argparse`) e início do Isaac Sim (`SimulationApp`). Os `import` do Isaac Sim
   (`omni.*`, `isaacsim.*`, `pxr`) só funcionam **depois** do `SimulationApp`.
2. **`PHYSICS_DT = 1/500`**: a física roda a 500 Hz, na CPU. É o que a política de locomoção do Spot exige. Se o
   Spot não for usado, dá para baixar para 1/200 ou 1/120 e a simulação fica mais rápida.
3. **`Spec` e `SPECS`**: a ficha de cada robô (nome, arquivo USD, tipo, posição, braço, garra, câmera...).
4. **Montagem da cena**: `build_environment`, `build_aircraft` e depois cada robô da `SPECS` é carregado da nuvem da
   NVIDIA e posicionado. As câmeras de bordo são criadas como filhas de um link do robô.
5. **Controladores**, uma classe por tipo, todas herdando de `Robot`:
   * `Ridgeback`: a base anda por três juntas virtuais (x, y, giro); o braço usa cinemática inversa diferencial
     (mínimos quadrados amortecidos);
   * `Spot`: política de RL oficial da NVIDIA (`RobotPolicyRunner`), que recebe `[vx, vy, wz]`;
   * `Galbot`: base movida cinematicamente; braços, tronco e cabeça pelo modo junta;
   * `Drone`: "position hold" simplificado, com a gravidade do drone desligada e controle por velocidades (sem
     modelo de empuxo das hélices). As colisões continuam valendo.
6. **Teclado** (`read_command`, `handle_events`) e **janela "Controles"** (`build_ui`).
7. **`on_physics(dt)`**: chamada a **cada passo de física** (500 vezes por segundo de simulação). Para cada robô,
   escolhe o comando (teclado, API ou ocioso) e chama o controlador dele.
8. **API, servidor, missão, vídeo e CSV** são criados a partir dos argumentos.
9. **`tick()`**: um **quadro** (cerca de 60 por segundo de simulação). Lê o teclado, atende o servidor, chama
   `passo()` da missão, atualiza a navegação e as fotos (`api.atualizar`), move a câmera seguidora, renderiza
   (`simulation_app.update()`) e grava o vídeo e o CSV.

O comando que chega aos controladores é um dicionário:

```python
{"base": (vx, vy, wz),     # m/s, m/s, rad/s, no referencial do robô
 "vz": 0.0,                # subida, drones
 "ee": (dx, dy, dz),       # velocidade da garra
 "joint": 0.0,             # -1/0/+1 no modo junta
 "fast": False, "toggle_grip": False, "home_arm": False, "inspect_arm": False}
```

`API.comando_para` troca os campos `base`, `vz` e `ee` pelos da API enquanto o comando externo não expira.

---

## 8. Adicionar um robô novo

1. **Ache o USD.** Os robôs da NVIDIA estão em *Window → Browsers → Isaac Sim Assets* ou em
   `.../Assets/Isaac/6.1/Isaac/Robots/` (veja o guia, seção 4.3). O caminho no `Spec` é relativo a `Robots/`.
2. **Acrescente um `Spec`** na lista `SPECS`:

   ```python
   Spec("meu_drone", "Meu drone (descrição para a janela)",
        "Fabricante/Modelo/modelo.usd", "drone", (10.0, -6.0, 0.4), yaw=90,
        cam_link="base_link", cam_pos=(0.2, 0, 0)),
   ```

   Se o tipo for um dos existentes (`ridgeback`, `spot`, `galbot`, `drone`), isso basta. Um Ridgeback com outro
   braço também precisa de `ee` (link da garra), `braco` (lista das juntas) e `pose_inspecao`.
3. **Tipo novo** (ex.: um robô de esteira): crie uma classe herdando de `Robot`, implemente
   `control(self, cmd, dt)` lendo `cmd["base"]` e registre em `CLASSES = {..., "esteira": Esteira}`.
   * Para mover por juntas (rodas): `self.art.set_dof_velocities(...)` ou `set_dof_position_targets(...)`, com os
     índices de `self.dofs`.
   * Para mover a base direto (como o drone): `self.art.set_velocities(lin, ang)`.
   * Ponha em `self.jlist` os índices das juntas que o modo junta (`[ ]` e `- =`) e a `API.juntas` podem mexer.
4. **Teste** com o autoteste, que aperta as teclas virtualmente e mostra se o robô andou, girou e subiu:

   ```bash
   ./executar.sh --headless --robos meu_drone --autoteste
   ```

As teclas `1..8` selecionam os oito primeiros robôs; TAB passa por todos.

**Sensores.** Uma câmera nova é um `UsdGeom.Camera` criado como filho de um link (veja `add_camera`). Para ler a
imagem dela em código, use `api.foto(caminho_da_camera, ...)` ou o `omni.replicator.core` (render product + annotator
`rgb`, como no `GravadorVideo`). O mesmo esquema dá profundidade (`distance_to_image_plane`) e segmentação
(`semantic_segmentation`). LiDAR e IMU estão nas extensões `isaacsim.sensors.*` e na documentação do Isaac Sim.
