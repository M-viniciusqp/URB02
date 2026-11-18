"""
config.py - todos os parametros do robo num lugar so.
Nenhum outro arquivo tem numero magico dentro.
"""

import numpy as np

# =====================================================================
# 1. HARDWARE (pinos BCM)
# =====================================================================
ENA_A = 5;  IN1_A = 6;  IN2_A = 13     # motor esquerdo
ENA_B = 17; IN3_B = 22; IN4_B = 27     # motor direito
SERVO_PIN = 18

# =====================================================================
# 2. SERVO (graus)
# SERVO_CENTRO nao e o meio geometrico: e o angulo em que a roda fica
# REALMENTE reta. Descubra com: python3 test_servo.py
# =====================================================================
SERVO_CENTRO   = 40
SERVO_MAX_ESQ  = 0
SERVO_MAX_DIR  = 90
SERVO_DEADBAND_DEG = 1.0   # so manda pulso se mudou mais que isso (mata o jitter)

STEER_INVERT = False

# =====================================================================
# 3. VELOCIDADE (0.0 a 1.0)
# =====================================================================
VELOCIDADE_BASE    = 1.00   # No maximo. O freio automatico por FPS baixo e
                            # o freio de curva fechada continuam ativos.
VELOCIDADE_MINIMA  = 0.45   # subiu de 0.35 -- piso normal de curva
VELOCIDADE_PERDIDO = 0.22   # perdeu a pista, andando devagar procurando

FPS_MINIMO_SEGURO = 15.0

# =====================================================================
# 4. CAMERA
# =====================================================================
CAMERA_RES = (480, 240)

# =====================================================================
# 5. SEGMENTACAO DA FAIXA BRANCA (top-hat, invariante a iluminacao)
# =====================================================================
LINE_MAX_WIDTH_PX = 18
TOPHAT_PERCENTIL  = 98
TOPHAT_FATOR      = 0.45
TOPHAT_MIN        = 18
SAT_MAX           = 90
V_MIN_ABS         = 70
MAX_WHITE_FRAC    = 0.22

# =====================================================================
# 6. VARREDURA (scanline)
# =====================================================================
SCAN_TOP_PCT    = 0.35
SCAN_ROWS       = 14
SCAN_MIN_RUN_PX = 4
SCAN_MIN_ROWS   = 6
SCAN_HW_MIN     = 25
SCAN_HW_MAX     = 160
SCAN_PESO_LONGE = 2.0

# --- ESPESSURA DA LINHA E LUZ (defaults = comportamento antigo) -----------
# O perfil urbano (config_urbano.py) liga e calcula estes valores a partir de
# LINHA_ESPESSURA; a corrida (main.py) nao muda.
VISAO_BLUR_K   = 5       # kernel do blur antes do top-hat
VISAO_OPEN_K   = 3       # abertura da mascara (apaga ruido; 1 = sem abertura)
LUZ_ADAPTATIVA = False   # True: limiar RELATIVO ao brilho local (reflexo/sombra)
LUZ_REL        = 0.45    # limiar = REL * (brilho_do_fundo + C)
LUZ_C          = 25.0
LUZ_V_MIN      = 35      # brilho minimo absoluto no modo adaptativo
LUZ_TOPHAT_MIN = 12      # contraste minimo absoluto no modo adaptativo
LUZ_BLOB       = False   # True: descarta manchas compactas/horizontais (reflexo)
LUZ_BLOB_AMIN  = 120     # area minima (px) pra uma mancha ser candidata a reflexo
LUZ_AUTO       = False   # True: o modo adaptativo liga sozinho conforme a espessura medida

# CALIBRACAO AUTOMATICA DA ESPESSURA DA LINHA (vision.py mede no inicio da execucao)
LINHA_AUTO       = False  # True: o robo mede a espessura nos primeiros segundos
LINHA_REF_PX     = 20.0   # largura (px, a 30% da altura da area de varredura, imagem 480x240) que vale ESPESSURA 1.0
LINHA_CAL_FRAMES = 60     # amostras validas pra fechar a medida (~2 s)
LINHA_CAL_TIMEOUT = 300   # frames sem medida suficiente -> usa LINHA_ESP_FALLBACK
LINHA_ESP_FALLBACK = 0.6  # valor seguro (tolerante) se nao conseguir medir
LINHA_RECAL_PERDIDOS = 75  # frames SEGUIDOS sem faixa valida (depois de calibrar) -> mede de novo
LINHA_RECAL_MAX      = 2   # no maximo isso de re-calibracoes por execucao


def aplicar_espessura(esp):
    """UNICA traducao espessura -> parametros da visao (usada pelo urbano e pela
    calibracao automatica). esp 1.0 = parametros de sempre. Menor = linha mais
    fina (mais tolerante); maior = mais grossa. O kernel do top-hat NUNCA fica
    abaixo de 18: a zebra tem barras largas e kernel pequeno a apagaria."""
    global LINE_MAX_WIDTH_PX, SCAN_MIN_RUN_PX, SCAN_MIN_ROWS, VISAO_BLUR_K, VISAO_OPEN_K
    global LUZ_ADAPTATIVA
    esp = max(0.3, min(3.0, float(esp)))
    LINE_MAX_WIDTH_PX = max(18, int(round(18 * esp)))
    SCAN_MIN_RUN_PX   = max(2, int(round(4 * esp)))
    SCAN_MIN_ROWS     = 6 if esp >= 0.8 else 5
    VISAO_BLUR_K      = 3 if esp < 0.8 else (5 if esp < 1.6 else 7)
    VISAO_OPEN_K      = 1 if esp < 0.8 else (3 if esp < 1.6 else 5)
    if LUZ_AUTO:
        LUZ_ADAPTATIVA = esp >= 0.8
    return esp
SCAN_TRACK_WINDOW = 95

HEADING_MAX          = 1.2
HEADING_SUAVIZACAO   = 0.55

FIT_MIN_PTS_GRAU2 = 6
FIT_MIN_SPAN_PCT  = 0.40
FIT_MARGEM        = 60

LOOKAHEAD_PCT = 0.55
CAR_CENTER_PCT = 0.50

# =====================================================================
# 6b. REACAO EM CURVA
# =====================================================================
FEEDFORWARD_HEADING   = 0.0
SPEED_TURN_PESO       = 1.2    # baixou um pouco de 1.3 -- menos freio pelo
                                # esterco atual, junto com o piso mais alto
SPEED_CURVATURA_PESO  = 75.0   # baixou de 90.0 -- mesma logica

CURVA_FECHADA_LIMIAR     = 0.004
CURVA_FECHADA_RANGE      = 0.014
VELOCIDADE_MINIMA_FECHADA = 0.38   # subiu de 0.32 -- piso mais alto so pra
                                    # curva bem fechada tambem

# =====================================================================
# 7. CONTROLE (PID)
# Ainda no valor conservador validado (KP=0.50). O procedimento de
# calibracao completo (secao 10) pode liberar um KP mais alto -- isso
# permite corrigir mais forte e rapido, o que deixa CURVAS mais rapidas
# sem precisar abrir mao de margem de seguranca. Vale fazer.
# =====================================================================
PID_KP = 0.62               # baixou de 1.25 -- o teste ESTATICO (carro na
                            # mao) nao revelou instabilidade, mas rodando de
                            # verdade na pista ele oscilou (erro -0.99 seguido
                            # de +0.50 no proximo instante) e saiu da faixa.
                            # 1.25 dinamico = Ku de verdade. Este valor e
                            # Ku*0.5, a regra classica de sempre. So volte a
                            # subir depois de confirmar ESTAVEL rodando na
                            # pista (nao so no teste parado).
PID_KI = 0.0
PID_KD = 0.12                # baixou de 0.15 -- estava dominando o comando
                            # em cima de erro pequeno, com KP=1.25
PID_D_SUAVIZACAO = 0.55      # subiu de 0.3 -- mais filtro na derivada, pra
                            # segurar o balanco sem voltar ao problema antigo
                            # de atraso empilhado (ERRO_SUAVIZACAO continua em
                            # 0.3, entao nao esta dobrando filtro pesado)
PID_I_LIMIT = 0.4
ERRO_SUAVIZACAO = 0.3
DEAD_ZONE = 0.04

# =====================================================================
# 8. SEGURANCA
# =====================================================================
MAX_FRAMES_PERDIDO = 12

# =====================================================================
# 9. VIES DE FAIXA - andar mais perto da faixa PONTILHADA que do centro
# =====================================================================
LANE_BIAS_FRAC   = 0.85     # subiu de 0.70 -- bem mais perto do pontilhado
LANE_BIAS_LIMITE = 0.90     # subiu de 0.80 -- abriu mais espaco pro vies.
                            # ATENCAO: isso ja come bastante da margem de
                            # seguranca. Perto do limite, qualquer falha
                            # momentanea de deteccao do pontilhado vira erro
                            # grande rapido, porque o alvo ja esta quase
                            # colado nele.

LADO_CONTINUO_SUAVIZACAO = 0.93
LADO_CONTINUO_MARGEM     = 0.15

# TRAVA da decisao de qual lado e continuo. Sem isso, um trecho ruim no meio
# do percurso (esterco violento perdendo o rastreio de um lado, camera
# engasgando) podia derrubar a taxa de deteccao daquele lado por um motivo
# que nao tem nada a ver com o padrao real da faixa, e o codigo TROCAVA de
# decisao no meio do percurso -- o alvo pulava pro lado oposto da pista.
# Agora, uma vez confiante (folga maior que a margem normal, sustentada por
# varios frames seguidos), a decisao trava de vez e para de reagir a
# chacoalhao no resto da execucao.
LADO_CONTINUO_TRAVA_MARGEM = 0.25   # folga maior que LADO_CONTINUO_MARGEM
                                     # pra comecar a contar pra travar
LADO_CONTINUO_TRAVA_FRAMES = 40     # frames CONSISTENTES seguidos pra travar

SAT_LIMITE = 0.95
MAX_FRAMES_SATURADO = 5


# =====================================================================
# 10. COMO CALIBRAR O PID NO HARDWARE DE VERDADE (pra ganhar velocidade
# de verdade em curva, nao so via piso/freio)
#
# PASSO 1 -- so P (KI=0, KD=0)
#   python3 test_control.py (carro na mao, tracao desligada)
#   Suba PID_KP aos poucos ate o servo comecar a balancar sozinho.
#   Anote esse valor -- e o "Ku".
# PASSO 2 -- PID_KP = Ku * 0.5
# PASSO 3 -- sobe PID_KD aos poucos ate corrigir rapido sem passar do ponto
# PASSO 4 -- so se sobrar erro parado numa reta, sobe PID_KI devagar
#
# Um KP maior valido = correcao mais forte = pode manter mais velocidade
# em curva sem sair. E o proximo ganho real de velocidade depois deste.
# =====================================================================
