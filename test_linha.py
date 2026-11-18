"""
test_linha.py - mede a ESPESSURA DA LINHA e confere se a visao enxerga a faixa.
NAO liga motor, NAO usa GPIO. Pode rodar com o carro na mao, em cima da pista.

    python3 test_linha.py              # calibracao automatica (padrao)
    python3 test_linha.py --esp 0.6    # forca um valor e mostra quantos frames deram certo
    python3 test_linha.py --cam 0      # outro indice de camera (padrao: camera da faixa)

Como usar na pista:
  1. Ponha o carro NA faixa (as duas linhas visiveis na frente da camera).
  2. Rode. Em ~3 segundos ele imprime a linha medida e o LINHA_ESPESSURA.
  3. Depois ele mostra, a cada 2 s, quantos frames a visao considerou OK com esse
     valor. O ideal e 90% ou mais, com o carro parado na faixa.
  4. Movimente o carro devagar pela pista (curvas, sombra, reflexo) e veja se a
     porcentagem se mantem. Se cair muito, tente --esp com valores menores.
  5. Quer fixar? Copie o numero pro config_urbano.py:  LINHA_ESPESSURA = 0.55
     (sem aspas). Deixando "auto", o main_urbano.py mede de novo a cada partida.
Ctrl+C encerra e imprime o resumo.
"""

import collections
import sys
import time

import cv2

import config
import config_urbano as cfgu
from vision import LaneVision


def _arg(nome, padrao=None):
    if nome in sys.argv:
        i = sys.argv.index(nome)
        if len(sys.argv) > i + 1:
            return sys.argv[i + 1]
    return padrao


def main():
    esp_forcada = _arg("--esp")
    if esp_forcada is not None:
        cfgu.LINHA_ESPESSURA = float(esp_forcada)
    cfgu.aplicar_overrides()

    indice = int(_arg("--cam", cfgu.CAMERA_FAIXA_INDEX))
    cap = cv2.VideoCapture(indice, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(indice)
    if not cap.isOpened():
        print(f"Camera {indice} nao abriu.")
        return
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfgu.CAMERA_RES_FAIXA[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfgu.CAMERA_RES_FAIXA[1])

    vision = LaneVision()
    print(__doc__.split("Como usar")[0])
    print("Lendo a camera... (Ctrl+C encerra)\n")

    motivos = collections.Counter()
    ok = total = 0
    t_rel = time.time()
    janela = collections.deque(maxlen=60)
    avisou_fim = False
    try:
        while True:
            lido, frame_full = cap.read()
            if not lido:
                continue
            frame = cv2.resize(frame_full, config.CAMERA_RES)
            res = vision.process(frame)
            bom = bool(res.valid and res.motivo == "ok")
            janela.append(bom)
            total += 1
            ok += int(bom)
            if not bom:
                motivos[res.motivo.split("(")[0]] += 1

            if vision.linha_espessura is not None and not avisou_fim:
                avisou_fim = True
                print(f"\n>>> LINHA MEDIDA: {vision.linha_largura_px:.0f} px  ->  "
                      f"LINHA_ESPESSURA = {vision.linha_espessura:.2f}\n")

            if time.time() - t_rel >= 2.0:
                t_rel = time.time()
                pct = 100.0 * sum(janela) / max(1, len(janela))
                estado = ("calibrando..." if vision._cal_ativa
                          else f"espessura {config.LINHA_AUTO and vision.linha_espessura or cfgu.LINHA_ESPESSURA}")
                print(f"frames OK nos ultimos {len(janela)}: {pct:3.0f}%  | {estado}")
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()

    print("\n--- RESUMO ---")
    print(f"frames lidos: {total} | OK: {100.0 * ok / max(1, total):.0f}%")
    if motivos:
        print("motivos dos frames ruins:", dict(motivos.most_common(4)))
    if vision.linha_espessura is not None:
        print(f"Linha medida: {vision.linha_largura_px:.0f} px -> "
              f"LINHA_ESPESSURA = {vision.linha_espessura:.2f}")
        print("Pra FIXAR, no config_urbano.py:  "
              f"LINHA_ESPESSURA = {vision.linha_espessura:.2f}")
    elif esp_forcada is None:
        print("A calibracao automatica NAO terminou (poucos frames com linha visivel).")
        print("Confira se as linhas aparecem na imagem e se a camera esta certa (--cam N).")
    if total and ok / total < 0.8:
        print("Poucos frames OK: tente --esp com valor MENOR (linha mais fina), "
              "melhore a luz, ou confira o angulo da camera.")


if __name__ == "__main__":
    main()
