"""
VISAO SEM CALIBRACAO (scanline / varredura por linhas)
------------------------------------------------------
Nao usa warp, nao usa calibrate_warp.py, nao precisa medir nada.

Ideia: em vez de "achar a faixa", varre N linhas horizontais da imagem.
Em cada linha procura a borda branca mais proxima a ESQUERDA do carro e a
mais proxima a DIREITA. O meio dessas duas = centro da pista naquela altura.

O pulo do gato (o que substitui a calibracao):
a meia-largura da pista em pixels varia com a altura da imagem por causa da
perspectiva. Em vez de voce medir isso na mao, o codigo APRENDE sozinho:
nas linhas onde achou os dois lados, ele guarda (y, meia_largura) e ajusta
uma reta. Nas linhas onde so achou um lado (tipico do pontilhado), ele usa
essa reta pra estimar onde estaria o outro lado.

API identica a vision.LaneVision -> main.py e controller.py nao mudam.
"""

import cv2
import numpy as np
import config


class LaneResult:
    __slots__ = ("error", "heading", "curvature", "found_left", "found_right",
                 "valid", "center_x", "debug_view", "debug_mask",
                 "motivo", "n_pontos", "lado_continuo")

    def __init__(self):
        self.error = 0.0
        self.heading = 0.0
        self.curvature = 0.0
        self.found_left = False
        self.found_right = False
        self.valid = False
        self.center_x = 0.0
        self.debug_view = None
        self.debug_mask = None
        self.motivo = "ok"     # por que perdeu, quando valid=False
        self.n_pontos = 0      # quantas linhas varridas deram ponto valido
        self.lado_continuo = None   # 'esquerda' | 'direita' | None (ainda indefinido)


class LaneVision:
    def __init__(self):
        self.W, self.H = config.CAMERA_RES
        self._k_open = None
        self._blur_k = 5
        self._k_line = None
        self._cal_ativa = bool(getattr(config, "LINHA_AUTO", False))
        self._monta_kernels()

        # ---- CALIBRACAO AUTOMATICA DA ESPESSURA DA LINHA ------------------
        # Mede a largura (px) dos trechos brancos numa faixa de linhas da imagem
        # (~30% da altura da area de varredura) nos primeiros frames, tira a
        # mediana e traduz em LINHA_ESPESSURA. Enquanto mede, a visao roda com parametros
        # TOLERANTES (config_urbano.aplicar_overrides), que funcionam de linha
        # fina a grossa. Zebra (muitas barras) e frames sem linha nao contam.
        self._cal_larguras = []
        self._cal_frames = 0
        self.linha_largura_px = None     # largura medida (None ate terminar)
        self.linha_espessura = None      # espessura calculada (None ate terminar)
        self._k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))


        # modelo aprendido: meia_largura(y) = a*y + b   (em pixels)
        self.hw_model = None
        self.smooth_error = 0.0
        self.last_center = self.W * config.CAR_CENTER_PCT
        self.lost_frames = 0
        self.last_thr = 0.0
        self.white_frac = 0.0
        self.ref_e = None
        self.ref_d = None
        self.sat_frames = 0
        self.smooth_heading = 0.0
        self.last_mask = None

        # ---- deteccao automatica de qual lado e a faixa CONTINUA ----
        # Ideia: a faixa pontilhada tem BURACOS. Rodando varias linhas de
        # varredura (scanlines) por frame, o lado pontilhado vai ficar sem
        # deteccao em uma fracao das linhas (caiu no vao entre tracos),
        # enquanto o lado continuo e detectado em quase todas. Isso nao
        # depende de qual lado (esquerda/direita da imagem) e qual faixa -
        # funciona igual nos dois sentidos da pista.
        self.smooth_taxa_e = 0.5    # taxa de deteccao suavizada, lado esquerdo
        self.smooth_taxa_d = 0.5    # taxa de deteccao suavizada, lado direito
        self.lado_continuo = None   # 'esquerda' | 'direita' | None -- so texto/HUD
        self._sinal_continuo = 0.0  # -1..+1, o que REALMENTE define o vies (continuo)

        # TRAVA da decisao. Sem isso, um trecho ruim no meio da execucao
        # (rastreio perdendo um lado por causa de esterco violento, camera
        # engasgando, etc) derruba a taxa de deteccao DAQUELE lado por um
        # motivo que nao tem nada a ver com o padrao real da faixa -- e o
        # codigo interpretava isso como troca de lado, o alvo pulava pro
        # lado oposto da pista, e o carro saia. Uma vez confiante, a decisao
        # trava pro resto da execucao e para de reagir a chacoalhao.
        self._frames_consistentes = 0
        self._ultimo_sinal_consistente = 0   # -1, 0 ou +1
        self._sinal_travado = None    # None ate travar; depois fica -1.0 ou +1.0

    def _monta_kernels(self):
        """(Re)cria os kernels que dependem da espessura (config.*)."""
        _ko = int(getattr(config, "VISAO_OPEN_K", 3))
        self._k_open = np.ones((_ko, _ko), np.uint8) if _ko > 1 else None
        self._blur_k = int(getattr(config, "VISAO_BLUR_K", 5))
        # kernel do top-hat: precisa ser MAIOR que a largura da faixa,
        # senao ele apaga a propria faixa junto com o fundo.
        # Enquanto MEDE a linha usa um kernel grande (linha de ate ~40 px): com
        # o kernel normal uma linha grossa perto do carro seria apagada e a
        # medida sairia fina demais.
        larg = max(config.LINE_MAX_WIDTH_PX, 40) if self._cal_ativa else config.LINE_MAX_WIDTH_PX
        k = larg * 2 + 1
        self._k_line = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))

    def _calibra_espessura(self, mask):
        """Acumula a largura da linha perto do carro; ao juntar amostras
        suficientes (ou estourar o tempo), ajusta a visao."""
        self._cal_frames += 1
        h = mask.shape[0]
        larguras = []
        n_runs = 0
        # Mede em ~30% da altura da area de varredura (contando de cima): ali as
        # duas linhas ja estao DENTRO do quadro e ainda tem largura util. Perto
        # do carro (baixo) a linha sai pelos lados da imagem e a medida
        # encolhe; no fundo ela vira 1-2 px e some no ruido.
        for f in (0.26, 0.30, 0.34):
            row = mask[int(f * (h - 1))]
            idx = np.flatnonzero(row)
            if idx.size == 0:
                continue
            grupos = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
            for g in grupos:
                w = len(g)
                n_runs += 1
                if 3 <= w <= 90:
                    larguras.append(w)
        # zebra (barras demais) ou nada na imagem: nao serve como amostra
        if larguras and n_runs <= 14:
            self._cal_larguras.append(float(np.median(larguras)))

        pronto = len(self._cal_larguras) >= config.LINHA_CAL_FRAMES
        estourou = self._cal_frames >= config.LINHA_CAL_TIMEOUT
        if not (pronto or estourou):
            return
        self._cal_ativa = False       # (_monta_kernels abaixo volta ao kernel normal)
        if len(self._cal_larguras) >= max(10, config.LINHA_CAL_FRAMES // 3):
            self.linha_largura_px = float(np.median(self._cal_larguras))
            esp = self.linha_largura_px / float(config.LINHA_REF_PX)
            esp = config.aplicar_espessura(esp)
            origem = "auto"
        else:
            esp = config.aplicar_espessura(config.LINHA_ESP_FALLBACK)
            origem = "AUTO FALHOU (poucas amostras) -> valor seguro"
        self.linha_espessura = esp
        self._monta_kernels()
        medida = (f"{self.linha_largura_px:.0f} px (a 30% da altura da imagem de varredura)"
                  if self.linha_largura_px else "sem medida")
        print(f"  [visao] linha medida: {medida} -> LINHA_ESPESSURA = {esp:.2f} ({origem}) | "
              f"trecho>={config.SCAN_MIN_RUN_PX}px linhas>={config.SCAN_MIN_ROWS} "
              f"blur {config.VISAO_BLUR_K} abertura {config.VISAO_OPEN_K} | luz "
              f"{'adaptativa' if config.LUZ_ADAPTATIVA else 'antiga'}")

    # ------------------------------------------------------------------
    def threshold(self, roi):
        """
        Segmentacao INVARIANTE A ILUMINACAO.

        Nao pergunta "esse pixel e claro?" (isso depende da luz do ambiente e
        e o que fazia o Otsu eleger o chao inteiro no escuro). Pergunta
        "esse pixel e mais claro que a VIZINHANCA dele?" - o que continua
        verdade tanto no sol quanto na sombra.

        Operador: TOP-HAT = imagem - abertura(imagem).
        A abertura com um kernel maior que a faixa apaga a faixa e deixa so o
        fundo/gradiente de luz. Subtraindo, sobra so estrutura clara e fina.
        """
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        S = hsv[:, :, 1]
        V = hsv[:, :, 2]

        gray0 = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = (cv2.GaussianBlur(gray0, (self._blur_k, self._blur_k), 0)
                if self._blur_k > 1 else gray0)
        adaptativa = bool(getattr(config, "LUZ_ADAPTATIVA", False))
        if adaptativa:
            # fundo local = abertura; top-hat = imagem - fundo
            fundo = cv2.morphologyEx(gray, cv2.MORPH_OPEN, self._k_line)
            tophat = cv2.subtract(gray, fundo)
            # LIMIAR RELATIVO: a linha precisa ser mais clara que o fundo DAQUELE
            # PONTO numa fracao do brilho dele. Em sombra o fundo e escuro e o
            # limiar cai junto (a linha continua passando); em reflexo/area
            # clara o fundo e claro e o limiar sobe (o brilho do reflexo nao
            # vira linha). O fator do tunel (TOPHAT_FATOR menor) afrouxa isso.
            rel = config.LUZ_REL * (config.TOPHAT_FATOR / 0.45)
            limiar_local = rel * (fundo.astype(np.float32) + config.LUZ_C)
            thr = float(config.LUZ_TOPHAT_MIN)
            v_min = min(config.LUZ_V_MIN, config.V_MIN_ABS)
        else:
            tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, self._k_line)
            # limiar automatico: percentil do proprio frame, nao numero fixo
            thr = max(float(config.TOPHAT_MIN),
                      float(np.percentile(tophat, config.TOPHAT_PERCENTIL)) * config.TOPHAT_FATOR)
            limiar_local = None
            v_min = config.V_MIN_ABS

        # trava de seguranca: faixa nunca ocupa 1/4 da tela.
        # Se ocupou, o limiar estava baixo demais -> sobe e tenta de novo.
        mask = None
        for _ in range(6):
            lim = thr if limiar_local is None else np.maximum(thr, limiar_local)
            mask = ((tophat >= lim) &
                    (S <= config.SAT_MAX) &
                    (V >= v_min)).astype(np.uint8) * 255
            if mask.mean() / 255.0 <= config.MAX_WHITE_FRAC:
                break
            thr *= 1.3

        self.last_thr = thr
        self.white_frac = mask.mean() / 255.0

        if self._k_open is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._k_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._k_close)
        return mask

    @staticmethod
    def _sem_reflexo(mask):
        """Tira da mascara DA FAIXA as manchas que parecem reflexo: compactas
        (quadradas) ou deitadas (horizontais) e cheias. Faixa e riscada
        (fina e comprida, ou diagonal com pouco preenchimento). Pequenas
        (traco longe) passam. A mascara que vai pra ZEBRA nao e filtrada."""
        n, lab, st, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n <= 1:
            return mask
        out = mask.copy()
        amin = config.LUZ_BLOB_AMIN
        for i in range(1, n):
            x, y, w, h, a = st[i]
            if a < amin:
                continue
            cheio = a / float(w * h)
            if cheio >= 0.55 and ((w < 2.2 * h and h < 2.2 * w) or w > 2.5 * h):
                out[lab == i] = 0
        return out

    @staticmethod
    def _runs(row):
        """Retorna [(x_ini, x_fim)] dos trechos brancos da linha."""
        idx = np.flatnonzero(row)
        if idx.size == 0:
            return []
        cortes = np.flatnonzero(np.diff(idx) > 1)
        grupos = np.split(idx, cortes + 1)
        return [(int(g[0]), int(g[-1])) for g in grupos
                if len(g) >= config.SCAN_MIN_RUN_PX]

    # ------------------------------------------------------------------
    def process(self, frame, debug=False):
        res = LaneResult()

        y0 = int(self.H * config.SCAN_TOP_PCT)      # corta ceu/fundo
        roi = frame[y0:, :]
        mask = self.threshold(roi)
        # expoe a mascara pro crosswalk.py (detector de faixa de pedestre)
        # reaproveitar em vez de recalcular o top-hat -- economiza ~5ms/frame.
        # Nao altera nada do comportamento da visao.
        self.last_mask = mask          # BRUTA: e o que a zebra usa
        if self._cal_ativa:
            self._calibra_espessura(mask)
        if getattr(config, "LUZ_BLOB", False):
            mask = self._sem_reflexo(mask)
        roi_h = mask.shape[0]

        ys = np.linspace(roi_h - 5, 5, config.SCAN_ROWS).astype(int)

        centros = []          # (y_roi, x_centro, confianca)
        pares = []            # (y_roi, meia_largura) -> alimenta o modelo
        car_x = self.W * config.CAR_CENTER_PCT

        # Referencias de rastreio: onde cada faixa estava no frame anterior.
        ref_e, ref_d = self.ref_e, self.ref_d
        jan = config.SCAN_TRACK_WINDOW

        # contadores pra deteccao automatica do lado continuo (ver __init__)
        n_rows_validas = 0
        n_hit_e = 0
        n_hit_d = 0

        # ys vem de BAIXO pra CIMA. Perto do carro a faixa e confiavel, entao a
        # identidade (esquerda/direita) e definida ali e PROPAGADA pra cima
        # seguindo a curva. E isso que impede a faixa de "trocar de lado"
        # quando ela cruza o centro do quadro numa curva forte.
        for y in ys:
            runs = self._runs(mask[y])
            if not runs:
                continue

            n_rows_validas += 1
            xe = xd = None
            i_e = i_d = -1

            if ref_e is None or ref_d is None:
                # sem historico: unica vez que usa o centro do carro como corte
                esq = [(i, r) for i, r in enumerate(runs) if r[1] < car_x]
                dirr = [(i, r) for i, r in enumerate(runs) if r[0] > car_x]
                if esq:
                    i_e, r = max(esq, key=lambda p: p[1][1]); xe = r[1]
                if dirr:
                    i_d, r = min(dirr, key=lambda p: p[1][0]); xd = r[0]
            else:
                # rastreio local: trecho branco mais proximo de onde essa
                # faixa estava na linha de baixo
                ce = [(i, r) for i, r in enumerate(runs) if abs(r[1] - ref_e) <= jan]
                cd = [(i, r) for i, r in enumerate(runs) if abs(r[0] - ref_d) <= jan]
                if ce:
                    i_e, r = min(ce, key=lambda p: abs(p[1][1] - ref_e)); xe = r[1]
                if cd:
                    i_d, r = min(cd, key=lambda p: abs(p[1][0] - ref_d)); xd = r[0]

                # as duas referencias agarraram o MESMO trecho: e uma faixa so.
                # Fica com a mais proxima e descarta a outra.
                if i_e >= 0 and i_e == i_d:
                    if abs(xe - ref_e) <= abs(xd - ref_d):
                        xd, i_d = None, -1
                    else:
                        xe, i_e = None, -1

            if xe is not None:
                n_hit_e += 1
            if xd is not None:
                n_hit_d += 1

            # ---- achou os dois: linha de ouro, alimenta o modelo de largura
            if xe is not None and xd is not None:
                largura = (xd - xe) / 2.0
                if config.SCAN_HW_MIN < largura < config.SCAN_HW_MAX:
                    centros.append((y, (xe + xd) / 2.0, 1.0))
                    pares.append((y, largura))
                    res.found_left = res.found_right = True
                    ref_e, ref_d = xe, xd
                continue

            if xe is None and xd is None:
                continue

            # ---- so um lado: completa com a meia-largura aprendida
            hw = self._half_width(y)
            if hw is None:
                # ainda sem modelo: nao inventa centro, mas mantem o rastreio vivo
                if xe is not None:
                    ref_e = xe
                if xd is not None:
                    ref_d = xd
                continue

            if xe is not None:
                centros.append((y, xe + hw, 0.6))
                res.found_left = True
                ref_e, ref_d = xe, xe + 2 * hw
            else:
                centros.append((y, xd - hw, 0.6))
                res.found_right = True
                ref_d, ref_e = xd, xd - 2 * hw

        # guarda a base do rastreio pro proximo frame
        self.ref_e, self.ref_d = ref_e, ref_d

        # ---- deteccao automatica do lado continuo x pontilhado ----
        # ANTES: decisao binaria (vira "esquerda" OU "direita" de uma vez).
        # Isso fazia o alvo SALTAR de +mag*hw pra -mag*hw no frame exato em
        # que a decisao virava -- foi o que tirou o carro da pista. Agora
        # self._sinal_continuo e um numero continuo entre -1 e +1: perto de
        # 0 quando os dois lados estao parecidos (sem bias, fica centralizado
        # -- o mais seguro quando ha duvida), e so chega em +-1 quando um
        # lado esta CLARAMENTE mais solido que o outro. Como ele nasce de
        # smooth_taxa_e/d (que ja e um EMA), a transicao de um lado pro
        # outro e sempre gradual, nunca um degrau.
        if n_rows_validas >= config.SCAN_MIN_ROWS:
            taxa_e = n_hit_e / n_rows_validas
            taxa_d = n_hit_d / n_rows_validas
            b = config.LADO_CONTINUO_SUAVIZACAO
            self.smooth_taxa_e = b * self.smooth_taxa_e + (1 - b) * taxa_e
            self.smooth_taxa_d = b * self.smooth_taxa_d + (1 - b) * taxa_d

            diff = self.smooth_taxa_e - self.smooth_taxa_d

            if self._sinal_travado is not None:
                # JA TRAVADO: ignora qualquer flutuacao de taxa daqui pra
                # frente. Um trecho ruim no meio do percurso nao pode mais
                # virar a decisao.
                self._sinal_continuo = self._sinal_travado
            else:
                # positivo = precisa deslocar o alvo pra DIREITA (continua e a direita)
                self._sinal_continuo = max(-1.0, min(
                    1.0, -diff / config.LADO_CONTINUO_MARGEM))

                # conta frames CONSISTENTES (mesmo lado, com folga confortavel
                # acima da margem normal) pra decidir travar. Qualquer
                # inconsistencia zera a contagem -- so trava com sinal limpo.
                if abs(diff) > config.LADO_CONTINUO_TRAVA_MARGEM:
                    # mesmo sinal que _sinal_continuo usa (-diff/MARGEM)
                    sinal_atual = 1 if diff < 0 else -1
                    if sinal_atual == self._ultimo_sinal_consistente:
                        self._frames_consistentes += 1
                    else:
                        self._ultimo_sinal_consistente = sinal_atual
                        self._frames_consistentes = 1
                else:
                    self._frames_consistentes = 0
                    self._ultimo_sinal_consistente = 0

                if self._frames_consistentes >= config.LADO_CONTINUO_TRAVA_FRAMES:
                    self._sinal_travado = float(self._ultimo_sinal_consistente)
                    self._sinal_continuo = self._sinal_travado
                    # trava o ROTULO tambem -- ele deve bater com a decisao
                    # real, nao ficar recalculando por conta propria depois
                    self.lado_continuo = "direita" if self._sinal_travado > 0 else "esquerda"

            # rotulo so pra telemetria/HUD -- so atualiza enquanto AINDA NAO
            # travou. Depois de travado, o bloco acima ja fixou o rotulo e
            # este trecho e pulado (fica so o texto condizente com a trava).
            if self._sinal_travado is None:
                if diff > config.LADO_CONTINUO_MARGEM:
                    self.lado_continuo = "esquerda"
                elif -diff > config.LADO_CONTINUO_MARGEM:
                    self.lado_continuo = "direita"
                # diferenca pequena: mantem o ultimo rotulo, so texto mesmo
        res.lado_continuo = self.lado_continuo

        # ---- atualiza o modelo de meia-largura
        if len(pares) >= 2:
            py = np.array([p[0] for p in pares], dtype=float)
            ph = np.array([p[1] for p in pares], dtype=float)
            novo = np.polyfit(py, ph, 1)
            # trava anti-deriva: o modelo so aceita ajuste pequeno por frame,
            # senao ele infla sozinho (cada frame usa a hw anterior pra formar
            # pares, que confirmam a hw inflada... e vira bola de neve)
            if self.hw_model is None:
                self.hw_model = novo
            else:
                self.hw_model = 0.9 * self.hw_model + 0.1 * novo
        elif len(pares) == 1 and self.hw_model is None:
            self.hw_model = np.array([0.0, pares[0][1]])

        # ---- sem informacao suficiente
        res.n_pontos = len(centros)
        if len(centros) < config.SCAN_MIN_ROWS:
            self.lost_frames += 1
            res.valid = self.lost_frames <= config.MAX_FRAMES_PERDIDO
            res.error = self.smooth_error
            res.motivo = f"poucos_pontos({len(centros)}<{config.SCAN_MIN_ROWS})"
            if debug:
                res.debug_view, res.debug_mask = self._draw(roi, mask, centros, None)
            return res

        self.lost_frames = 0

        cy = np.array([c[0] for c in centros], dtype=float)
        cx = np.array([c[1] for c in centros], dtype=float)
        cw = np.array([c[2] for c in centros], dtype=float)

        # peso extra pras linhas mais distantes (lookahead) mas sem ignorar o capo
        peso = cw * np.linspace(1.0, config.SCAN_PESO_LONGE, len(cy))[np.argsort(cy)[::-1].argsort()]

        media = float(np.average(cx, weights=peso))
        y_min, y_max = float(cy.min()), float(cy.max())
        span = y_max - y_min

        # NUNCA extrapolar: o alvo so pode ficar dentro da faixa de alturas que
        # o robo REALMENTE enxergou. Extrapolar parabola com poucos pontos
        # manda o alvo pra fora da tela (era o ponto verde colado na borda).
        y_look = roi_h * (1.0 - config.LOOKAHEAD_PCT)
        y_look = max(y_min, min(y_max, y_look))

        # grau 2 exige pontos suficientes E bem espalhados; senao, reta.
        grau = 2 if (len(cy) >= config.FIT_MIN_PTS_GRAU2 and
                     span >= roi_h * config.FIT_MIN_SPAN_PCT) else 1

        alvo = media
        raw_heading = 0.0
        try:
            fit = np.polyfit(cy, cx, grau, w=peso)
            cand = float(np.polyval(fit, y_look))
            # sanidade: o alvo tem que estar perto dos pontos observados
            if cx.min() - config.FIT_MARGEM <= cand <= cx.max() + config.FIT_MARGEM:
                alvo = cand
                if grau == 2:
                    raw_heading = 2 * fit[0] * y_look + fit[1]
                    res.curvature = float(abs(fit[0]))
                else:
                    raw_heading = fit[0]
        except (np.linalg.LinAlgError, ValueError):
            pass

        # TRAVA DE ESTABILIDADE DO HEADING.
        # O heading (dx/dy) alimenta o feed-forward do PID. Sem limite, um fit
        # ruim (poucos pontos, curva no limiar grau1/grau2) gera valores tipo
        # 3.4 quando o plausivel e proximo de +-1 - isso INVERTE o sinal do
        # esterco mesmo com o erro estavel. Grampeia e suaviza, igual ao erro.
        raw_heading = max(-config.HEADING_MAX, min(config.HEADING_MAX, raw_heading))
        self.smooth_heading = (config.HEADING_SUAVIZACAO * self.smooth_heading +
                               (1 - config.HEADING_SUAVIZACAO) * raw_heading)
        res.heading = self.smooth_heading

        # Se o alvo caiu FORA da imagem, o rastreio esta furado. Nao vale gerar
        # erro +-1 com confianca total: melhor declarar perdido.
        if not (0 <= alvo <= self.W):
            self.ref_e = self.ref_d = None
            self.lost_frames += 1
            res.valid = self.lost_frames <= config.MAX_FRAMES_PERDIDO
            res.error = self.smooth_error
            res.center_x = float(np.clip(alvo, 0, self.W))
            res.motivo = f"alvo_fora_da_tela({alvo:.0f})"
            if debug:
                res.debug_view, res.debug_mask = self._draw(roi, mask, centros, None)
            return res

        # ---- VIES DE FAIXA: andar mais perto da faixa CONTINUA que do centro.
        # Desloca o ALVO (nao o carro) em fracao da meia-largura da pista.
        # LANE_BIAS_FRAC agora e so a MAGNITUDE do vies (0..LANE_BIAS_LIMITE).
        # O SINAL (pra que lado desloca) vem de self.lado_continuo, que e
        # aprendido sozinho olhando qual lado tem buracos (pontilhado) e qual
        # nao tem (continuo) -- por isso funciona igual com a faixa continua
        # na esquerda ou na direita, sem precisar mudar nada no config.
        # Sempre escala com a largura real vista no frame -> funciona em
        # qualquer trecho, reto ou curva, sem virar valor fixo em pixel.
        if config.LANE_BIAS_FRAC != 0.0:
            hw_look = self._half_width(y_look)
            if hw_look is not None:
                mag = min(config.LANE_BIAS_LIMITE, abs(config.LANE_BIAS_FRAC))
                # O SINAL de LANE_BIAS_FRAC escolhe o lado:
                #   POSITIVO -> aproxima da PONTILHADA (usado na corrida:
                #               encurta a trajetoria)
                #   NEGATIVO -> aproxima da CONTINUA (usado no urbano: a
                #               pontilhada e a divisoria de mao dupla, entao
                #               colar nela e andar na beira da contramao)
                sentido = -1.0 if config.LANE_BIAS_FRAC > 0 else 1.0
                # lado da continua: aprendido (padrao) ou FORCADO pelo
                # config (urbano = mao direita -> continua sempre a direita).
                # Forcar elimina o risco de a visao adivinhar o lado errado
                # logo apos uma curva. Na corrida o atributo nao existe.
                sinal = self._sinal_continuo
                forcado = getattr(config, "LADO_CONTINUO_FORCADO", None)
                if forcado == "direita":
                    sinal = 1.0
                elif forcado == "esquerda":
                    sinal = -1.0
                alvo += sentido * sinal * mag * hw_look
                alvo = max(0.0, min(float(self.W), alvo))

        self.last_center = 0.6 * self.last_center + 0.4 * media

        car_x = self.W * config.CAR_CENTER_PCT
        raw = max(-1.0, min(1.0, (alvo - car_x) / (self.W / 2.0)))

        a = config.ERRO_SUAVIZACAO
        self.smooth_error = a * self.smooth_error + (1 - a) * raw
        res.error = self.smooth_error

        # ---- TRAVA ANTI-SATURACAO
        # Erro colado em +-1 por muitos frames nao e curva: e rastreio perdido
        # (tipico de faixa trocando de lado). Melhor admitir que se perdeu do
        # que virar o esterco todo com confianca total.
        if abs(res.error) >= config.SAT_LIMITE:
            self.sat_frames += 1
        else:
            self.sat_frames = 0

        if self.sat_frames > config.MAX_FRAMES_SATURADO:
            self.ref_e = self.ref_d = None      # zera o rastreio e reengata
            self.sat_frames = 0
            self.smooth_error = 0.0             # nao carrega o erro podre adiante
            res.valid = False
            res.center_x = alvo
            res.motivo = "saturado"
            return res

        res.center_x = alvo
        res.valid = True

        if debug:
            res.debug_view, res.debug_mask = self._draw(
                roi, mask, centros, (alvo, y_look, car_x))
        return res

    # ------------------------------------------------------------------
    def _half_width(self, y):
        if self.hw_model is None:
            return None
        hw = float(np.polyval(self.hw_model, y))
        if hw < config.SCAN_HW_MIN or hw > config.SCAN_HW_MAX:
            return None
        return hw

    def _draw(self, roi, mask, centros, alvo):
        vis = roi.copy()
        for (y, x, conf) in centros:
            cor = (0, 255, 0) if conf >= 1.0 else (0, 165, 255)
            cv2.circle(vis, (int(x), int(y)), 3, cor, -1)
        if alvo is not None:
            ax, ay, car_x = alvo
            cv2.circle(vis, (int(car_x), vis.shape[0] - 8), 6, (0, 0, 255), -1)
            cv2.circle(vis, (int(ax), int(ay)), 7, (0, 255, 0), -1)
            cv2.line(vis, (int(car_x), vis.shape[0] - 8), (int(ax), int(ay)),
                     (255, 255, 255), 2)
        return vis, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
