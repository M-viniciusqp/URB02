# Guia da pista oficial (sem internet) — versão "linha fina"

Esta pasta é a versão **experimental**. A original continua intacta na outra pasta
(`~/URB`). No Pi, cada versão fica na sua pasta e você roda a que quiser:

```bash
cd ~/URB-linha-fina && python3 main_urbano.py     # esta versão (linha fina, calibra sozinha)
cd ~/URB && python3 main_urbano.py                # a original
```

Comparar as duas na pista: rode a mesma volta nas duas e veja qual perde menos a faixa
(`PERDIDO` no log), para menos na zebra errada e faz as curvas certas.

## O que é AUTOMÁTICO nesta versão
- **Espessura da linha**: nos primeiros ~2 s de execução o robô mede a largura da linha e
  ajusta a visão sozinho. Nada para editar (`LINHA_ESPESSURA = "auto"`).
- **Luz** (sombra/reflexo): liga sozinha o limiar relativo ao brilho local quando a linha
  medida não é fina (`LUZ_ADAPTATIVA_URBANO = None`). Com linha fina usa o método antigo,
  que foi igual ou melhor nos testes.
- Tudo o que já era automático continua (modelo de largura da pista, lado da contínua, etc.).

## Ao ligar, olhe no log esta linha
```
[visao] linha medida: 12 px (a 30% da altura ...) -> LINHA_ESPESSURA = 0.59 (auto) | trecho>=2px ...
```
- Apareceu em ~2-3 s → calibrou. Anote o número (ex.: 0.59).
- Apareceu `AUTO FALHOU (poucas amostras) -> valor seguro` → não viu linha suficiente nos
  primeiros segundos (carro fora da faixa/luz ruim). Ele usa 0.6, que é tolerante e serve.
- Não apareceu nada → você fixou um número no config (modo manual).

## Teste rápido da visão (sem motor)
Com o carro NA faixa:
```bash
python3 test_linha.py
```
Mostra a linha medida e a % de frames OK (o ideal é 90% ou mais). Movimente o carro devagar
pela pista (curva, sombra, reflexo) e veja se a % se mantém. `python3 test_linha.py --esp 0.5`
força um valor para você comparar.

## Como editar no Pi (sem internet)
```bash
nano config_urbano.py        # Ctrl+W procura o nome do parâmetro | Ctrl+O + Enter salva | Ctrl+X sai
```
Depois rode de novo o `main_urbano.py`. Valores de teste ficam só nesse arquivo.

## SINTOMA → O QUE MUDAR (todos em `config_urbano.py`)

### Visão / linha
| O que você vê | O que fazer |
|---|---|
| Muitos `PERDIDO(poucos_pontos)` e o carro para/hesita | `LINHA_ESPESSURA = 0.45` (menor = linha mais fina). Teste 0.45, 0.4. |
| Carro trepida/zigue-zague, segue ruído, linha é grossa | `LINHA_ESPESSURA = 1.0` ou maior (1.3). |
| O "auto" mediu estranho (muito diferente do que você vê) | Fixe: `LINHA_ESPESSURA = 0.6` (sem aspas). Ou ajuste `LINHA_REF_PX` (maior = acha a linha mais fina). |
| Sombra/reflexo faz o carro desviar | `LUZ_ADAPTATIVA_URBANO = True` e teste. Se piorar, `False`. |
| Reflexo vira "linha falsa" | `LUZ_FILTRA_REFLEXO_URBANO = True` (arrisca perder faixa perto do carro; teste devagar). |
| Carro anda no lado errado da pista | `LADO_CONTINUO_ESPERADO` (direita/esquerda) e `LANE_BIAS_FRAC_URBANO` (0.50 = mais pro lado da pontilhada; 0.35 = mais centro). |

### Placas
| O que você vê | O que fazer |
|---|---|
| `[placa] ... DESCARTADO: LONGE demais` | A distância está superestimada. Rode `python3 test_signs.py --calib 0.50` (marcador a 50 cm da lente), cole o `FOCAL_PX` que ele imprimir. |
| `placas: -` o tempo todo, nenhum `[placa]` | A câmera não vê o marcador: ângulo/posição da câmera, luz. Rode `python3 test_signs.py`. |
| `!! ID N detectado ... NAO esta no mapa PLACAS` | Corrija o mapa `PLACAS` (ID → tipo). |
| Placa errada/trocada | Confirme os IDs com `python3 test_signs.py --scan`. |

### Zebra e parada
| O que você vê | O que fazer |
|---|---|
| Passa direto pela zebra | `SIGN_PARADA_DIST_M` maior (0.65): para pela placa mais cedo. `ZEBRA_PARADA_FRAC` menor (0.20 já é o mínimo). |
| Para cedo demais antes da zebra | `SIGN_PARADA_DIST_M` menor (0.40). |
| Para de novo na zebra de saída do cruzamento | `ZEBRA_NOVA_MIN_S` maior (2.5). |
| Passa direto pelo cruzamento seguinte (rua curta) | `ZEBRA_NOVA_MIN_S` menor (1.5). |
| Fica parado tempo demais/de menos | `PARADA_TEMPO_S` (padrão 5 s). Depois da ré espera no mínimo `RE_ESPERA_MIN_S` (3 s). |

### Curvas e ré
| O que você vê | O que fazer |
|---|---|
| Curva passa dos 90° | `VELOCIDADE_MANOBRA` menor (0.18). |
| Curva fica curta | `VELOCIDADE_MANOBRA` maior (0.20) ou `MANOBRA_CURVA_DIR_S` / `MANOBRA_CURVA_ESQ_S` maiores. |
| Curva à direita raspa na placa | `MANOBRA_ABERTURA_DIR_S` maior (0.55). Se invadir a contramão, menor (0.48). |
| Termina a curva à direita na faixa errada | `MANOBRA_CURVA_DIR_EXTRA_S` maior (0.30; servo fica mais tempo à direita no fim da curva). |
| Ré sai torta | `python3 test_re.py -0.10` (traseira vai pra direita → valor negativo; esquerda → positivo). Cole em `RE_TURN`. |
| Ré curta demais/longa demais | `RE_ESQ_S` / `RE_DIR_S` (0.6 s). |
| Motor não tem força / zumbe parado | `python3 test_piso.py` acha o duty mínimo; cole os valores `VELOCIDADE_*`. |

## Ordem sugerida na pista
1. `python3 test_piso.py` (se a bateria/piso mudou) → velocidades.
2. `python3 test_linha.py` com o carro na faixa → confere a linha e a % de frames OK.
3. `python3 test_signs.py --calib 0.50` → `FOCAL_PX`; depois `python3 test_signs.py` para ver as placas.
4. `python3 test_re.py` → `RE_TURN` (ré reta).
5. `python3 main_urbano.py` → uma volta devagar; leia o log com a tabela acima.

## Voltar ao que funcionava
Qualquer valor que você mudar e não gostar: volte ao anterior no `config_urbano.py`. Para
descartar tudo, use a pasta original (`~/URB`), que não foi alterada.
