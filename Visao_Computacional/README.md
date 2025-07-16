# Módulo de Visão

Este diretório contém os scripts e experimentos relacionados à visão computacional do projeto. O foco principal é detectar e rastrear uma linha azul capturada pela câmera, permitindo controle autônomo baseado em visão.

---

## 📋 Testes Realizados

| Nº | Objetivo do Teste                                   | Descrição                                                                                     | Código Testado              |
|----|-----------------------------------------------------|------------------------------------------------------------------------------------------------|-----------------------------|
| 1  | Detecção de linha azul   | [Explicação do `teste1.py`](./codigos_explicados/teste1.md)                  | [`teste1.py`](./Testes_e_Validacoes/linha_azul/teste1.py)     |
| 2  | Detecção com ruído de fundo                         | [Explicação do `teste_faixa_hsv.py`](./explicacoes/teste_faixa_hsv.md)                        | `teste_faixa_hsv.py`        |
| 3  | Rastreamento com câmera em movimento                | [Explicação do `rastreamento_linha.py`](./explicacoes/rastreamento_linha.md)                  | `rastreamento_linha.py`     |
| 4  | Teste com diferentes tonalidades de azul            | [Explicação do `teste_faixa_hsv.py`](./explicacoes/teste_faixa_hsv.md)                        | `teste_faixa_hsv.py`        |
| 5  | Performance em tempo real                           | [Explicação do `visao_debug.py`](./explicacoes/visao_debug.md)                                | `visao_debug.py`            |

---

## 💻 Códigos Validados

| Script                        | Finalidade                                            |
|------------------------------|--------------------------------------------------------|
| [`rastreamento_linha.py`](./Codigos_Validos/linha_azul.py)    | Detecção e rastreamento da linha azul. |
| `teste_faixa_hsv.py`         | Testes para calibrar a faixa HSV da cor azul.         |
| `visao_offline.py`           | Análise de vídeos gravados para testes sem câmera ao vivo. |
| `visao_debug.py`             | Versão com visualizações extras para calibração visual. |
