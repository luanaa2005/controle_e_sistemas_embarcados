# Módulo de Visão

Este diretório contém os scripts e experimentos relacionados à visão computacional do projeto. O foco principal é detectar e rastrear uma linha azul capturada pela câmera, permitindo controle autônomo baseado em visão.

---

## Testes Realizados

| Nº | Objetivo do Teste                                   | Descrição                                                                                     | Código Testado              |
|----|-----------------------------------------------------|------------------------------------------------------------------------------------------------|-----------------------------|
| 1  | Abrir a webcam do computador   | [Explicação do `webcam.py`](./codigos_explicados/webcam.md)                  | [`webcam.py`](./Testes_e_Validacoes/webcam/webcam.py)     |
| 2  | Detecção da linha azul                         | [Explicação do `teste1.py`](./codigos_explicados/teste1.md)                        | [`teste1.py`](./Testes_e_Validacoes/linha_azul/teste1.py)             |
| 3  | Detecção da linha azul                | [Explicação do `teste2.py`](./codigos_explicados/teste2.md)                  | [`teste2.py`](./Testes_e_Validacoes/linha_azul/teste2.py)     |
| 4  | Detecção da linha azul            | [Explicação do `teste3.py`](./codigos_explicados/teste3.md)                        | [`teste3.py`](./Testes_e_Validacoes/linha_azul/teste3.py)        |
| 5  | Detecção da linha azul                           | [Explicação do `teste4.py`](./codigos_explicados/teste4.md)                                | [`teste4.py`](./Testes_e_Validacoes/linha_azul/teste4.py)            |
| 6  | Leitura QR CODE                           | [Explicação do `qrcode.py`](./codigos_explicados/qrcode.md)                                | [`qrcode.py`](./Testes_e_Validacoes/qrcode/qrcode.py)            |
| 7  | Leitura de figuras geométricas                           | [Explicação do `formas.py`](./codigos_explicados/formas.md)                                | [`formas.py`](./Testes_e_Validacoes/formas_geometricas/formas.py)            |

---

## Códigos Validados

| Script                        | Descrição                                            |
|------------------------------|--------------------------------------------------------|
| [`rastreamento_linha.py`](./Codigos_Validos/rastreamento_linha.py)    | [`rastreamento_linha.py`](./codigos_explicados/rastreamento_linha.md) |
| `Leitura qrcode`         | Testes para calibrar a faixa HSV da cor azul.         |
| `Leitura formas geométricas`           | Análise de vídeos gravados para testes sem câmera ao vivo. |

