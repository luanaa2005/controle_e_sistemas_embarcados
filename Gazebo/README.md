# Nós para a missão no Gazebo



O fluxo da missão ocorre da seguinte forma:

1.  **Inicialização e Decolagem:**
    * O nó de controle inicia a sequência enviando comandos para o PX4 para armar os motores e entrar no modo `Offboard`.
    * A primeira ordem é um setpoint de posição para decolar a uma altitude definida, por exemplo, 1.5 metros, e manter a posição (hover). O nó de controle envia continuamente este setpoint enquanto o PX4 gerencia a subida.

2.  **Fase 1: Navegação por Linha:**
    * Uma vez estabilizado, o controle passa a escutar o tópico `/drone/line_center`, alimentado pelo nosso nó **"observador de linha"**.
    * Este nó de visão analisa o feed da câmera inferior e publica o deslocamento lateral da linha azul em relação ao centro do drone.
    * O nó de controle utiliza essa informação para ajustar a velocidade lateral do drone, gerando novos setpoints para o PX4, garantindo que o drone siga a linha com precisão enquanto avança.

3.  **Transição de Missão: Detecção de QR Code:**
    * Paralelamente, o nó **"detetive de QR Code"** processa o mesmo feed de vídeo, procurando por marcadores.
    * Ao detectar e decodificar um QR Code (ex: "número 2"), ele publica essa informação no tópico `/drone/qr_code_data`.
    * O recebimento desta mensagem funciona como um **gatilho** para o nó de controle, que finaliza a fase de seguimento de linha.

4.  **Fase 2: Navegação para a Zona de Entrega:**
    * Com a informação do QR Code, o nó de controle consulta uma tabela de mapeamento pré-definida para determinar as coordenadas do próximo objetivo. Por exemplo, "QR Code 2" corresponde à base de entrega localizada "5 metros ao Norte".
    * O nó de controle calcula as coordenadas globais do destino e começa a enviar uma nova sequência de setpoints de posição para o PX4, guiando o drone até a área de entrega.

5.  **Fase 3: Identificação do Alvo e Entrega:**
    * Ao se aproximar da área de destino, o nó de controle começa a processar os dados do nosso terceiro nó de visão, o **"identificador de bases"**.
    * Este nó detecta formas e cores (ex: "hexágono vermelho", "triângulo azul") e publica suas descobertas.
    * O nó de controle compara a detecção com o alvo esperado para aquela missão (baseado no QR Code 2, esperávamos um "triângulo azul").
    * Quando o alvo correto é detectado e o drone está centralizado sobre ele, o nó de controle envia o comando final: acionar o atuador para **liberar a carga**.

Após a confirmação da entrega, o nó de controle pode comandar o pouso ou o retorno à base, concluindo a operação.





## Nós

| Script                        | Descrição                                            |
|------------------------------|--------------------------------------------------------|
| [`takeoff_node.py`](./controle/takeoff_node.py)    | [`controle.py`](./controle/explicacao.md) |
| [`line_detector_node.py`](./visao/line_detector_node.py)         | [`linha.py`](./visao/expl1.md)     |
| [`qr_code_detector_node.py`](./visao/qr_code_detector_node.py)         | [`qrcode.py`](./visao/expl2.md) |
| [`base_detector_node.py`](./visao/base_detector_node.py)         | [`base.py`](./visao/expl3.md) |

