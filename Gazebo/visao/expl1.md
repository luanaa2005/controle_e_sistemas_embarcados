
# Nó de Detecção de Linha (`line_detector_node.py`)


## Visão Geral

O `line_detector_node.py` é um nó ROS 2 que consome dados de imagem da câmera do drone e aplica técnicas de **visão computacional** (usando a biblioteca OpenCV) para detectar a posição central de uma linha de cor azul. Ele então publica essa informação em um tópico ROS 2, permitindo que outros nós (como o `takeoff_node.py`) usem esses dados para navegação e controle.

Este nó será responsável por:

* **Assinar tópicos de imagem**: Ele receberá os quadros de vídeo da câmera do drone (geralmente do tópico `/camera/image_raw` ou similar).
* **Processamento de Imagem**: Aplicará técnicas de visão computacional (como OpenCV) para identificar a linha azul. Isso pode envolver:
    * Conversão de espaço de cor (ex: BGR para HSV).
    * Definição de limiares de cor para isolar o azul.
    * Detecção de contornos ou uso de Momentos para encontrar o centro da linha.
* **Publicar a posição da linha**: Ele publicará a posição X da linha (em pixels ou normalizado) em um tópico ROS 2, como `/drone/line_center`, que o `takeoff_node.py` já está configurado para assinar.

---

## Estrutura do Código

O código é implementado como uma classe Python chamada `LineDetector`, que herda de `rclpy.node.Node`, integrando-o ao ecossistema ROS 2.

### 1. Inicialização (`__init__`)

No construtor da classe `LineDetector`:

* **Subscrição de Imagem**: O nó se inscreve no tópico de imagem da câmera (padrão `/camera/image_raw`). É crucial verificar e ajustar este tópico para corresponder ao tópico real da câmera do seu drone no Gazebo.
* **Publicação da Posição da Linha**: Um publicador é configurado para enviar mensagens do tipo `geometry_msgs/Point` no tópico `/drone/line_center`. Este tópico é assinado pelo seu `takeoff_node.py`.
* **CvBridge**: Uma instância de `CvBridge` é criada. Esta ferramenta é essencial para converter as mensagens de imagem do ROS 2 (formato `sensor_msgs/Image`) para o formato de matrizes do OpenCV (`numpy.ndarray`), que é necessário para o processamento de imagem.

### 2. Callback de Imagem (`image_callback`)

Esta é a função central do nó, acionada sempre que uma nova imagem é recebida do tópico da câmera:

* **Conversão da Imagem**: A imagem recebida (no formato ROS) é convertida para o formato OpenCV (BGR - Azul, Verde, Vermelho) usando `CvBridge`.
* **Processamento para Detecção de Linha Azul**:
    * **Conversão para HSV**: A imagem é convertida do espaço de cores BGR para HSV (Hue, Saturation, Value). O HSV é mais robusto para a detecção de cores sob diferentes condições de iluminação.
    * **Definição de Limites de Cor**: São definidos dois arrays NumPy (`lower_blue` e `upper_blue`) que representam os limites inferior e superior para a cor azul no espaço HSV. Pixels cujos valores HSV caem dentro desse intervalo são considerados "azuis". Estes valores podem exigir ajuste fino dependendo da tonalidade específica do azul da linha no seu ambiente simulado ou real, e das condições de iluminação.
    * **Criação de Máscara**: A função `cv2.inRange()` utiliza os limites HSV para criar uma máscara binária. Nesta máscara, os pixels que correspondem à cor azul definida são marcados como branco (255), e todos os outros pixels como preto (0).
    * **Operações Morfológicas (Opcional, mas Recomendado)**: Operações de **erode** (erosão) e **dilate** (dilatação) são aplicadas à máscara. A erosão ajuda a remover pequenos ruídos ou pontos isolados, enquanto a dilatação ajuda a expandir e conectar regiões azuis que podem ter sido fragmentadas. Isso resulta em uma máscara mais limpa e coerente.
    * **Detecção de Contornos**: A função `cv2.findContours()` é usada para identificar as bordas das regiões brancas na máscara (ou seja, as regiões que correspondem à linha azul).
    * **Cálculo do Centro da Linha**: Se contornos forem encontrados, o maior contorno (assumindo que a linha é o maior objeto azul) é selecionado. A função `cv2.moments()` calcula os momentos geométricos do contorno, que são usados para determinar o centroide (`cx`, `cy`) do contorno. A coordenada `cx` (posição X do centro da linha na imagem) é o valor-chave.
* **Publicação da Posição**:
    * Uma mensagem `geometry_msgs/Point` é criada.
    * `point_msg.x` é preenchido com a `line_center_x` detectada. Se nenhuma linha for encontrada, `line_center_x` permanece como `-1.0`.
    * Opcionalmente, `point_msg.y` e `point_msg.z` podem ser preenchidos com a largura e altura da imagem, respectivamente, fornecendo mais contexto para o nó de controle.
    * A mensagem `point_msg` é então publicada no tópico `/drone/line_center`.


