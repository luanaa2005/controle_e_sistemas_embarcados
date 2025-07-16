# Explicação do Código: Rastreamento de Cor Azul com OpenCV

Este documento descreve o funcionamento de um script que rastreia objetos com tonalidades de azul capturados pela webcam, utilizando **OpenCV** e **NumPy**.

---



### 1. Definição da Faixa de Cores (Azul)
São definidas duas cores em formato `numpy.array` que representam os **limites inferior e superior dos tons de azul** que se deseja rastrear:
- `azulEscuro`: limite inferior (tons mais escuros).
- `azulClaro`: limite superior (tons mais claros).

Esses vetores delimitam uma **faixa de cor em RGB** usada para segmentar os objetos azuis.

### 2. Captura de Vídeo
O script acessa a webcam usando `cv2.VideoCapture(0)` e, dentro de um laço infinito, captura os frames em tempo real.

### 3. Segmentação da Cor Azul
Para cada frame:
- É criada uma **imagem binária** (`obj`) com a função `cv2.inRange(...)`, onde só os pixels dentro da faixa de azul aparecem brancos.
- Em seguida, aplica-se um **desfoque Gaussiano** com `cv2.GaussianBlur(...)` para reduzir ruídos e suavizar a imagem.

### 4. Detecção de Contornos
Com `cv2.findContours(...)`, o script detecta os contornos dos objetos visíveis na imagem binária. Se houver contornos:
- O maior contorno (em área) é selecionado.
- Envolve-se esse contorno com o **menor retângulo possível rotacionado** (`cv2.minAreaRect(...)`) e desenha-se na imagem original.

### 5. Exibição das Janelas
São abertas duas janelas:
- `Tracking`: mostra o vídeo com o retângulo ao redor do objeto azul detectado.
- `Binary`: mostra a imagem binária com a máscara aplicada à faixa de cor azul.

### ⌨6. Encerramento
O programa continua rodando até que a tecla `'q'` seja pressionada. Após isso:
- A câmera é liberada com `camera.release()`.
- As janelas abertas são fechadas com `cv2.destroyAllWindows()`.

---

