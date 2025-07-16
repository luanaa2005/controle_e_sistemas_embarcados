# Explicação do Código: Rastreamento de Cor Azul com HSV (OpenCV)

Este documento explica o funcionamento de um script Python que rastreia objetos com **tonalidade azul** capturados pela webcam, utilizando o **espaço de cor HSV** — mais estável sob diferentes iluminações.

---



### 1. Faixa de Cor em HSV
Ao contrário da versão anterior que usava RGB, este código utiliza **HSV (Hue, Saturation, Value)** — um espaço de cor mais adequado para filtragem de tons de forma consistente.

- `azulEscuro`: vetor HSV com limites inferiores do azul.
- `azulClaro`: vetor HSV com limites superiores do azul.

Esses valores definem a **faixa de tons de azul** que será detectada.

### 2. Captura da Imagem
A webcam é acessada com `cv2.VideoCapture(0)` e cada frame é capturado em um laço contínuo.

### 3. Conversão para HSV
Cada frame capturado é convertido do formato **BGR** (padrão do OpenCV) para **HSV** com:
```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
