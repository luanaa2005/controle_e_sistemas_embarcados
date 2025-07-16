# Rastreamento de Linha Azul com OpenCV

Este projeto realiza o rastreamento de uma linha azul usando a webcam do computador. A técnica é baseada na detecção de cor no espaço HSV e no cálculo do centro da linha usando momentos da imagem.


## Descrição do Funcionamento

O código realiza as seguintes etapas:

1. **Captura de vídeo**: Utiliza a webcam (`cv2.VideoCapture(0)`) para obter imagens em tempo real.
2. **Conversão de cor**: A ROI é convertida de BGR para HSV, pois o HSV é mais estável para detectar cores.
3. **Criação de máscara**: É aplicada uma faixa de tons de azul para isolar apenas os pixels da linha azul.
4. **Cálculo do centro da linha**:
    - Usa os momentos da imagem binária para encontrar o centro da linha azul detectada.
    - Desenha um círculo vermelho nesse ponto.
5. **Visualização do desvio**:
    - Mostra a linha central da imagem.
    - Mostra uma linha verde conectando o centro da imagem ao centro da linha azul.
6. **Cálculo do erro**:
    - O erro é a diferença horizontal (em pixels) entre o centro da imagem e o centro da linha.
    - Esse valor pode ser usado por algoritmos de controle (como PID) para ajustar o movimento do drone.

