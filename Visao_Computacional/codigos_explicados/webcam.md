# Explicação do Código de Captura de Vídeo com OpenCV

Este arquivo descreve o funcionamento do código que utiliza a biblioteca **OpenCV** para capturar vídeo da webcam e exibir em tempo real.

---


### 1. Abertura da Webcam
O programa inicia acessando a **webcam padrão** do computador (`cv2.VideoCapture(0)`). O número `0` indica a primeira câmera disponível.

### 2. Verificação de Acesso
Com `webcam.isOpened()`, o código verifica se a câmera foi acessada corretamente antes de continuar.

### 3. Loop de Captura
Enquanto a webcam estiver funcionando, o código entra em um laço `while` que:
- Captura continuamente frames da webcam.
- Exibe os frames em uma janela chamada **"Video da webcam"**.

### 4. Exibição do Vídeo
Cada frame é exibido com `cv2.imshow(...)`, permitindo ver em tempo real o que está sendo capturado.

### 5. Encerramento com Tecla ESC
O programa escuta por teclas pressionadas com `cv2.waitKey(5)`. Se a tecla **ESC** for detectada, o laço é encerrado e o programa para.

---
