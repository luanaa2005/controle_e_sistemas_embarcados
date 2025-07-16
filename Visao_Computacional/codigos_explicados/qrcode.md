# Explicação do Código: Geração e Leitura de QR Codes com OpenCV e qrcode

Este script Python realiza duas tarefas principais:  
1. Gera imagens de QR Codes com valores simples (1, 2, 3).  
2. Usa a webcam e o OpenCV para detectar e decodificar QR Codes em tempo real.

---



### 1. Geração de QR Codes

Antes de iniciar a leitura com a câmera, o código gera automaticamente três QR Codes com os valores "1", "2" e "3", salvando-os como imagens PNG. Isso é feito com a biblioteca `qrcode`, útil para testes locais.

---

### 2. Inicialização da Webcam

A webcam é iniciada com `cv2.VideoCapture(0)`, e o laço principal começa a capturar frames continuamente, desde que a câmera esteja funcionando corretamente.

---

### 3. Detector de QR Code

O código utiliza o `cv2.QRCodeDetector()` para detectar e decodificar automaticamente qualquer QR Code presente no frame. Esse detector retorna três informações:
- O valor decodificado do QR Code.
- Os pontos (vértices) do QR Code detectado.
- A imagem transformada, se necessário (não usada neste script).

---

### 4. Verificação e Decodificação

Se um QR Code for detectado, o texto contido nele (`valor`) é extraído. Em seguida:
- O texto é exibido na tela usando `cv2.putText`.
- Os pontos (vértices) do QR Code podem ser usados para determinar sua posição na imagem, embora o script atual utilize apenas parte desses dados.

---

### 5. Visualização

O frame da webcam é exibido continuamente com os dados do QR Code sobrepostos, caso detectado. A janela de vídeo é atualizada em tempo real até que o usuário pressione a tecla **ESC** (código 27) para encerrar.

---

### 6. Encerramento

Ao final, a câmera é liberada com `webcam.release()` e todas as janelas abertas são fechadas com `cv2.destroyAllWindows()`.

---
