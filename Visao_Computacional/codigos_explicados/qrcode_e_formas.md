# Explicação do Código de Leitura de QR Code e Detecção de Formas e Cores com OpenCV

Este programa realiza duas etapas:

1. **Leitura de um QR Code pela webcam**
2. **Detecção em tempo real de formas geométricas coloridas específicas, definidas pelo QR Code lido**

---

## Funcionamento detalhado:

### 1. Leitura do QR Code

- O programa inicia capturando vídeo pela webcam.
- Utiliza o detector `QRCodeDetector` do OpenCV para tentar localizar e decodificar um QR Code em cada frame.
- Quando um QR Code válido é detectado, seu valor é armazenado e exibido no terminal.
- Se o QR Code lido não for "1", "2" ou "3", o programa informa que o código é inválido e encerra.
- Os valores "1", "2" e "3" correspondem a alvos pré-definidos de forma e cor que o programa deve buscar:
  - "1": Quadrado marrom
  - "2": Triângulo azul
  - "3": Hexágono vermelho

### 2. Detecção de formas e cores ao vivo

- Após definir a forma e cor alvo com base no QR Code, o programa começa a processar os frames da webcam para detectar contornos.
- Cada frame é convertido para escala de cinza, borrado para reduzir ruídos, e passa pelo detector de bordas Canny.
- São encontrados os contornos externos para análise.
- Para cada contorno maior que uma área mínima:
  - O contorno é aproximado para simplificar sua forma e calcular o número de vértices.
  - Um retângulo delimitador é extraído para recortar a área da forma no frame original.
  - A região recortada é convertida para o espaço de cores HSV para facilitar a detecção das cores.
  - Máscaras são criadas para as cores azul, marrom e vermelho (com dois intervalos para o vermelho).
  - A porcentagem de pixels em cada máscara é calculada para identificar a cor predominante da forma.
  - A forma geométrica é classificada como triângulo, quadrado/retângulo ou hexágono, conforme o número de vértices.
- Se a forma e a cor detectadas correspondem àquelas definidas pelo QR Code, o contorno é destacado em verde na imagem, e o programa indica que o alvo foi detectado com sucesso, exibindo o resultado.

### 3. Controle e encerramento

- O programa exibe o vídeo processado em tempo real, mostrando a detecção das formas.
- A tecla ESC encerra a execução a qualquer momento.
- Após detectar com sucesso o alvo, o programa também encerra, liberando os recursos da webcam e fechando as janelas abertas.

---

## Resumo

Este código combina leitura de QR Code com processamento de imagens para buscar, ao vivo, uma forma geométrica colorida específica definida pelo código. Ele usa técnicas de detecção de contornos, análise da forma geométrica pelo número de vértices e identificação da cor dominante no espaço HSV para reconhecer os objetos desejados na cena capturada pela webcam.

---

