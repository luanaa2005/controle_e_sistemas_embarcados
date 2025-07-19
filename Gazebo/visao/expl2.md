

-----

## Estrutura do Código

O código é implementado como uma classe Python chamada `QRCodeDetector`, que herda de `rclpy.node.Node`, integrando-o perfeitamente ao ambiente ROS 2.

### 1\. Inicialização (`__init__`)

No construtor da classe `QRCodeDetector`:

  * **Subscrição de Imagem**: O nó se inscreve no tópico da imagem da câmera, por padrão configurado para `/camera/image_raw`. **É vital que você verifique e ajuste este tópico** para que corresponda exatamente ao nome do tópico de imagem que a câmera do seu drone no Gazebo está publicando (use `ros2 topic list` para confirmar).
  * **Publicação de Dados do QR Code**: Um publicador é configurado para enviar mensagens do tipo `geometry_msgs/Point` para o tópico `/drone/qr_code_data`.
      * O campo `x` da mensagem `Point` será usado para o **ID numérico do QR Code**.
      * Os campos `y` e `z` da mensagem `Point` armazenarão as **coordenadas X e Y do centro do QR Code na imagem**, respectivamente.
  * **`CvBridge`**: Uma instância de `CvBridge` é criada. Essa ponte é indispensável para converter mensagens de imagem ROS 2 (do tipo `sensor_msgs/Image`) para o formato de matrizes NumPy que o OpenCV utiliza para processamento.
  * **Flag de Controle**: `self.qr_code_detected_and_published` é uma flag que garante que o nó publique as informações de um QR Code apenas uma vez. Isso evita o reprocessamento constante do mesmo QR Code, especialmente útil em cenários onde você só precisa da primeira detecção para progredir na missão.

### 2\. Callback de Imagem (`image_callback`)

Esta é a função principal do nó, executada a cada nova imagem recebida da câmera:

  * **Controle de Publicação**: Verifica `self.qr_code_detected_and_published`. Se já detectou e publicou um QR Code, o callback é ignorado para aquela imagem, otimizando o processamento.
  * **Conversão da Imagem**: A imagem recebida (no formato ROS) é convertida para uma imagem OpenCV em BGR (Azul, Verde, Vermelho) usando `CvBridge`.
  * **Conversão para Tons de Cinza**: A imagem é então convertida para escala de cinza (`cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)`). Essa conversão é recomendada e otimiza o desempenho de bibliotecas de decodificação de códigos de barras, como `pyzbar`, pois o contraste é mais importante que a cor para a detecção de padrões.
  * **Decodificação com `pyzbar`**: A função `decode(gray_image)` da biblioteca `pyzbar` é utilizada para escanear a imagem em busca de códigos de barras e QR Codes.
  * **Processamento de QR Codes Detectados**:
      * Se `decoded_objects` não estiver vazio, o código itera sobre cada objeto encontrado.
      * Ele verifica se o `obj.type` é realmente um `'QRCODE'`.
      * Os dados decodificados do QR Code (`obj.data`) são convertidos de bytes para uma string UTF-8 (`qr_data.decode('utf-8')`).
      * Uma tentativa é feita para converter `qr_data` para um **inteiro (`qr_id`)**. Se a conversão falhar (indicando que o conteúdo do QR Code não é um ID numérico), a detecção é ignorada.
      * As coordenadas do **retângulo delimitador (`obj.rect`)** do QR Code são extraídas.
      * O **centro do QR Code na imagem (`center_x`, `center_y`)** é calculado a partir do retângulo delimitador.
  * **Publicação dos Dados**:
      * Uma nova mensagem `geometry_msgs/Point` é criada.
      * `point_msg.x` recebe o `qr_id` (o ID numérico do QR Code).
      * `point_msg.y` recebe `center_x` (a coordenada horizontal do centro do QR Code na imagem).
      * `point_msg.z` recebe `center_y` (a coordenada vertical do centro do QR Code na imagem).
      * A mensagem é então publicada no tópico `/drone/qr_code_data`.
      * A flag `self.qr_code_detected_and_published` é definida como `True`, garantindo que este QR Code não seja republicado desnecessariamente.

---