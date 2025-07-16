# Explicação do Código ROS2 para Seguidor de Linha Azul

Este código implementa um **nó ROS2** em Python que realiza o seguimento de uma linha azul utilizando imagens capturadas pela câmera do drone e envia comandos de velocidade para controlar seu movimento.

---

## Funcionamento do Código

### 1. Inicialização do Nó

- O nó é criado com o nome `'line_follower'`.
- Um objeto `CvBridge` é instanciado para converter imagens do formato ROS para OpenCV.
- O nó se inscreve no tópico `/drone/camera/image_raw` para receber as imagens capturadas pela câmera do drone.
- Um publicador é criado para enviar comandos de velocidade (`Twist`) no tópico `/mavros/setpoint_velocity/cmd_vel_unstamped`, que controla o movimento do drone.

---

### 2. Processamento da Imagem (Callback)

- Cada vez que uma nova imagem é recebida, a função `image_callback` é chamada.
- A mensagem de imagem ROS é convertida para uma imagem OpenCV no formato BGR.
- A imagem é convertida do espaço de cor BGR para HSV, que facilita a detecção de cores específicas.
- É criada uma máscara binária que isola os pixels na faixa de azul definida pelos limites HSV `lower_blue` e `upper_blue`.
- São calculados os momentos da máscara para encontrar o centro da área azul detectada.

---

### 3. Cálculo do Erro e Controle

- Se a área azul detectada for suficientemente grande (`m00 > 1000`), o código calcula a coordenada horizontal do centro da linha azul.
- O erro é definido como a diferença entre o centro da imagem e o centro da linha azul detectada.
- Um comando de velocidade é criado:
  - Velocidade linear constante para frente (`0.3` m/s).
  - Velocidade angular proporcional ao erro, para corrigir o alinhamento do drone com a linha azul.
- Este comando é publicado para controlar o drone.

---

### 4. Perda da Linha Azul

- Caso a linha azul não seja detectada em um frame, um contador de perda é incrementado.
- Se a linha ficar perdida por mais de 10 frames consecutivos, o drone recebe comandos para parar o movimento.

---

### 5. Visualização para Depuração

- A máscara binária que destaca a cor azul é exibida em uma janela com OpenCV para facilitar a visualização e depuração durante a execução.

---

### 6. Execução do Nó

- A função `main` inicializa o ROS2, cria o nó `LineFollower` e mantém a execução até que o nó seja encerrado.

---

