

# Código do Nó de Controle Principal (`takeoff_node.py`) e Sua Comunicação

## Visão Geral

O `takeoff_node.py` é um nó ROS 2 que se comunica com o PX4 (o "piloto automático" do drone) e com os nós de visão. Sua função principal é receber dados de telemetria e visão, processá-los e, com base no estado atual da missão, enviar os comandos para o drone.

## Estrutura Detalhada do Código

### 1. Definição dos Estados da Missão

A classe `MissionState` é o mapa da jornada do drone, definindo os nove estágios sequenciais da missão:

-   **INITIAL**: O drone aguarda o momento certo para ser armado e entrar no modo de controle autônomo (Offboard).
-   **TAKEOFF**: A fase em que o drone decola verticalmente, subindo até a altitude pré-definida de 1.5 metros.
-   **HOVER_AT_ALTITUDE**: Após decolar, o drone paira na altitude alvo para estabilizar e se preparar para a próxima fase.
-   **FOLLOW_LINE**: O drone segue a linha azul no chão, usando dados fornecidos por um nó de visão. Enquanto segue a linha, ele também procura por um QR Code.
-   **NAVIGATE_TO_DELIVERY_ZONE**: Uma vez que o QR Code é detectado, o drone calcula as coordenadas da zona de entrega (relativas ao QR Code) e voa diretamente para essa área.
-   **SEARCH_DELIVERY_BASE**: Chegando à zona de entrega, o drone começa a procurar pela figura geométrica específica, aguardando a detecção do nó de visão.
-   **DROP_PACKAGE**: Ao identificar a base correta e se posicionar sobre ela, o drone executa o comando para soltar o pacote.
-   **LANDING**: Após a entrega, o drone se desloca para um ponto de pouso seguro (definido como as coordenadas iniciais do drone, 0,0,0 no Gazebo) e inicia a descida controlada.
-   **MISSION_COMPLETE**: O estado final, indicando que a missão foi concluída com sucesso, com o drone desarmado no chão.

### 2. Inicialização (`__init__`)

No construtor da classe `OffboardControl`:

#### Configuração de Qualidade de Serviço (QoS)
Define como as mensagens ROS 2 são tratadas para a comunicação com o PX4.

#### Publicadores (Publishers)
Canais de comunicação para o drone enviar instruções.

-   `/fmu/in/offboard_control_mode`: Garante que o drone permaneça no modo Offboard para receber comandos.
-   `/fmu/in/trajectory_setpoint`: Envia as coordenadas (X, Y, Z) e a orientação (Yaw) que o drone deve alcançar.
-   `/fmu/in/vehicle_command`: Usado para comandos gerais como armar/desarmar e mudar o modo de voo.

#### Assinantes (Subscribers)
Canais de comunicação para o drone receber informações.

-   `/fmu/out/vehicle_local_position`: Monitora a posição atual do drone no ambiente (Gazebo).
-   `/fmu/out/vehicle_status`: Acompanha o estado do drone (armado, desarmado, modo de navegação).
-   `/drone/line_center`: Recebe a posição da linha detectada pelo nó de visão da linha.
-   `/drone/qr_code_data`: Recebe o ID e a posição do QR Code detectado.
-   `/drone/base_data`: Recebe o tipo e a posição da base de entrega detectada.

#### Atributos de Estado e Controle
Variáveis que armazenam o estado atual da missão (`self.current_state`), a altura alvo de voo (`self.target_height`), informações capturadas pelos nós de visão, e as coordenadas dos pontos-chave da missão.

-   `self.delivery_zones`: Um dicionário que mapeia o ID de cada QR Code para os offsets de X e Y (deslocamentos em metros) em relação à posição do QR Code, indicando a localização da zona de entrega.
-   `self.expected_base_types`: Mapeia o ID do QR Code ao tipo de base esperado (por exemplo, "TRIANGLE_BLUE") que o drone deve procurar naquela zona de entrega.
-   **Timer Principal**: Um timer configurado para chamar a função `timer_callback` a cada 0.1 segundos (10 Hz). Esta função é o coração do loop de controle da máquina de estados.

### 3. Funções de Callback (Receptoras de Dados)

Essas funções são ativadas automaticamente quando novas mensagens chegam aos tópicos que o nó está assinando:

-   `vehicle_local_position_callback`, `vehicle_status_callback`: Atualizam as variáveis internas do nó com a telemetria do drone.
-   `line_position_callback`, `qr_code_callback`, `base_detection_callback`: Recebem e armazenam os dados de detecção dos seus respectivos nós de visão. É importante notar que o `qr_code_callback` e `base_detection_callback` capturam a posição do drone no momento da detecção para cálculos futuros.

### 4. Funções de Publicação (Enviadoras de Comandos)

Estas funções são métodos auxiliares para enviar comandos e setpoints de forma padronizada para o PX4:

-   `publish_offboard_control_mode()`: Envia continuamente a mensagem para manter o modo Offboard ativo no PX4.
-   `publish_trajectory_setpoint(x, y, z, yaw)`: A função mais usada para comandar o drone a voar para uma posição específica (X, Y, Z) com uma orientação (Yaw) desejada.
-   `publish_vehicle_command(command, param1, param2)`: Usada para comandos MAVLink de baixo nível, como armar/desarmar os motores ou mudar o modo de voo.
-   `calculate_delivery_target_coordinates(qr_id)`: Uma função inteligente que usa o ID do QR Code detectado e a posição atual do drone para calcular as coordenadas absolutas X, Y e Z da zona de entrega no ambiente do Gazebo.