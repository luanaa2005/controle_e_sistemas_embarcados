# CargaPowerNode: Controle de Liberação de Carga

Este documento descreve o funcionamento do nó `CargaPowerNode`, responsável liberar uma carga do drone. Ele atua como uma interface entre o nó de controle (`offboard_control.py`) e o hardware físico no drone.

## 1. Visão Geral

O `CargaPowerNode` escuta por comandos de ativação/desativação da carga em um tópico ROS 2 específico. Ao receber um comando, ele traduz essa instrução para um sinal PWM (Pulse Width Modulation) adequado e o envia ao PX4, que por sua vez controla o servo correspondente.

## 2. Estrutura do Código

### `CargaPowerNode` Classe

A classe `CargaPowerNode` herda de `rclpy.node.Node`, estabelecendo-o como um nó ROS 2.

#### 2.1. Inicialização (`__init__`)

No construtor, o nó é inicializado e os seguintes componentes são configurados:

* **Configuração de QoS (Quality of Service):** Um perfil de QoS é definido para as mensagens `VehicleCommands`.
    * `ReliabilityPolicy.RELIABLE`: Garante que os comandos importantes (como os de servo) sejam entregues, mesmo que haja retransmissões.
    * `DurabilityPolicy.TRANSIENT_LOCAL`: Permite que um publicador envie a "última" mensagem para novos assinantes, o que pode ser útil para garantir que o estado inicial do servo seja aplicado.
* **Publicador de Comandos PX4 (`self.vehicle_command_publisher`):**
    * Publica mensagens do tipo `px4_msgs.msg.VehicleCommands` no tópico `/fmu/in/vehicle_commands`. Este é o canal direto para enviar comandos de baixo nível (MAVLink) para o firmware PX4.
* **Assinante de Controle de Carga (`self.power_subscriber`):**
    * Assina mensagens do tipo `std_msgs.msg.Bool` (Boolean) no tópico `/drone/carga_power`. Este tópico é onde o `offboard_control.py` publica o comando `True` (para prender a carga) ou `False` (para soltar a carga).
* **Parâmetros do Servo:**
    * `self.SERVO_OUTPUT_INDEX = 8.0`: Define qual saída AUX da Pixhawk está conectada ao servo. Para a Pixhawk, a saída AUX1 geralmente corresponde ao índice de servo MAVLink 8. **Este valor precisa ser ajustado de acordo com a sua configuração de hardware.**
    * `self.PWM_VALUE_HOLD = 1500.0`: O valor PWM (em microssegundos) que mantém a carga presa. **Deve ser calibrado para o seu servo e mecanismo de liberação.**
    * `self.PWM_VALUE_RELEASE = 1000.0`: O valor PWM que libera a carga. **Também deve ser calibrado.**
* **Estado Inicial:** O nó inicia com `self.current_power_state = False` (carga desativada/solta por segurança) e imediatamente envia um comando para **prender a carga** (`self.PWM_VALUE_HOLD`) ao iniciar.

#### 2.2. Callback de Controle de Carga (`power_callback`)

Esta função é acionada sempre que uma nova mensagem é recebida no tópico `/drone/carga_power`.

* Ela verifica o valor `msg.data` (que é um booleano: `True` ou `False`).
* Se `msg.data` for `True` e o estado atual for `False` (ou seja, comando para prender e a carga está solta), ele envia o `PWM_VALUE_HOLD` para o servo e atualiza `self.current_power_state` para `True`.
* Se `msg.data` for `False` e o estado atual for `True` (ou seja, comando para soltar e a carga está presa), ele envia o `PWM_VALUE_RELEASE` para o servo e atualiza `self.current_power_state` para `False`.
* A lógica `if not self.current_power_state` e `if self.current_power_state` evita enviar comandos repetidos para o servo se o estado não mudou.

#### 2.3. Envio de Comando de Servo (`send_servo_command`)

Esta função encapsula a lógica para construir e publicar uma mensagem `VehicleCommands` para controlar o servo.

* Cria uma nova mensagem `VehicleCommands`.
* Define `msg.command` como `VehicleCommands.VEHICLE_CMD_DO_SET_SERVO`, que é o comando MAVLink para configurar um servo.
* `msg.param1`: É o índice do servo a ser controlado (`self.SERVO_OUTPUT_INDEX`).
* `msg.param2`: É o valor PWM desejado para o servo (`pwm_value`).
* `msg.timestamp`: Define o timestamp da mensagem, crucial para o PX4.
* Publica a mensagem no `self.vehicle_command_publisher`.

## 3. Fluxo de Operação

1.  O nó `CargaPowerNode` é iniciado.
2.  Ele envia um comando inicial para prender a carga (garantindo que o pacote não caia inadvertidamente).
3.  Ele fica aguardando mensagens no tópico `/drone/carga_power`.
4.  Quando o `offboard_control.py` (ou outro nó) decide prender a carga, ele publica `True` em `/drone/carga_power`. O `CargaPowerNode` recebe isso e comanda o servo para `PWM_VALUE_HOLD`.
5.  Quando o `offboard_control.py` decide liberar a carga (no estado `DROP_PACKAGE`), ele publica `False` em `/drone/carga_power`. O `CargaPowerNode` recebe isso e comanda o servo para `PWM_VALUE_RELEASE`, liberando o pacote.

## 4. Uso (main function)

A função `main` é o ponto de entrada padrão para o nó ROS 2:
* Inicializa o `rclpy`.
* Cria uma instância do `CargaPowerNode`.
* Mantém o nó em execução com `rclpy.spin()`, que processa os callbacks.
* Limpa os recursos do nó ao finalizar com `destroy_node()` e `shutdown()`.

---