import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from px4_msgs.msg import VehicleCommands

class CargaPowerNode(Node):
    def __init__(self):
        super().__init__('carga_power_node')

        # Configuração de QoS para comunicação com PX4
        # Note que para VehicleCommands, Transient Local é importante para garantir a entrega
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Melhor para comandos importantes
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher para enviar comandos para o PX4 (para controle do servo)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommands, '/fmu/in/vehicle_commands', qos_profile)

        # Subscriber para receber o comando de ligar/desligar a carga
        self.power_subscriber = self.create_subscription(
            Bool, 
            '/drone/carga_power', 
            self.power_callback, 
            10 # Depth
        )

        # Parâmetros do Servo de Carga (AJUSTE CONFORME SEU HARDWARE!)
        self.SERVO_OUTPUT_INDEX = 8.0  # MAVLink index para AUX OUTPUT 1 (se estiver usando 1-6 é 1-6, AUX1 é 8, AUX2 é 9, etc.)
                                      # Para PX4, AUX1 é o servo 8 na mensagem VEHICLE_CMD_DO_SET_SERVO.
        self.PWM_VALUE_HOLD = 1500.0  # Valor PWM para MANTER a carga (ex: 1500 us)
        self.PWM_VALUE_RELEASE = 1000.0 # Valor PWM para SOLTAR a carga (ex: 1000 us)

        self.get_logger().info("CargaPowerNode started. Waiting for power commands on /drone/carga_power")
        self.current_power_state = False # Estado inicial: carga desativada por segurança

        # Envia o estado inicial do servo (prender carga)
        self.send_servo_command(self.SERVO_OUTPUT_INDEX, self.PWM_VALUE_HOLD)


    def power_callback(self, msg):
        """
        Callback para o tópico /drone/carga_power.
        Controla o servo com base no valor recebido.
        """
        if msg.data is True:
            if not self.current_power_state:
                self.get_logger().info("Carga: ATIVADA. Enviando comando para SEGURAR o pacote.")
                self.send_servo_command(self.SERVO_OUTPUT_INDEX, self.PWM_VALUE_HOLD)
                self.current_power_state = True
        else: # msg.data is False
            if self.current_power_state: # Só age se o estado mudou para desligado
                self.get_logger().info("Carga: DESATIVADA. Enviando comando para SOLTAR o pacote!")
                self.send_servo_command(self.SERVO_OUTPUT_INDEX, self.PWM_VALUE_RELEASE)
                self.current_power_state = False


    def send_servo_command(self, servo_num, pwm_value):
        """
        Publica um comando VEHICLE_CMD_DO_SET_SERVO para o PX4.
        """
        msg = VehicleCommands()
        msg.command = VehicleCommands.VEHICLE_CMD_DO_SET_SERVO
        msg.param1 = float(servo_num) # Índice do servo (ex: 8 para AUX1)
        msg.param2 = float(pwm_value) # Valor PWM em microssegundos
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    carga_power_node = CargaPowerNode()
    rclpy.spin(carga_power_node)
    carga_power_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()