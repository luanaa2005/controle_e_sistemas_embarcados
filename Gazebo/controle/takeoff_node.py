import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommands
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

from geometry_msgs.msg import Point # Para receber informações da linha e QR/bases

import numpy as np
import time

# --- 1. Definição dos Estados da Missão ---
class MissionState:
    INITIAL = 0
    TAKEOFF = 1
    HOVER_AT_ALTITUDE = 2 # Novo estado para pairar após decolar
    FOLLOW_LINE = 3
    SEARCH_QR_CODE = 4 # Pode ser parte de FOLLOW_LINE ou um estado separado
    NAVIGATE_TO_DELIVERY_ZONE = 5
    SEARCH_DELIVERY_BASE = 6
    DROP_PACKAGE = 7
    LANDING = 8         # NOVO ESTADO: Drone pousando no ponto inicial
    MISSION_COMPLETE = 9 # Estado final da missão

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_takeoff')

        # Configuração de QoS para comunicação com PX4 (geralmente BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers: Para enviar dados e comandos ao PX4
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommands, '/fmu/in/vehicle_commands', qos_profile)

        # Subscribers: Para receber dados do PX4 e dos nós de visão
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Subscribers para os nós de visão (tópicos de exemplo, ajuste conforme seus nós)
        self.line_position_subscriber = self.create_subscription(
            Point, '/drone/line_center', self.line_position_callback, 10)
        self.qr_code_subscriber = self.create_subscription(
            Point, '/drone/qr_code_data', self.qr_code_callback, 10) 
        self.base_detection_subscriber = self.create_subscription(
            Point, '/drone/base_data', self.base_detection_callback, 10)

        # Atributos de controle e estado da missão
        self.offboard_setpoint_counter = 0 # Contador para manter o modo Offboard ativo
        self.vehicle_local_position = VehicleLocalPosition() # Posição local atual do drone
        self.vehicle_status = VehicleStatus() # Status de armar e modo de navegação do drone
        self.target_height = -1.5  # Altura de voo da missão (1.5 metros, Z negativo no NED)
        self.current_state = MissionState.INITIAL # Estado inicial da máquina de estados

        # Atributos para dados de controle de visão (inicializados como None)
        self.line_center_x = None # Posição X da linha na imagem
        self.qr_code_info = None # Tupla: (ID do QR, pos_x_imagem, pos_y_imagem)
        self.detected_base_info = None # Tupla: (tipo_base_enum, pos_x_imagem, pos_y_imagem)
        
        # Coordenadas do QR Code no mundo (capturadas no momento da detecção)
        self.qr_world_x = 0.0
        self.qr_world_y = 0.0
        self.qr_world_z = 0.0

        # Coordenadas da Zona de Entrega (calculadas a partir do QR Code)
        self.delivery_target_x = 0.0
        self.delivery_target_y = 0.0
        self.delivery_target_z = 0.0 # Manter a altura de voo para a zona de entrega

        # Ponto de pouso final (geralmente o ponto inicial do drone: 0,0,0)
        self.landing_target_x = 0.0
        self.landing_target_y = 0.0
        self.landing_target_z = 0.0 

        # Mapeamento de QR Code para Zona de Entrega (offsets relativos à posição do QR)
        # Formato: {QR_ID: (offset_norte_em_Y, offset_leste_em_X)}
        # Importante: No sistema NED, Norte é -Y, Leste é +X.
        self.delivery_zones = {
            1: (-4.0, -3.0), # Zona 1: 4m Norte (-Y), 3m Oeste (-X) do QR Code
            2: (-5.0, 0.0),  # Zona 2: 5m Norte (-Y) do QR Code (direto à frente)
            3: (-4.0, 3.0)   # Zona 3: 4m Norte (-Y), 3m Leste (+X) do QR Code
        }
        
        # Mapeamento de QR Code para o Tipo de Base esperado na zona de entrega
        # Os valores devem corresponder aos tipos que seu `base_detector_node` publica
        self.expected_base_types = {
            1: "SQUARE_BROWN",  # Exemplo: QR 1 corresponde a um quadrado marrom
            2: "TRIANGLE_BLUE", # Exemplo: QR 2 corresponde a um triângulo azul
            3: "HEXAGON_RED"    # Exemplo: QR 3 corresponde a um hexágono vermelho
        }

        # Timer principal para o loop de controle
        self.timer = self.create_timer(0.1, self.timer_callback) # 100ms, 10Hz

        self.get_logger().info("OffboardControl Node started. Initializing mission...")

    # --- Funções de Callback (Subscribers) ---
    def vehicle_local_position_callback(self, msg):
        """Callback para a posição local do drone."""
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        """Callback para o status do veículo (armado, modo de voo)."""
        self.vehicle_status = msg

    def line_position_callback(self, msg):
        """Callback para a posição da linha detectada pelo line_detector_node."""
        self.line_center_x = msg.x

    def qr_code_callback(self, msg):
        """Callback para os dados do QR Code detectado pelo qr_code_detector_node."""
        # Assumindo msg.x é o ID do QR (int), msg.y e msg.z são a posição do QR na imagem (pixels)
        if self.qr_code_info is None: # Processa apenas a primeira detecção para evitar re-gatilho
            self.qr_code_info = (int(msg.x), msg.y, msg.z)
            self.get_logger().info(f"QR Code {self.qr_code_info[0]} detectado!")
            # Captura as coordenadas do drone no momento da detecção do QR para cálculo futuro
            # Estas são as coordenadas do drone no sistema de referência global do Gazebo (NED)
            self.qr_world_x = self.vehicle_local_position.x
            self.qr_world_y = self.vehicle_local_position.y
            self.qr_world_z = self.vehicle_local_position.z # A altura em que o QR foi visto

    def base_detection_callback(self, msg):
        """Callback para os dados da base de entrega detectada pelo base_detector_node."""
        # Assumindo msg.x é o tipo de base (int/enum), msg.y e msg.z são a posição da base na imagem (pixels)
        if self.detected_base_info is None: # Processa apenas a primeira detecção de base por estado
            self.detected_base_info = (int(msg.x), msg.y, msg.z)
            self.get_logger().info(f"Base detectada: Tipo {self.detected_base_info[0]}")


    # --- Funções de Publicação (Envio de Comandos para o PX4) ---
    def publish_offboard_control_mode(self):
        """Publica o modo de controle Offboard para o PX4."""
        msg = OffboardControlMode()
        msg.position = True # Estamos controlando por posição
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        """Publica um setpoint de trajetória (posição X, Y, Z e guinada)."""
        msg = TrajectorySetpoint()
        msg.position[0] = float(x)
        msg.position[1] = float(y)
        msg.position[2] = float(z)
        msg.yaw = float(yaw) # Guinada (orientação em torno do eixo Z)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publica um comando geral do veículo (MAVLink)."""
        msg = VehicleCommands()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def calculate_delivery_target_coordinates(self, qr_id):
        """
        Calcula as coordenadas absolutas (X, Y, Z) da zona de entrega
        com base no ID do QR Code e na posição do drone quando o QR foi visto.
        """
        if qr_id in self.delivery_zones:
            offset_north, offset_east = self.delivery_zones[qr_id]
            
            # No sistema NED (North-East-Down):
            # Norte é ao longo do eixo -Y
            # Leste é ao longo do eixo +X
            target_x = self.qr_world_x + offset_east
            target_y = self.qr_world_y - offset_north 
            target_z = self.target_height # Manter a altura de voo padrão

            return target_x, target_y, target_z
        else:
            self.get_logger().error(f"QR Code ID {qr_id} não mapeado para zona de entrega. Verifique `self.delivery_zones`.")
            return None, None, None

    # --- Lógica Principal da Máquina de Estados (Chamada pelo Timer) ---
    def timer_callback(self):
        """
        Função principal do loop de controle.
        Gerencia as transições de estado e as ações do drone.
        """
        self.publish_offboard_control_mode() # Sempre publicar para manter o modo Offboard ativo

        # Captura a posição atual do drone para uso na lógica de estados
        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y
        current_z = self.vehicle_local_position.z
        
        # --- Lógica para cada estado da missão ---

        # Estado: INITIAL (Inicialização e Preparação para Decolagem)
        if self.current_state == MissionState.INITIAL:
            self.get_logger().info("Estado: INITIAL - Aguardando ativação do modo Offboard.")
            # Publica setpoints por um tempo para garantir que o Offboard pode ser ativado
            if self.offboard_setpoint_counter < 100: # Publica por aprox. 10 segundos (100 * 0.1s)
                self.publish_trajectory_setpoint(0.0, 0.0, self.target_height) # Setpoint inicial para Z
                self.offboard_setpoint_counter += 1
            else:
                # Tenta armar o drone se ainda não estiver armado
                if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                    self.get_logger().info("Armando drone...")
                    self.publish_vehicle_command(VehicleCommands.VEHICLE_CMD_ARM_DISARM, 1.0) # param1 = 1.0 (armar)
                    time.sleep(1) # Pequena pausa para o comando ser processado
                
                # Tenta mudar para o modo Offboard
                if not (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                    self.get_logger().info("Solicitando modo Offboard...")
                    self.publish_vehicle_command(VehicleCommands.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # param2 = 6.0 (Offboard)
                else:
                    self.get_logger().info("Drone em modo Offboard. Transicionando para TAKEOFF.")
                    self.current_state = MissionState.TAKEOFF
            
        # Estado: TAKEOFF (Decolagem Vertical)
        elif self.current_state == MissionState.TAKEOFF:
            self.get_logger().info(f"Estado: TAKEOFF - Subindo para {abs(self.target_height):.2f}m. Altura atual: {abs(current_z):.2f}m")
            # Mantém a posição XY atual e sobe para a altura alvo
            self.publish_trajectory_setpoint(current_x, current_y, self.target_height)
            
            # Transição: Altura alvo alcançada
            if abs(current_z - self.target_height) < 0.1: # Margem de erro de 10cm
                self.get_logger().info("Altura alvo alcançada. Transicionando para HOVER_AT_ALTITUDE.")
                self.current_state = MissionState.HOVER_AT_ALTITUDE
                self.start_hover_time = self.get_clock().now().nanoseconds / 1_000_000_000 # Marca o tempo para a pausa

        # Estado: HOVER_AT_ALTITUDE (Pairar na Altitude Alvo)
        elif self.current_state == MissionState.HOVER_AT_ALTITUDE:
            self.get_logger().info(f"Estado: HOVER_AT_ALTITUDE - Pairando. Altura: {abs(current_z):.2f}m")
            self.publish_trajectory_setpoint(current_x, current_y, self.target_height) # Mantém a posição e altura

            # Transição: Após uma breve pausa, ou se outras condições forem satisfeitas
            if (self.get_clock().now().nanoseconds / 1_000_000_000 - self.start_hover_time) > 2.0: # Pairar por 2 segundos
                self.get_logger().info("Pausa para hover concluída. Transicionando para FOLLOW_LINE.")
                self.current_state = MissionState.FOLLOW_LINE
                self.get_logger().info(f"Drone na posição inicial de linha: X={current_x:.2f}, Y={current_y:.2f}")
                self.line_center_x = None # Resetar para esperar novos dados da linha

        # Estado: FOLLOW_LINE (Seguimento da Linha Azul)
        elif self.current_state == MissionState.FOLLOW_LINE:
            self.get_logger().info(f"Estado: FOLLOW_LINE - Seguindo linha. Linha X: {self.line_center_x}")
            
            # Transição: QR Code detectado
            if self.qr_code_info is not None:
                self.get_logger().info(f"QR Code {self.qr_code_info[0]} detectado! Transicionando para NAVIGATE_TO_DELIVERY_ZONE.")
                self.current_state = MissionState.NAVIGATE_TO_DELIVERY_ZONE
                self.line_center_x = None # Limpar informação da linha, não mais necessária
                
                # Calcula o destino exato da zona de entrega com base no QR Code
                self.delivery_target_x, self.delivery_target_y, self.delivery_target_z = \
                    self.calculate_delivery_target_coordinates(self.qr_code_info[0])
                
                if self.delivery_target_x is None: # Se houver erro no cálculo, ir para missão completa ou estado de erro
                    self.get_logger().error("Erro no cálculo da zona de entrega. Indo para MISSÃO COMPLETA.")
                    self.current_state = MissionState.MISSION_COMPLETE
                return # Sai do callback para permitir que o novo estado seja processado na próxima iteração

            # Lógica de controle para seguir a linha
            if self.line_center_x is not None:
                image_center_pixel = 640 / 2 # Centro da imagem (assumindo largura de 640 pixels)
                error_x = self.line_center_x - image_center_pixel # Erro: distância da linha ao centro da imagem
                
                Kp_horizontal = 0.005 # Ganho Proporcional para controle horizontal (ajuste necessário!)
                
                # Calcula a mudança desejada na posição Y do drone (NED)
                # Se o erro é positivo (linha à direita do centro), o drone move para +Y (direita no NED)
                # para centralizar a linha. Se o erro é negativo, move para -Y.
                desired_y_change = error_x * Kp_horizontal 
                
                # Atualiza o setpoint Y para tentar centralizar a linha
                # Pode precisar de um controlador PID mais robusto e suavização de movimento
                new_setpoint_y = current_y + desired_y_change
                
                # Publica o setpoint de trajetória (mantém X e Z, ajusta Y)
                # Para avançar ao longo da linha, você adicionaria um pequeno avanço no eixo X ou Y
                # Ex: current_x + 0.1 (para avançar no eixo X)
                self.publish_trajectory_setpoint(current_x, new_setpoint_y, self.target_height) 
            else:
                # Se a linha não é detectada, o drone paira ou executa uma estratégia de busca
                self.get_logger().warn("Linha não detectada no estado FOLLOW_LINE. Pairando...")
                self.publish_trajectory_setpoint(current_x, current_y, self.target_height)


        # Estado: NAVIGATE_TO_DELIVERY_ZONE (Navegar para a Zona de Entrega)
        elif self.current_state == MissionState.NAVIGATE_TO_DELIVERY_ZONE:
            self.get_logger().info(f"Estado: NAVIGATE_TO_DELIVERY_ZONE - Indo para X:{self.delivery_target_x:.2f}, Y:{self.delivery_target_y:.2f}")
            # Publica o setpoint para a zona de entrega
            self.publish_trajectory_setpoint(self.delivery_target_x, self.delivery_target_y, self.delivery_target_z)
            
            # Transição: Posição alvo da zona de entrega alcançada
            distance_to_target = np.sqrt(
                (current_x - self.delivery_target_x)**2 +
                (current_y - self.delivery_target_y)**2
            )
            if distance_to_target < 0.5: # Dentro de 0.5m do alvo
                self.get_logger().info("Zona de entrega alcançada. Transicionando para SEARCH_DELIVERY_BASE.")
                self.current_state = MissionState.SEARCH_DELIVERY_BASE
                self.detected_base_info = None # Limpa informação de base para nova detecção
            
        # Estado: SEARCH_DELIVERY_BASE (Procurar e Identificar a Base de Entrega)
        elif self.current_state == MissionState.SEARCH_DELIVERY_BASE:
            self.get_logger().info(f"Estado: SEARCH_DELIVERY_BASE - Procurando base. Base detectada: {self.detected_base_info}")
            
            # Por enquanto, o drone apenas paira na zona de entrega e espera a detecção
            # Uma estratégia de busca (espiral, varredura) pode ser implementada aqui se necessário
            self.publish_trajectory_setpoint(current_x, current_y, self.target_height)
            
            # Transição: Base correta detectada e drone está sobre ela
            if self.detected_base_info is not None:
                qr_id = self.qr_code_info[0] if self.qr_code_info else None
                expected_base = self.expected_base_types.get(qr_id) # Pega o tipo de base esperado para o QR lido
                
                # Compara o tipo de base detectado com o tipo esperado
                # (Assumindo que self.detected_base_info[0] é um string ou um enum que você mapeia para string)
                if expected_base and str(self.detected_base_info[0]) == expected_base: 
                    self.get_logger().info(f"Base correta ({expected_base}) detectada! Transicionando para DROP_PACKAGE.")
                    self.current_state = MissionState.DROP_PACKAGE
                    # Opcional: Adicione lógica aqui para centralizar o drone sobre a base detectada
                    # Ex: self.target_x_base_center, self.target_y_base_center = self.transform_image_to_world_coords(self.detected_base_info[1], self.detected_base_info[2])
                    # self.publish_trajectory_setpoint(self.target_x_base_center, self.target_y_base_center, self.target_height)
                else:
                    self.get_logger().warn(f"Base detectada ({self.detected_base_info[0]}) não é a esperada ({expected_base}). Continuar procurando...")
                    # Pode implementar uma estratégia de busca ou apenas esperar por uma nova detecção

        # Estado: DROP_PACKAGE (Soltar o Pacote)
        elif self.current_state == MissionState.DROP_PACKAGE:
            self.get_logger().info("Estado: DROP_PACKAGE - Soltando pacote...")
            # Enviar comando MAVLink para soltar o pacote (ajuste o comando e parâmetros para sua simulação/hardware)
            # Exemplo: controle de servo (servo_num=8, pwm_value=1000us)
            self.publish_vehicle_command(VehicleCommands.VEHICLE_CMD_DO_SET_SERVO, 8.0, 1000.0) 
            self.get_logger().info("Comando de soltura enviado.")
            time.sleep(2) # Pausa para simular o tempo de soltura
            
            # Transição: Após soltar o pacote, iniciar o processo de pouso
            self.get_logger().info("Pacote solto. Transicionando para LANDING.")
            self.current_state = MissionState.LANDING
            # O ponto de pouso já está definido no __init__ como 0,0,0
            # Mas você pode redefinir aqui se precisar de um ponto de pouso dinâmico
            self.landing_target_x = 0.0 # Ex: Coordenada X do ponto inicial
            self.landing_target_y = 0.0 # Ex: Coordenada Y do ponto inicial
            self.landing_target_z = 0.0 # Z=0.0 para pousar no chão no NED

        # Estado: LANDING (Retorno para Ponto Inicial e Pouso)
        elif self.current_state == MissionState.LANDING:
            self.get_logger().info(f"Estado: LANDING - Movendo para o ponto de pouso (X:{self.landing_target_x:.2f}, Y:{self.landing_target_y:.2f}) e descendo para Z:{self.landing_target_z:.2f}.")
            
            # Publica o setpoint para o ponto de pouso final (X, Y e Z no chão)
            self.publish_trajectory_setpoint(self.landing_target_x, self.landing_target_y, self.landing_target_z)
            
            # Condições para considerar o pouso completo
            distance_to_landing_spot = np.sqrt(
                (current_x - self.landing_target_x)**2 +
                (current_y - self.landing_target_y)**2
            )
            
            # Transição: Pouso detectado (próximo ao ponto alvo e no chão ou drone desarmado)
            if (distance_to_landing_spot < 0.5 and abs(current_z - self.landing_target_z) < 0.1) or \
               self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                # Se o drone está no chão (Z próximo de 0.0) e próximo do ponto alvo,
                # ou se o PX4 desarmou automaticamente (indicando pouso), a missão está completa.
                self.get_logger().info("Drone pousou com sucesso. Transicionando para MISSION_COMPLETE.")
                self.current_state = MissionState.MISSION_COMPLETE

        # Estado: MISSION_COMPLETE (Missão Finalizada)
        elif self.current_state == MissionState.MISSION_COMPLETE:
            self.get_logger().info("Estado: MISSION_COMPLETE - Missão finalizada. Desarmando e encerrando.")
            # Garante que o drone está desarmado.
            self.publish_vehicle_command(VehicleCommands.VEHICLE_CMD_ARM_DISARM, 0.0) # Param1 = 0.0 (desarmar)
            # Neste ponto, o nó continuará rodando, mas não enviará mais comandos de voo significativos.
            # O usuário deve encerrá-lo manualmente (Ctrl+C).

# --- Função Principal para Iniciar o Nó ---
def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control) # Mantém o nó rodando e processando callbacks
    offboard_control.destroy_node() # Libera os recursos do nó ao finalizar
    rclpy.shutdown() # Encerra o ROS 2

if __name__ == '__main__':
    main()

