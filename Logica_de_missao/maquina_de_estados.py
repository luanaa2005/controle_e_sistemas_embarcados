
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum # Melhor usar enum.Enum para clareza e robustez

# IMPORTAÇÕES DE MENSAGENS (NECESSÁRIAS PARA O CÓDIGO FUNCIONAR, MAS NÃO SÃO PARTE DA LÓGICA DA ME)
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommands
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import Bool

from geometry_msgs.msg import Point

import numpy as np
import time # Para delays simples, embora seja melhor evitar em callbacks de alta frequência

# --- 1. Definição dos Estados da Missão ---
class MissionState(Enum): # Usando Enum do módulo enum para tipagem mais forte
    INITIAL = 0
    TAKEOFF = 1
    HOVER_AT_ALTITUDE = 2 # Novo estado para pairar após decolar
    FOLLOW_LINE = 3
    SEARCH_QR_CODE = 4 # Este estado está definido, mas sua transição é mais implícita em FOLLOW_LINE
    NAVIGATE_TO_DELIVERY_ZONE = 5
    SEARCH_DELIVERY_BASE = 6
    DROP_PACKAGE = 7
    LANDING = 8         # NOVO ESTADO: Drone pousando no ponto inicial
    MISSION_COMPLETE = 9 # Estado final da missão
    EMERGENCY_LAND = 10  # Adicionado estado de pouso de emergência

# --- Definição dos Tipos de Base (DEVE CORRESPONDER AO SEU base_detector_node.py) ---
# Se o seu base_detector_node envia ints, use int. Se envia strings, use strings.
# Aqui, usando uma Enum para a lógica interna, mas comparando com o int que viria do ROS2 msg.x
class BaseType(Enum):
    SQUARE_BROWN = 1
    TRIANGLE_BLUE = 2
    HEXAGON_RED = 3

# OBSERVAÇÃO: A classe OffboardControl e seus métodos __init__, callbacks e publishers
# ESTARIAM AQUI ACIMA DESTA PARTE, como no código completo que você forneceu.
# Estou mostrando apenas a parte que você pediu explicitamente como "máquina de estados".

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')
        # ... (atributos e publishers/subscribers seriam inicializados aqui) ...
        # Apenas para que este trecho seja executável, vou adicionar os atributos mínimos.
        # No seu código real, estes já estariam definidos pelo __init__ completo.
        self.offboard_setpoint_counter = 0
        self.vehicle_status = type('VehicleStatus', (object,), {'arming_state': 0, 'NAVIGATION_STATE_OFFBOARD': 6, 'ARMING_STATE_ARMED': 2, 'ARMING_STATE_DISARMED': 1})() # Mock para teste
        self.vehicle_local_position = type('VehicleLocalPosition', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})() # Mock
        self.target_height = -1.5
        self.current_state = MissionState.INITIAL
        self.package_delivered = False
        self.home_position = None
        self.start_hover_time = 0.0
        self.line_center_x = None
        self.qr_code_info = None
        self.detected_base_info = None
        self.qr_world_x = 0.0
        self.qr_world_y = 0.0
        self.qr_world_z = 0.0
        self.delivery_target_x = 0.0
        self.delivery_target_y = 0.0
        self.delivery_target_z = 0.0
        self.landing_target_x = 0.0
        self.landing_target_y = 0.0
        self.landing_target_z = 0.0
        self.delivery_zones = {
            1: (-4.0, -3.0),
            2: (-5.0, 0.0),
            3: (-4.0, 3.0)
        }
        self.expected_base_types = {
            1: BaseType.SQUARE_BROWN.value,
            2: BaseType.TRIANGLE_BLUE.value,
            3: BaseType.HEXAGON_RED.value
        }
        # Mocks para publishers/loggers para permitir que o trecho seja isolado
        self.power_pub = type('MockPublisher', (object,), {'publish': lambda x: None})()
        self.get_logger = type('MockLogger', (object,), {'info': lambda x: None, 'warn': lambda x: None, 'error': lambda x: None})
        self.get_logger = self.get_logger()
        self.get_clock = type('MockClock', (object,), {'now': lambda: type('MockTime', (object,), {'nanoseconds': time.time() * 1_000_000_000})()})

        # Mocks para funções de publicação de comandos PX4 (elas seriam definidas acima)
        self.publish_offboard_control_mode = lambda: self.get_logger.info("Mock: publicando offboard control mode")
        self.publish_trajectory_setpoint = lambda x, y, z, yaw=0.0: self.get_logger.info(f"Mock: Setpoint to X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}")
        self.publish_vehicle_command = lambda cmd, p1=0.0, p2=0.0: self.get_logger.info(f"Mock: Vehicle Command {cmd} P1:{p1} P2:{p2}")
        self.calculate_delivery_target_coordinates = self._calculate_delivery_target_coordinates # Usar a própria função para mock

    def _calculate_delivery_target_coordinates(self, qr_id):
        # Esta é a função auxiliar que estaria fora do timer_callback
        if qr_id in self.delivery_zones:
            offset_north, offset_east = self.delivery_zones[qr_id]
            target_x = self.qr_world_x + offset_east
            target_y = self.qr_world_y - offset_north 
            target_z = self.target_height
            return target_x, target_y, target_z
        else:
            self.get_logger.error(f"QR Code ID {qr_id} não mapeado.")
            return None, None, None


    # --- Lógica Principal da Máquina de Estados (Chamada pelo Timer) ---
    def timer_callback(self):
        """
        Função principal do loop de controle.
        Gerencia as transições de estado e as ações do drone.
        """
        # Sempre publicar offboard_control_mode para manter o modo ativo no PX4
        self.publish_offboard_control_mode()

        # Controla a energia da carga (ligada por padrão, desligada quando `package_delivered` é True)
        self.power_pub.publish(Bool(data=not self.package_delivered))

        # Captura a posição atual do drone para uso na lógica de estados
        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y
        current_z = self.vehicle_local_position.z # Lembre-se: Z é negativo para altitudes acima do solo no NED
        
        # --- Lógica para cada estado da missão ---

        # Estado: INITIAL (Inicialização e Preparação para Decolagem)
        if self.current_state == MissionState.INITIAL:
            self.get_logger.info("Estado: INITIAL - Aguardando ativação do modo Offboard.")
            
            # Publica setpoints por um tempo para garantir que o Offboard possa ser ativado
            if self.offboard_setpoint_counter < 100: # Publica por aprox. 10 segundos (100 * 0.1s)
                # Envia um setpoint para a posição atual, mantendo a altura alvo, para "engajar" o offboard
                self.publish_trajectory_setpoint(current_x, current_y, self.target_height)
                self.offboard_setpoint_counter += 1
            else:
                # Tenta armar o drone se ainda não estiver armado
                if self.vehicle_status.arming_state == self.vehicle_status.ARMING_STATE_DISARMED:
                    self.get_logger.info("Armando drone...")
                    self.publish_vehicle_command(VehicleCommands.VEHICLE_CMD_ARM_DISARM, 1.0) # param1 = 1.0 (armar)
                    # Não use time.sleep em callbacks de timer. Em vez disso, retorne e espere o próximo ciclo.
                
                # Tenta mudar para o modo Offboard
                if not (self.vehicle_status.nav_state == self.vehicle_status.NAVIGATION_STATE_OFFBOARD):
                    self.get_logger.info("Solicitando modo Offboard...")
                    self.publish_vehicle_command(VehicleCommands.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # param2 = 6.0 (Offboard)
                else:
                    self.get_logger.info("Drone em modo Offboard. Transicionando para TAKEOFF.")
                    self.current_state = MissionState.TAKEOFF
                    
        # Estado: TAKEOFF (Decolagem Vertical)
        elif self.current_state == MissionState.TAKEOFF:
            self.get_logger.info(f"Estado: TAKEOFF - Subindo para {abs(self.target_height):.2f}m. Altura atual: {abs(current_z):.2f}m")
            # Mantém a posição XY atual e sobe para a altura alvo
            self.publish_trajectory_setpoint(current_x, current_y, self.target_height)
            
            # Transição: Altura alvo alcançada
            if abs(current_z - self.target_height) < 0.1: # Margem de erro de 10cm
                self.home_position = (current_x, current_y) # Registra a posição de decolagem
                self.get_logger.info(f"Altitude alvo alcançada. Base registrada em {self.home_position}. Transicionando para HOVER_AT_ALTITUDE.")
                self.current_state = MissionState.HOVER_AT_ALTITUDE
                self.start_hover_time = self.get_clock().now().nanoseconds / 1_000_000_000 # Marca o tempo para a pausa

        # Estado: HOVER_AT_ALTITUDE (Pairar na Altitude Alvo)
        elif self.current_state == MissionState.HOVER_AT_ALTITUDE:
            self.get_logger.info(f"Estado: HOVER_AT_ALTITUDE - Pairando. Altura: {abs(current_z):.2f}m")
            self.publish_trajectory_setpoint(current_x, current_y, self.target_height) # Mantém a posição e altura

            # Transição: Após uma breve pausa, ou se outras condições forem satisfeitas
            if (self.get_clock().now().nanoseconds / 1_000_000_000 - self.start_hover_time) > 2.0: # Pairar por 2 segundos
                self.get_logger.info("Pausa para hover concluída. Transicionando para FOLLOW_LINE.")
                self.current_state = MissionState.FOLLOW_LINE
                self.get_logger.info(f"Drone na posição inicial de linha: X={current_x:.2f}, Y={current_y:.2f}")
                self.line_center_x = None # Resetar para esperar novos dados da linha

        # Estado: FOLLOW_LINE (Seguimento da Linha Azul)
        elif self.current_state == MissionState.FOLLOW_LINE:
            self.get_logger.info(f"Estado: FOLLOW_LINE - Seguindo linha. Linha X: {self.line_center_x}")
            
            # Transição: QR Code detectado
            if self.qr_code_info is not None:
                self.get_logger.info(f"QR Code {self.qr_code_info[0]} detectado! Transicionando para NAVIGATE_TO_DELIVERY_ZONE.")
                self.current_state = MissionState.NAVIGATE_TO_DELIVERY_ZONE
                self.line_center_x = None # Limpar informação da linha, não mais necessária
                
                # Calcula o destino exato da zona de entrega com base no QR Code
                self.delivery_target_x, self.delivery_target_y, self.delivery_target_z = \
                    self.calculate_delivery_target_coordinates(self.qr_code_info[0])
                
                if self.delivery_target_x is None: # Se houver erro no cálculo, ir para missão completa ou estado de erro
                    self.get_logger.error("Erro no cálculo da zona de entrega. Indo para MISSÃO COMPLETA.")
                    self.current_state = MissionState.MISSION_COMPLETE
                return # Sai do callback para permitir que o novo estado seja processado na próxima iteração

            # Lógica de controle para seguir a linha
            if self.line_center_x is not None:
                image_width = 640 # Centro da imagem (assumindo largura de 640 pixels)
                image_center_pixel = image_width / 2 
                error_x = self.line_center_x - image_center_pixel # Erro: distância da linha ao centro da imagem
                
                Kp_horizontal = 0.005 # Ganho Proporcional para controle horizontal (ajuste necessário!)
                
                # Calcula a mudança desejada na posição Y do drone (NED)
                desired_y_change = error_x * Kp_horizontal 
                
                # Limita a correção para evitar movimentos bruscos
                max_correction = 0.5 # metros por ciclo de 100ms
                desired_y_change = max(min(desired_y_change, max_correction), -max_correction)

                # Atualiza o setpoint Y para tentar centralizar a linha
                # Para avançar ao longo da linha, você adicionaria um pequeno avanço no eixo X ou Y
                # Ex: current_x + 0.1 (para avançar no eixo X)
                # Para uma linha reta no eixo X (com X aumentando):
                forward_speed_x = 0.5 # m/s
                new_setpoint_x = current_x + (forward_speed_x * 0.1) # 0.1 é o intervalo do timer
                new_setpoint_y = current_y + desired_y_change
                
                self.publish_trajectory_setpoint(new_setpoint_x, new_setpoint_y, self.target_height) 
            else:
                # Se a linha não é detectada, o drone paira ou executa uma estratégia de busca
                self.get_logger.warn("Linha não detectada no estado FOLLOW_LINE. Pairando...")
                self.publish_trajectory_setpoint(current_x, current_y, self.target_height)


        # Estado: NAVIGATE_TO_DELIVERY_ZONE (Navegar para a Zona de Entrega)
        elif self.current_state == MissionState.NAVIGATE_TO_DELIVERY_ZONE:
            self.get_logger.info(f"Estado: NAVIGATE_TO_DELIVERY_ZONE - Indo para X:{self.delivery_target_x:.2f}, Y:{self.delivery_target_y:.2f}")
            # Publica o setpoint para a zona de entrega
            self.publish_trajectory_setpoint(self.delivery_target_x, self.delivery_target_y, self.delivery_target_z)
            
            # Transição: Posição alvo da zona de entrega alcançada
            distance_to_target = np.sqrt(
                (current_x - self.delivery_target_x)**2 +
                (current_y - self.delivery_target_y)**2
            )
            if distance_to_target < 0.5: # Dentro de 0.5m do alvo
                self.get_logger.info("Zona de entrega alcançada. Transicionando para SEARCH_DELIVERY_BASE.")
                self.current_state = MissionState.SEARCH_DELIVERY_BASE
                self.detected_base_info = None # Limpa informação de base para nova detecção
            
        # Estado: SEARCH_DELIVERY_BASE (Procurar e Identificar a Base de Entrega)
        elif self.current_state == MissionState.SEARCH_DELIVERY_BASE:
            self.get_logger.info(f"Estado: SEARCH_DELIVERY_BASE - Procurando base. Base detectada: {self.detected_base_info}")
            
            # Por enquanto, o drone apenas paira na zona de entrega e espera a detecção
            # Uma estratégia de busca (espiral, varredura) pode ser implementada aqui se necessário
            self.publish_trajectory_setpoint(current_x, current_y, self.target_height)
            
            # Transição: Base correta detectada e drone está sobre ela
            if self.detected_base_info is not None:
                qr_id = self.qr_code_info[0] if self.qr_code_info else None
                expected_base_value = self.expected_base_types.get(qr_id) # Pega o tipo de base esperado (valor int)
                
                # Compara o tipo de base detectado (int de msg.x) com o tipo esperado (int da enum)
                if expected_base_value is not None and self.detected_base_info[0] == expected_base_value: 
                    self.get_logger.info(f"Base correta ({BaseType(expected_base_value).name}) detectada! Transicionando para DROP_PACKAGE.")
                    self.current_state = MissionState.DROP_PACKAGE
                    # Opcional: Adicione lógica aqui para centralizar o drone sobre a base detectada
                    # Isso envolveria usar self.detected_base_info[1] e [2] (pixels) para calcular offsets
                    # e enviar setpoints para centralizar. Poderia ser um sub-estado ou um PID rápido.
                else:
                    self.get_logger.warn(f"Base detectada ({self.detected_base_info[0]}) não é a esperada ({expected_base_value}). Continuar procurando...")
                    # Pode implementar uma estratégia de busca ou apenas esperar por uma nova detecção

        # Estado: DROP_PACKAGE (Soltar o Pacote)
        elif self.current_state == MissionState.DROP_PACKAGE:
            self.get_logger.info("Estado: DROP_PACKAGE - Soltando pacote: cortando energia da carga.")
            # Define a flag para True, o que fará com que power_pub publique False (cortando a energia)
            self.package_delivered = True 
            self.publish_trajectory_setpoint(current_x, current_y, self.target_height) # Mantém hover
            # time.sleep(2) # NÃO USE time.sleep() em um callback de timer ROS 2! Ele bloqueia o nó.
            # Para atrasos, use um contador de ticks do timer ou um timer ROS 2 separado.
            
            # Simples transição imediata após o comando (para demonstração)
            # Em um cenário real, você teria um timer para garantir a soltura completa.
            self.get_logger.info("Pacote solto (simulado). Transicionando para LANDING.")
            self.current_state = MissionState.LANDING
            # Define o ponto de pouso para a posição de decolagem
            if self.home_position:
                self.landing_target_x = self.home_position[0] 
                self.landing_target_y = self.home_position[1]
            else:
                self.get_logger.warn("Posição de decolagem não registrada! Pousando em (0,0).")
                self.landing_target_x = 0.0
                self.landing_target_y = 0.0
            self.landing_target_z = 0.0 # Z=0.0 para pousar no chão no NED

        # Estado: LANDING (Retorno para Ponto Inicial e Pouso)
        elif self.current_state == MissionState.LANDING:
            self.get_logger.info(f"Estado: LANDING - Movendo para o ponto de pouso (X:{self.landing_target_x:.2f}, Y:{self.landing_target_y:.2f}) e descendo para Z:{self.landing_target_z:.2f}.")
            
            # Publica o setpoint para o ponto de pouso final (X, Y e Z no chão)
            self.publish_trajectory_setpoint(self.landing_target_x, self.landing_target_y, self.landing_target_z)
            
            # Condições para considerar o pouso completo
            distance_to_landing_spot = np.sqrt(
                (current_x - self.landing_target_x)**2 +
                (current_y - self.landing_target_y)**2
            )
            
            # Transição: Pouso detectado (próximo ao ponto alvo e no chão ou drone desarmado)
            if (distance_to_landing_spot < 0.5 and abs(current_z - self.landing_target_z) < 0.1) or \
               self.vehicle_status.arming_state == self.vehicle_status.ARMING_STATE_DISARMED:
                # Se o drone está no chão (Z próximo de 0.0) e próximo do ponto alvo,
                # ou se o PX4 desarmou automaticamente (indicando pouso), a missão está completa.
                self.get_logger.info("Drone pousou com sucesso. Transicionando para MISSION_COMPLETE.")
                self.current_state = MissionState.MISSION_COMPLETE

        # Estado: MISSION_COMPLETE (Missão Finalizada)
        elif self.current_state == MissionState.MISSION_COMPLETE:
            self.get_logger.info("Estado: MISSION_COMPLETE - Missão finalizada. Desarmando e encerrando.")
            # Garante que o drone está desarmado.
            self.publish_vehicle_command(VehicleCommands.VEHICLE_CMD_ARM_DISARM, 0.0) # Param1 = 0.0 (desarmar)
            # Neste ponto, o nó continuará rodando, mas não enviará mais comandos de voo significativos.
            # O usuário deve encerrá-lo manualmente (Ctrl+C).

        # Estado: EMERGENCY_LAND (Pouso de Emergência - Adicionado)
        elif self.current_state == MissionState.EMERGENCY_LAND:
            self.get_logger.error("Estado: EMERGENCY_LAND. Realizando pouso de emergência!")
            # Envia comando de pouso (o PX4 cuida do resto)
            self.publish_vehicle_command(VehicleCommands.VEHICLE_CMD_NAV_LAND)
            # Transita para MISSION_COMPLETE uma vez que o drone esteja no chão e desarmado
            if self.vehicle_status.arming_state == self.vehicle_status.ARMING_STATE_DISARMED and abs(current_z - 0.0) < 0.1:
                self.get_logger.info("Pouso de emergência concluído e drone desarmado.")
                self.current_state = MissionState.MISSION_COMPLETE

# # --- Função Principal para Iniciar o Nó (necessário para um arquivo .py executável) ---
# def main(args=None):
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
