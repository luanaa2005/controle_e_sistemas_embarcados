import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Para receber a imagem da câmera
from geometry_msgs.msg import Point # Para publicar a posição da linha
from cv_bridge import CvBridge # Para converter mensagens ROS Image para imagens OpenCV

import cv2
import numpy as np

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector_node')

        # Cria um assinante para o tópico da imagem da câmera
        # O tópico pode variar, verifique qual seu drone publica (ex: /camera/image_raw, /depth_camera/image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # <--- VERIFIQUE E AJUSTE ESTE TÓPICO!
            self.image_callback,
            10 # QoS depth
        )
        self.subscription # Evita aviso de variável não usada

        # Cria um publicador para a posição central da linha
        self.publisher = self.create_publisher(Point, '/drone/line_center', 10)

        # Inicializa o CvBridge para conversão de imagem
        self.bridge = CvBridge()

        self.get_logger().info('Line Detector Node has been started.')

    def image_callback(self, msg):
        """
        Callback que é chamada toda vez que uma nova imagem é recebida.
        Processa a imagem para detectar a linha azul e publica sua posição.
        """
        try:
            # Converte a mensagem ROS Image para uma imagem OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Obtenha as dimensões da imagem
        height, width, _ = cv_image.shape

        # --- Processamento de Imagem para Detecção da Linha Azul ---

        # 1. Converter para o espaço de cores HSV (Hue, Saturation, Value)
        # O HSV é melhor para detecção de cores do que BGR/RGB
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. Definir os limites para a cor azul na escala HSV
        # Estes valores podem precisar de AJUSTES dependendo das condições de iluminação e da tonalidade do azul.
        # Use ferramentas como `trackbars` do OpenCV ou `color picker` para encontrar os valores ideais.
        # Exemplo para azul:
        # lower_blue = np.array([90, 50, 50])   # Limite inferior de Hue, Saturation, Value
        # upper_blue = np.array([130, 255, 255]) # Limite superior

        # Um range um pouco mais genérico para azul (pode precisar de ajuste fino)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # 3. Criar uma máscara para isolar a cor azul
        # Tudo que estiver dentro do range azul vira branco (255), o resto vira preto (0)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 4. Aplicar operações morfológicas para limpar o ruído na máscara
        # Opcional, mas ajuda a remover pequenos pontos e unir regiões azuis
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)   # Erosão: remove pequenos pontos brancos
        mask = cv2.dilate(mask, kernel, iterations=1)  # Dilatação: expande regiões brancas

        # 5. Encontrar contornos na máscara
        # Contornos representam as bordas das regiões azuis
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_center_x = -1.0 # Valor padrão se a linha não for encontrada

        if len(contours) > 0:
            # Encontra o maior contorno (assumindo que a linha é o maior objeto azul)
            largest_contour = max(contours, key=cv2.contourArea)

            # Calcula o "momento" do contorno para encontrar seu centro (centroid)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0: # Evita divisão por zero
                cx = int(M['m10'] / M['m00']) # Coordenada X do centro do contorno
                cy = int(M['m01'] / M['m00']) # Coordenada Y do centro do contorno (não usaremos diretamente aqui)

                line_center_x = float(cx) # Converte para float para o tipo Point

                # Opcional: Desenha um círculo no centro detectado para visualização (apenas para debug)
                # cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1) # Verde

        # --- Publicar a Posição da Linha ---
        point_msg = Point()
        point_msg.x = line_center_x # Posição X do centro da linha na imagem
        point_msg.y = float(width)  # Opcional: Largura da imagem para normalização ou referência
        point_msg.z = float(height) # Opcional: Altura da imagem para normalização ou referência
        
        self.publisher.publish(point_msg)
        # self.get_logger().info(f"Published line center X: {line_center_x}")

        # Opcional: Mostrar a imagem processada (para debug)
        # cv2.imshow("Original Image", cv_image)
        # cv2.imshow("Masked Image (Blue Line)", mask)
        # cv2.waitKey(1) # Necessário para atualizar as janelas do OpenCV

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector) # Mantém o nó rodando e processando callbacks
    line_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    