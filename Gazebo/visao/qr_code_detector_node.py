import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Para receber a imagem da câmera
from geometry_msgs.msg import Point # Para publicar o ID e a posição do QR Code
from cv_bridge import CvBridge # Para converter mensagens ROS Image para imagens OpenCV

import cv2
from pyzbar.pyzbar import decode # Para decodificar QR Codes
import numpy as np

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector_node')

        # Cria um assinante para o tópico da imagem da câmera
        # O tópico pode variar, verifique qual seu drone publica (ex: /camera/image_raw, /depth_camera/image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # <--- VERIFIQUE E AJUSTE ESTE TÓPICO!
            self.image_callback,
            10 # QoS depth
        )
        self.subscription # Evita aviso de variável não usada

        # Cria um publicador para os dados do QR Code (ID e posição na imagem)
        # O campo 'x' do Point será o ID do QR Code
        # Os campos 'y' e 'z' do Point serão as coordenadas (x, y) do centro do QR Code na imagem
        self.publisher = self.create_publisher(Point, '/drone/qr_code_data', 10)

        # Inicializa o CvBridge para conversão de imagem
        self.bridge = CvBridge()

        # Flag para garantir que publicamos o QR Code apenas uma vez por detecção
        # ou até que o drone se mova para longe do QR Code atual.
        # Para esta missão, vamos considerar que só precisamos do primeiro QR detectado.
        self.qr_code_detected_and_published = False

        self.get_logger().info('QR Code Detector Node has been started.')

    def image_callback(self, msg):
        """
        Callback que é chamada toda vez que uma nova imagem é recebida.
        Processa a imagem para detectar e decodificar QR Codes.
        """
        if self.qr_code_detected_and_published:
            # Se já detectamos e publicamos um QR Code, não processamos mais até resetar.
            # Em uma missão real, você pode querer um mecanismo de reset (ex: drone se afastou)
            return

        try:
            # Converte a mensagem ROS Image para uma imagem OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Converter a imagem para escala de cinza, o que é melhor para a detecção de códigos
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Usar pyzbar para decodificar códigos de barras e QR Codes
        decoded_objects = decode(gray_image)

        if len(decoded_objects) > 0:
            for obj in decoded_objects:
                # Verifica se o tipo do objeto decodificado é um QR Code
                if obj.type == 'QRCODE':
                    qr_data = obj.data.decode('utf-8') # Decodifica os bytes para string
                    self.get_logger().info(f"QR Code detectado! Dados: {qr_data}")

                    try:
                        # Tenta converter os dados do QR Code para um inteiro (o ID da zona)
                        qr_id = int(qr_data)
                    except ValueError:
                        self.get_logger().warn(f"Dados do QR Code '{qr_data}' não são um número inteiro válido. Ignorando.")
                        continue # Pula para o próximo objeto decodificado

                    # Pega as coordenadas do bounding box do QR Code
                    (x, y, w, h) = obj.rect
                    
                    # Calcula o centro do QR Code na imagem
                    center_x = x + w / 2
                    center_y = y + h / 2

                    # --- Publicar os Dados do QR Code ---
                    point_msg = Point()
                    point_msg.x = float(qr_id)    # ID do QR Code
                    point_msg.y = float(center_x) # Posição X do centro do QR Code na imagem
                    point_msg.z = float(center_y) # Posição Y do centro do QR Code na imagem
                    
                    self.publisher.publish(point_msg)
                    self.get_logger().info(f"Publicado QR Code ID: {qr_id}, Centro X: {center_x:.2f}, Y: {center_y:.2f}")
                    
                    # Marca que um QR Code foi detectado e publicado
                    self.qr_code_detected_and_published = True
                    return # Publica apenas o primeiro QR Code válido encontrado por imagem

                # Opcional: Desenhar o bounding box e o texto do QR Code na imagem para debug
                # cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # cv2.putText(cv_image, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Opcional: Mostrar a imagem processada (para debug)
        # cv2.imshow("QR Code Detector Feed", cv_image)
        # cv2.waitKey(1) # Necessário para atualizar as janelas do OpenCV

def main(args=None):
    rclpy.init(args=args)
    qr_code_detector = QRCodeDetector()
    rclpy.spin(qr_code_detector) # Mantém o nó rodando e processando callbacks
    qr_code_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    