#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String # Importado para publicar o texto do QR Code


def talker():
    # --- Configuração Inicial do Nó ROS ---
    # Código base: rospy.init_node("talker_python", anonymous=False)
    # Modificado: 
    rospy.init_node("qr_code_reader_node", anonymous=True)

    # Código base: loop_rate = rospy.Rate(10)
    # Modificado: Taxa de processamento de 30 Hz.
    loop_rate = rospy.Rate(30)

    # Código base: chatter_pub = rospy.Publisher("chatter", String, queue_size = 100)
    # Modificado: Publicadores específicos para o texto decodificado do QR Code e a imagem processada.
    qr_text_pub = rospy.Publisher("/qr_reader/text", String, queue_size=10)
    processed_image_pub = rospy.Publisher("/qr_reader/processed_image", Image, queue_size=10)

    # Instância do CvBridge para converter imagens entre OpenCV e o formato de mensagens ROS.
   
    bridge = CvBridge()

    # --- Inicialização de Hardware (Câmera) e Ferramentas de Visão (Detector QR Code) ---

    camera = cv2.VideoCapture(0) # Inicializa a câmera (índice 0 para a câmera padrão)
    if not camera.isOpened(): # Verifica se a câmera abriu corretamente
        rospy.logerr("Não foi possível abrir a câmera. Verifique se ela está conectada e disponível.")
        rospy.signal_shutdown("Câmera não disponível.") # Sinaliza ao ROS para desligar o nó
        return # Sai da função talker() se a câmera não puder ser aberta

    detector = cv2.QRCodeDetector() # Inicializa o detector de QR Code do OpenCV

    rospy.loginfo("Nó 'qr_code_reader_node' pronto para o processamento de frames.")

    # Código base:
    # count = 0
    # while not rospy.is_shutdown():
    #     txt = "Ola ROS Python! Contagem: " + str(count)
    #     rospy.loginfo(txt)
    #     chatter_pub.publish(txt)
    #     loop_rate.sleep()
    #     count = count + 1
    #     webcam = cv2.VideoCapture(0) # <-- Era inicializado incorretamente dentro do loop
    #     detector = cv2.QRCodeDetector() # <-- Era inicializado incorretamente dentro do loop
    #     for i in range(1, 4): # <-- Código de GERAÇÃO de QR Code (removido)
    #         img = qrcode.make(str(i))
    #         img.save(f"qr_{i}.png")

    while not rospy.is_shutdown():
        # --- Captura e Processamento de Cada Frame da Câmera ---
        # Código base: validacao, frame = webcam.read()
        # Modificado:
        sucesso, frame = camera.read()
        if not sucesso:
            rospy.logwarn("Falha ao capturar frame da câmera. Tentando novamente...")
            loop_rate.sleep() # Adiciona um pequeno atraso antes de tentar novamente
            continue # Pula para a próxima iteração do loop

        # --- Detecção e Decodificação do QR Code ---
        # Código base:
        # while validacao: 
        #     validacao, frame = webcam.read()
        #     valor, pontos, qrcode = detector.detectAndDecode(frame)

        # Modificado: 
        valor, pontos, qrcode_bbox = detector.detectAndDecode(frame) # qrcode_bbox são os dados binários do QR Code, não a biblioteca

        # --- Processamento e Publicação (se QR Code for detectado) ---
        if valor != "": # Se um valor foi decodificado do QR Code
            # --- Desenho da Bounding Box e Texto no Frame ---
            if pontos is not None: # Verifica se os pontos da bounding box foram retornados
                # Desenha a bounding box ao redor do QR Code
                pontos = np.intp(pontos) # Garante que os pontos são inteiros para cv2.polylines
                cv2.polylines(frame, [pontos], True, (0, 255, 0), 5) # Desenha um polígono verde
                x = int(pontos[0][0][0]) # X do primeiro ponto (superior esquerdo)
                y = int(pontos[0][0][1]) # Y do primeiro ponto (superior esquerdo)

                # Código base: cv2.putText(frame, str(valor), (20,35), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,0),4)
                # Modificado: 
                cv2.putText(frame, valor, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2, cv2.LINE_AA)
                rospy.loginfo(f"QR Code Detectado: {valor}") # Loga o valor decodificado

            # --- Publicação do Texto do QR Code ---
            qr_text_pub.publish(String(valor)) # Publica o texto decodificado no tópico ROS

        # --- Exibição e Publicação da Imagem Processada ---
        # Código base:
        # cv2.imshow("Video da webcam", frame)
        # key = cv2.waitKey(5)
        # if key == 27: #ESC(Teclas tem um número específico)
        #     break

        cv2.imshow("QR Code Reader", frame) # Exibe a imagem com as marcações
        # O waitKey é necessário para que cv2.imshow funcione. 1ms é suficiente.
        cv2.waitKey(1)

        # --- Publicação da Imagem Processada para o ROS ---
        try:
            # Converte a imagem OpenCV (BGR8) para uma mensagem ROS Image
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            processed_image_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Erro ao converter e publicar imagem: {e}")

        # --- Controle de Frequência do Loop ---
        # Garante que o nó rode na taxa definida (30 Hz).
        loop_rate.sleep()

    camera.release() # Libera o recurso da câmera
    cv2.destroyAllWindows() # Fecha todas as janelas OpenCV
    rospy.loginfo("Nó de leitura de QR Code encerrado.")


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass