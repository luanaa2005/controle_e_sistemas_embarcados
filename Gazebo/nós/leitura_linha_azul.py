#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image       # Para publicar as imagens da câmera processadas
from std_msgs.msg import Float32        # Para publicar o valor do "erro"
from cv_bridge import CvBridge          # Ferramenta para converter imagens OpenCV para o formato ROS

import numpy as np                      # Já estava no código de visão
import cv2                              # Já estava no código de visão

# Faixa de azul em HSV (ajuste se necessário)
azulEscuro = np.array([100, 150, 50])
azulClaro  = np.array([140, 255, 255])

#Quando você define uma variável fora de qualquer função em Python, ela se torna 
#uma variável global (ou pelo menos de escopo de módulo). 
#Isso significa que ela pode ser acessada de qualquer lugar 
#dentro daquele arquivo Python, incluindo dentro da função talker().


def talker():
    #mudei o nome do nó
    rospy.init_node("vision_line_tracker_node", anonymous=True)
    loop_rate = rospy.Rate(30) # Processar a 30 frames por segundo

    #Nó de visão não vai publicar um String chamado "chatter". 
    # Ele vai publicar o erro (um Float32) 
    # e a imagem processada (um Image).

    error_pub = rospy.Publisher("/line_tracker/error", Float32, queue_size=10)
    processed_image_pub = rospy.Publisher("/line_tracker/processed_image", Image, queue_size=10)
    # Crie a instância do CvBridge aqui também
    bridge = CvBridge()

    # Inicialização da Câmera: A câmera só precisa ser aberta uma vez, antes de começar o loop principal.

    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        rospy.logerr("Não foi possível abrir a câmera. Verifique se ela está conectada e disponível.")
        rospy.signal_shutdown("Câmera não disponível.")
        return # Importante para sair da função se a câmera falhar
    
    rospy.loginfo("Nó 'vision_line_tracker_node' pronto para o processamento de frames.")
    while not rospy.is_shutdown():
    # AQUI É ONDE SEU CÓDIGO DE VISÃO ORIGINAL SERÁ COLOCADO
    # Faixa de azul em HSV (ajuste se necessário)


        sucesso, frame = camera.read()
        if not sucesso:
            rospy.logwarn("Falha ao capturar frame da câmera. Tentando novamente...")
            loop_rate.sleep() # Adiciona um pequeno atraso antes de tentar novamente
            continue # Pula para a próxima iteração do loop

        altura, largura, _ = frame.shape

        # Região de interesse: parte inferior da imagem
        roi = frame[int(0.7 * altura):, :]

        # Converte para HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Máscara para cor azul
        obj = cv2.inRange(hsv, azulEscuro, azulClaro)
        obj = cv2.GaussianBlur(obj, (3, 3), 0)

        # Calcula o centro da linha com momentos
        M = cv2.moments(obj)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

        # Desenha ponto no centro da linha
        cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)

        # Linha central da imagem
        cv2.line(roi, (largura // 2, 0), (largura // 2, roi.shape[0]), (255, 255, 0), 2)

        # Linha verde mostrando o desvio
        cv2.line(roi, (largura // 2, cy), (cx, cy), (0, 255, 0), 2)

        # Calcula o erro (desvio do centro)
        erro = cx - (largura // 2)
        rospy.loginfo(f"Erro de posição: {erro} px")
        error_pub.publish(Float32(erro))
        # Exibe os resultados
        cv2.imshow("Tracking", roi)
        cv2.imshow("Binary", obj)
        # --- PUBLICAÇÃO DA IMAGEM PROCESSADA ---
        try:
            img_msg = bridge.cv2_to_imgmsg(roi, encoding="bgr8") # 'roi' é sua imagem processada
            processed_image_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Erro ao converter e publicar imagem: {e}")
        # --- FIM DA PUBLICAÇÃO ---

    
        
        loop_rate.sleep()
    camera.release()
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass