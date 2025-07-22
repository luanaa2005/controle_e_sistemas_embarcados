#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String


# --- Definições de Limites de Cores ( EM HSV) ---

COLOR_RANGES_HSV = {
    "Azul": ((100, 150, 50), (140, 255, 255)),
    "Marrom": ((10, 100, 20), (20, 255, 200)),
    "Vermelho1": ((0, 150, 50), (10, 255, 255)),     # Parte 1 do vermelho (hue de 0 a 10)
    "Vermelho2": ((160, 150, 50), (179, 255, 255)),   # Parte 2 do vermelho (hue de 160 a 179)
}

# --- Função Principal do Nó ROS ---
def shape_color_detector_node():
    rospy.init_node("shape_color_detector_node", anonymous=True)
    rate = rospy.Rate(30) # 30 Hz

    shape_color_pub = rospy.Publisher("/shape_detector/detected_shape_color", String, queue_size=10)
    image_pub = rospy.Publisher("/shape_detector/processed_image", Image, queue_size=5)

    bridge = CvBridge()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Erro ao acessar a câmera! Verifique se está conectada.")
        rospy.signal_shutdown("Câmera não disponível.")
        return

    rospy.loginfo("Nó 'shape_color_detector_node' iniciado e pronto para detectar formas e cores (HSV).")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn_throttle(5, "Falha ao capturar frame da câmera. Tentando novamente...")
            rate.sleep()
            continue

        display_frame = frame.copy()

        # Pré-processamento
        gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Detecção de Contornos
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_info = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500: # Filtra contornos pequenos
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
                x, y, w, h = cv2.boundingRect(approx)

                if w == 0 or h == 0: continue

                # Recorta a região da forma e converte para HSV para análise de cor
                shape_roi = frame[y:y+h, x:x+w]
                if shape_roi.size == 0: continue
                
                shape_roi_hsv = cv2.cvtColor(shape_roi, cv2.COLOR_BGR2HSV)

                # --- Detecção de Cor usando HSV ---
                detected_color = "Desconhecida"
                
                # Trata o vermelho com duas faixas de HUE
                mask_red1 = cv2.inRange(shape_roi_hsv, COLOR_RANGES_HSV["Vermelho1"][0], COLOR_RANGES_HSV["Vermelho1"][1])
                mask_red2 = cv2.inRange(shape_roi_hsv, COLOR_RANGES_HSV["Vermelho2"][0], COLOR_RANGES_HSV["Vermelho2"][1])
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)
                
                percent_red = np.sum(mask_red > 0) / mask_red.size
                if percent_red > 0.3: # Limite para considerar a cor dominante
                    detected_color = "Vermelho"
                else: # Se não é vermelho, verifica outras cores
                    for color_name, (lower, upper) in COLOR_RANGES_HSV.items():
                        if "Vermelho" not in color_name: # Já tratamos vermelho
                            mask = cv2.inRange(shape_roi_hsv, lower, upper)
                            percent_color = np.sum(mask > 0) / mask.size
                            if percent_color > 0.3: # Limite para considerar a cor dominante
                                detected_color = color_name
                                break # Encontrou uma cor, pode parar de verificar

                # --- Identificação da Forma ---
                num_vertices = len(approx)
                forma = "Desconhecida"
                if num_vertices == 3: forma = "Triângulo"
                elif num_vertices == 4:
                    aspect_ratio = w / float(h)
                    forma = "Quadrado" if 0.95 <= aspect_ratio <= 1.05 else "Retângulo"
                elif num_vertices == 6: forma = "Hexágono"
                else: forma = f"{num_vertices} lados"

                rospy.loginfo_throttle(1, f"Detectado: {forma} {detected_color}")

                # Desenha contornos (roxo claro)
                cv2.drawContours(display_frame, [approx], -1, (255, 0, 255), 1)

                # Se for uma das formas e cores esperadas, destaca em verde
                # Estes são os "alvos" que você quer destacar
                if (forma == "Triângulo" and detected_color == "Azul") or \
                   (forma == "Quadrado" and detected_color == "Marrom") or \
                   (forma == "Hexágono" and detected_color == "Vermelho"):
                    
                    texto = f"{forma} {detected_color}"
                    cv2.drawContours(display_frame, [approx], -1, (0, 255, 0), 2) # Contorno verde
                    cv2.putText(display_frame, texto, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    detected_info.append(texto)

        # Publica todas as detecções do frame
        if detected_info:
            shape_color_pub.publish(String(", ".join(detected_info)))
        else:
            shape_color_pub.publish(String("Nenhuma forma/cor alvo detectada"))

        # Exibe a imagem e publica para ROS
        cv2.imshow("Detecção de Formas e Cores ao vivo (HSV)", display_frame)
        cv2.waitKey(1)

        try:
            ros_image = bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
            image_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"Erro ao converter e publicar imagem ROS: {e}")

        rate.sleep()

    # Libera Recursos na Saída do Nó
    cap.release()
    cv2.destroyAllWindows()
    rospy.loginfo("Nó 'shape_color_detector_node' encerrado.")

# --- Ponto de Entrada Principal ---
if __name__ == '__main__':
    try:
        shape_color_detector_node()
    except rospy.ROSInterruptException:
        pass