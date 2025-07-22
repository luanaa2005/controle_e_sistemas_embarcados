#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool # Manter Bool para o status de encontrado

# --- Configurações ---
COLOR_RANGES_HSV = {
    "Azul": ((100, 150, 50), (140, 255, 255)),
    "Marrom": ((10, 100, 20), (20, 255, 200)),
    "Vermelho1": ((0, 150, 50), (10, 255, 255)),
    "Vermelho2": ((160, 150, 50), (179, 255, 255)),
}
QR_TARGET_MAP = {
    "1": ("Quadrado", "Marrom"), "2": ("Triângulo", "Azul"), "3": ("Hexágono", "Vermelho")
}

current_target_shape = None
current_target_color = None

# --- Função Principal do Nó ---
def qr_shape_detector_node():
    global current_target_shape, current_target_color

    rospy.init_node("qr_shape_detector_node", anonymous=True)
    rate = rospy.Rate(30)

    # Publicadores essenciais
    qr_value_pub = rospy.Publisher("/qr_detector/qr_value", String, queue_size=1)
    target_found_pub = rospy.Publisher("/qr_detector/target_found", Bool, queue_size=1)
    image_pub = rospy.Publisher("/qr_detector/processed_image", Image, queue_size=5)

    bridge = CvBridge()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Câmera não disponível.")
        rospy.signal_shutdown("Falha na câmera.")
        return

    qr_detector = cv2.QRCodeDetector()
    rospy.loginfo("Nó iniciado.")

    last_qr_value_published = ""

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rate.sleep()
            continue

        display_frame = frame.copy()
        target_found_in_frame = False

        # Leitura QR Code
        qr_data, qr_points, _ = qr_detector.detectAndDecode(frame)
        if qr_points is not None:
            cv2.polylines(display_frame, [np.intp(qr_points)], True, (0, 255, 255), 3) # Desenha caixa QR
            if qr_data and qr_data in QR_TARGET_MAP:
                if qr_data != last_qr_value_published:
                    current_target_shape, current_target_color = QR_TARGET_MAP[qr_data]
                    rospy.loginfo(f"Alvo definido: {current_target_shape} {current_target_color} (QR: {qr_data})")
                    qr_value_pub.publish(String(qr_data))
                    last_qr_value_published = qr_data
                cv2.putText(display_frame, f"QR: {qr_data}", (int(qr_points[0][0][0]), int(qr_points[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            elif qr_data:
                cv2.putText(display_frame, f"QR Invalido", (int(qr_points[0][0][0]), int(qr_points[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Detecção de Formas e Cores (se houver alvo)
        if current_target_shape and current_target_color:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 400: continue

                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
                x, y, w, h = cv2.boundingRect(approx)
                if w == 0 or h == 0: continue
                
                shape_roi_hsv = cv2.cvtColor(frame[y:y+h, x:x+w], cv2.COLOR_BGR2HSV)

                # Detecção de cor
                detected_color = "Desconhecida"
                for c_name, (lower, upper) in COLOR_RANGES_HSV.items():
                    if "Vermelho" in c_name:
                        mask_red = cv2.bitwise_or(cv2.inRange(shape_roi_hsv, COLOR_RANGES_HSV["Vermelho1"][0], COLOR_RANGES_HSV["Vermelho1"][1]),
                                                  cv2.inRange(shape_roi_hsv, COLOR_RANGES_HSV["Vermelho2"][0], COLOR_RANGES_HSV["Vermelho2"][1]))
                        if np.sum(mask_red > 0) / mask_red.size > 0.3: detected_color = "Vermelho"; break
                    else:
                        mask = cv2.inRange(shape_roi_hsv, lower, upper)
                        if np.sum(mask > 0) / mask.size > 0.3: detected_color = c_name; break

                # Detecção de forma
                num_vertices = len(approx)
                detected_shape = "Desconhecida"
                if num_vertices == 3: detected_shape = "Triângulo"
                elif num_vertices == 4:
                    aspect_ratio = w / float(h)
                    detected_shape = "Quadrado" if 0.8 <= aspect_ratio <= 1.2 else "Retângulo"
                elif num_vertices == 6: detected_shape = "Hexágono"
                
                # Verifica alvo e desenha
                if detected_shape == current_target_shape and detected_color == current_target_color:
                    cv2.drawContours(display_frame, [approx], -1, (0, 255, 0), 3) # Verde
                    cv2.putText(display_frame, f"{detected_shape} {detected_color} (ALVO!)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    target_found_in_frame = True
                    break # Alvo encontrado, sai do loop de contornos
                elif detected_shape != "Desconhecida" and detected_color != "Desconhecida":
                    cv2.drawContours(display_frame, [approx], -1, (255, 0, 0), 2) # Azul
                    cv2.putText(display_frame, f"{detected_shape} {detected_color}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
        
        target_found_pub.publish(Bool(target_found_in_frame))
        
        # Exibição e Publicação da Imagem
        cv2.imshow("Detector de QR e Formas", display_frame)
        cv2.waitKey(1)

        try:
            image_pub.publish(bridge.cv2_to_imgmsg(display_frame, encoding="bgr8"))
        except Exception as e:
            rospy.logerr(f"Erro ao publicar imagem: {e}")

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()
    rospy.loginfo("Nó encerrado.")

if __name__ == '__main__':
    try:
        qr_shape_detector_node()
    except rospy.ROSInterruptException:
        pass