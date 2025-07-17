import cv2
import numpy as np

# === Etapa 1: Ler QR code com a webcam ===
webcam = cv2.VideoCapture(0)
detector = cv2.QRCodeDetector()

print("Posicione o QR code diante da câmera...")

valor_lido = ""
while True:
    ret, frame = webcam.read()
    if not ret:
        break

    valor, pontos, _ = detector.detectAndDecode(frame)
    cv2.imshow("Leitura QR Code", frame)

    if valor != "":
        valor_lido = valor
        print(f"QR Code detectado: {valor_lido}")
        break

    if cv2.waitKey(1) == 27:  # ESC
        break

if valor_lido not in ["1", "2", "3"]:
    print("QR Code inválido. Use 1, 2 ou 3.")
    webcam.release()
    cv2.destroyAllWindows()
    exit()

# === Etapa 2: Detectar formas ao vivo com base no QR ===
alvo = {
    "1": ("Quadrado", "Marrom"),
    "2": ("Triângulo", "Azul"),
    "3": ("Hexágono", "Vermelho")
}
forma_alvo, cor_alvo = alvo[valor_lido]

print(f"Buscando: {forma_alvo} {cor_alvo}")

encontrado = False

# Limites HSV para as cores alvo
limites_hsv = {
    "Azul": ((100, 150, 50), (140, 255, 255)),
    "Marrom": ((10, 100, 20), (20, 255, 200)),
    "Vermelho1": ((0, 150, 50), (10, 255, 255)),
    "Vermelho2": ((160, 150, 50), (179, 255, 255)),
}

while True:
    ret, frame = webcam.read()
    if not ret:
        break

    cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    borrada = cv2.GaussianBlur(cinza, (5, 5), 0)
    bordas = cv2.Canny(borrada, 50, 150)
    contornos, _ = cv2.findContours(bordas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area < 400:  # Ajuste área mínima
            continue

        perimetro = cv2.arcLength(contorno, True)
        aprox = cv2.approxPolyDP(contorno, 0.02 * perimetro, True)
        x, y, w, h = cv2.boundingRect(aprox)

        recorte = frame[y:y+h, x:x+w]
        if recorte.size == 0:
            continue

        recorte_hsv = cv2.cvtColor(recorte, cv2.COLOR_BGR2HSV)

        mask_azul = cv2.inRange(recorte_hsv, limites_hsv["Azul"][0], limites_hsv["Azul"][1])
        mask_marrom = cv2.inRange(recorte_hsv, limites_hsv["Marrom"][0], limites_hsv["Marrom"][1])
        mask_vermelho1 = cv2.inRange(recorte_hsv, limites_hsv["Vermelho1"][0], limites_hsv["Vermelho1"][1])
        mask_vermelho2 = cv2.inRange(recorte_hsv, limites_hsv["Vermelho2"][0], limites_hsv["Vermelho2"][1])
        mask_vermelho = cv2.bitwise_or(mask_vermelho1, mask_vermelho2)

        percent_azul = np.sum(mask_azul > 0) / mask_azul.size
        percent_marrom = np.sum(mask_marrom > 0) / mask_marrom.size
        percent_vermelho = np.sum(mask_vermelho > 0) / mask_vermelho.size

        cor_dominante = "Desconhecida"
        if percent_azul > 0.3:
            cor_dominante = "Azul"
        elif percent_marrom > 0.3:
            cor_dominante = "Marrom"
        elif percent_vermelho > 0.3:
            cor_dominante = "Vermelho"

        vertices = len(aprox)
        forma = "Desconhecida"

        if vertices == 3:
            forma = "Triângulo"
        elif vertices == 4:
            razao = w / float(h)
            forma = "Quadrado" if 0.9 <= razao <= 1.1 else "Retângulo"
        elif vertices == 6:
            forma = "Hexágono"

        # Debug prints (opcional)
        # print(f"Forma detectada: {forma}, Cor detectada: {cor_dominante}")

        if forma == forma_alvo and cor_dominante == cor_alvo:
            texto = f"{forma} {cor_dominante} (alvo)"
            cv2.drawContours(frame, [aprox], -1, (0, 255, 0), 3)
            cv2.putText(frame, texto, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            encontrado = True
            break

    if encontrado:
        print(f"{forma_alvo} {cor_alvo} detectado com sucesso!")
        cv2.imshow("Resultado da Missão", frame)
        cv2.waitKey(0)
        break
    else:
        cv2.imshow("Detectando formas...", frame)

    if cv2.waitKey(1) == 27:  # ESC
        break

webcam.release()
cv2.destroyAllWindows()
