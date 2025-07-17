import cv2
import numpy as np

# Inicia a webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Erro ao acessar a câmera!")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    imagem = frame.copy()

    # Pré-processamento
    cinza = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)
    borrada = cv2.GaussianBlur(cinza, (5, 5), 0)
    bordas = cv2.Canny(borrada, 50, 150)

    # Detecta contornos
    contornos, _ = cv2.findContours(bordas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area > 500:
            perimetro = cv2.arcLength(contorno, True)
            aprox = cv2.approxPolyDP(contorno, 0.02 * perimetro, True)
            x, y, w, h = cv2.boundingRect(aprox)

            # Recorta a forma para analisar a cor média
            recorte = imagem[y:y+h, x:x+w]
            media_cor = cv2.mean(recorte)[:3]
            b, g, r = map(int, media_cor)

            vertices = len(aprox)
            forma = "Desconhecida"
            cor_dominante = "Desconhecida"

            # Tenta identificar a forma
            if vertices == 3:
                forma = "Triângulo"
                if b > r + 40 and b > g + 40:
                    cor_dominante = "Azul"
            elif vertices == 4:
                razao = w / float(h)
                forma = "Quadrado" if 0.95 <= razao <= 1.05 else "Retângulo"
                if r > 100 and g > 60 and b < 100:
                    cor_dominante = "Marrom"
            elif vertices == 6:
                forma = "Hexágono"
                if r > g + 40 and r > b + 40:
                    cor_dominante = "Vermelho"
            else:
                forma = f"{vertices} lados"

            print(f"Detectado: {forma} | Cor média: R={r}, G={g}, B={b}")

            # Desenha todos os contornos para debug
            cv2.drawContours(imagem, [aprox], -1, (255, 0, 255), 1)

            # Se for forma e cor esperada, destaca
            if (forma == "Triângulo" and cor_dominante == "Azul") or \
               (forma == "Quadrado" and cor_dominante == "Marrom") or \
               (forma == "Hexágono" and cor_dominante == "Vermelho"):

                texto = f"{forma} {cor_dominante}"
                cv2.drawContours(imagem, [aprox], -1, (0, 255, 0), 2)
                cv2.putText(imagem, texto, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Exibe a imagem processada
    cv2.imshow("Detecção ao vivo", imagem)

    # Pressione 'q' para sair
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera recursos
cap.release()
cv2.destroyAllWindows()
