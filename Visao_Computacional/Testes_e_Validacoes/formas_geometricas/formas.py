import cv2
import numpy as np
import locale

locale.setlocale(locale.LC_ALL, 'Portuguese') #configura o locale para português do Brasil

# Detectar formas geométricas usando OpenCV
imagem = cv2.imread("figuras_planas.png") #carrega a imagem
if imagem is None:
    print("Erro ao carregar a imagem!")
    exit()


cinza = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY) #converter a imagem para escala de cinza
borrada = cv2.GaussianBlur(cinza, (5, 5), 0) #borra a imagem para suavizar e tirar ruídos
bordas = cv2.Canny(borrada, 50, 150) #detecção de bordas

contornos, _ = cv2.findContours(bordas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #encontra contoros e compacta

for contorno in contornos: 
        area = cv2.contourArea(contorno) #calcula a área do contorno 
        if area > 500: 
            perimetro = cv2.arcLength(contorno, True)
            aprox = cv2.approxPolyDP(contorno, 0.02 * perimetro, True) #resumir a borda para identificar pontos e assim deduzir ser um vértice, o True indica que o contorno é fechado.
            x, y, w, h, = cv2.boundingRect(aprox) #detecta uma "caixa envolvente"

            if len(aprox) == 3:
                forma = "Triangulo"
            elif len(aprox) == 4:
                razao = w / float(h) 
                forma = "Quadrado" if 0.95 <= razao <= 1.05 else "Retangulo"
            elif len(aprox) >= 4:
                forma = "Círculo ou Elipse"
            elif len(aprox) == 6:
                forma = "Hexagono"
            elif len(aprox) == 7:
                forma = "Heptagono"
            elif len(aprox) == 8:
                forma = "Octagono"
            else:
                forma = "Desconhecida"
        
            cv2.drawContours(imagem, [aprox], -1, (0, 255, 0), 2)
            cv2.putText(imagem, forma, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            #aqui vai colocar o nome da forma detectada na imagem, com fonte, tamanho e cor especificados
             # aqui o contorno vai expresso na webcam na cor verde,com espessura de borda 2
            if cv2.waitKey(1) & 0xFF == 27: #tecla ESC para sair 
                break
        print(area)
cv2.imshow("Formas Detectadas", imagem)
cv2.waitKey(0)
cv2.destroyAllWindows()