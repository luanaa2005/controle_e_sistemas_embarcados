# Explicação do Código: Detecção de Formas Geométricas com OpenCV

Este script Python utiliza a biblioteca **OpenCV** para detectar formas geométricas (como triângulos, quadrados, retângulos e círculos) em uma imagem estática. A detecção é feita com base nos contornos encontrados após pré-processamento da imagem.

---


### 1. Configuração de Localização

O script configura o locale para o idioma português do Brasil, o que pode ser útil para tratar textos ou formatações regionais.

---

### 2. Leitura da Imagem

O código começa carregando uma imagem chamada `figuras_planas.png`. Se a imagem não for encontrada, ele exibe uma mensagem de erro e encerra.

---

### 3. Pré-processamento

A imagem passa pelas seguintes transformações:
- Conversão para escala de cinza.
- Aplicação de um desfoque Gaussiano para suavizar ruídos.
- Detecção de bordas usando o algoritmo **Canny**, que destaca os contornos.

---

### 4. Detecção de Contornos

O script utiliza `cv2.findContours` para encontrar contornos externos na imagem. Cada contorno detectado é analisado individualmente.

---

### 5. Filtragem e Aproximação

Para evitar pequenos ruídos, só são considerados os contornos com área superior a 500 pixels.

Cada contorno é:
- Aproximado com o algoritmo `approxPolyDP`, que reduz o número de vértices do contorno para facilitar a identificação da forma.
- Envolvido por um retângulo delimitador (`boundingRect`), de onde se obtém altura e largura para comparar proporções.

---

### 6. Classificação das Formas

Com base na quantidade de vértices da forma aproximada:
- 3 vértices → **Triângulo**
- 4 vértices → **Quadrado** (se largura ≈ altura) ou **Retângulo**
- 6 a 8 vértices → **Hexágono**, **Heptágono**, **Octógono**
- ≥ 4 vértices sem proporções definidas → **Círculo ou Elipse**
- Outros → **Desconhecida**

---

### 7. Visualização

Para cada forma detectada:
- O contorno é desenhado em verde.
- O nome da forma é exibido acima da figura com texto em verde.
- A imagem com os resultados é exibida em uma janela chamada `"Formas Detectadas"`.

---

### 8. Encerramento

A execução aguarda uma tecla ser pressionada para encerrar (`cv2.waitKey(0)`) e fecha todas as janelas com `cv2.destroyAllWindows()`.

---

