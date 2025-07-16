# Explicação do Código: Rastreamento de Linha Azul com Cálculo de Erro (OpenCV + HSV)

Este script Python realiza o rastreamento de uma linha azul capturada pela webcam utilizando **OpenCV** e **NumPy**. Ele processa apenas a região inferior da imagem, detecta a linha azul com base na cor em HSV, calcula o centro da linha e determina o erro de posição em relação ao centro da imagem. Esse tipo de aplicação é comum em robótica, veículos autônomos e drones seguidores de trajetória.

---


### 1. Definição da Faixa de Cor Azul (HSV)

O código define uma faixa de tonalidade azul no espaço de cor HSV, pois este modelo é mais robusto a variações de iluminação. A máscara binária será gerada com base nessa faixa, permitindo isolar a cor azul na imagem.

---

### 2. Captura da Imagem

A webcam é iniciada e, em um laço contínuo, os frames são capturados em tempo real. Isso permite o processamento e análise dinâmica do vídeo.

---

### 3. Seleção da Região de Interesse (ROI)

Para tornar o processamento mais eficiente, o script considera apenas a parte inferior da imagem. Essa área é onde geralmente a linha azul está localizada, por exemplo, no chão de uma pista de navegação. Isso reduz o ruído e melhora o desempenho da detecção.

---

### 4. Conversão para o Espaço de Cor HSV

A região de interesse é convertida de BGR (formato padrão do OpenCV) para HSV, o que permite aplicar a máscara de cor com mais precisão e estabilidade, mesmo sob variações de luz.

---

### 5. Criação da Máscara Binária

Uma máscara binária é criada, onde os pixels que correspondem à faixa de azul definida aparecem em branco (valor 255), e o restante em preto (valor 0). Em seguida, aplica-se um desfoque gaussiano para suavizar a imagem e eliminar pequenas imperfeições ou ruídos.

---

### 6. Cálculo do Centro da Linha com Momentos

Com a imagem binária, são calculados os **momentos de área**. Caso uma linha azul seja detectada, calcula-se seu centroide, que representa o ponto central da linha. Este ponto é essencial para determinar a posição da linha dentro da imagem.

---

### 7. Visualização Gráfica

Para facilitar o entendimento visual do rastreamento:
- Um ponto vermelho é desenhado no centro da linha detectada.
- Uma linha amarela marca o centro da imagem.
- Uma linha verde conecta o centro da imagem ao centro detectado da linha azul.

Esses elementos permitem visualizar se a linha está centralizada ou deslocada.

---

### 8. Cálculo do Erro de Posição

O erro de posição é calculado como a diferença horizontal entre o centro da imagem e o centro da linha azul detectada. Esse valor pode ser usado em sistemas de controle para corrigir a trajetória de um robô ou drone, reposicionando-o para alinhar com a linha.

---

### 9. Exibição dos Resultados

Duas janelas são exibidas:
- Uma com a imagem original (região de interesse) com os elementos gráficos desenhados.
- Outra com a imagem binária (máscara da cor azul) mostrando apenas os pixels detectados.

---

### 10. Encerramento

O programa é encerrado ao pressionar a tecla `q`, garantindo que os recursos da câmera sejam liberados e as janelas de exibição fechadas corretamente.

---
