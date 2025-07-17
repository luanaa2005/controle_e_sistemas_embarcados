# Código de Controle de Drone

Este programa utiliza a biblioteca DroneKit para conectar-se a um drone, armar os motores, decolar até uma altitude específica, e depois aterrissar automaticamente.

---

## Funcionamento do Código

### 1. Função `arm_and_takeoff(targetHeight)`

Esta função realiza o processo de armar o drone e decolar até a altitude desejada (em metros).

- Espera até que o drone esteja pronto para ser armado (`vehicle.is_armable`).
- Muda o modo de voo para `GUIDED`, que permite controle via comandos.
- Arma os motores do drone (`vehicle.armed = True`).
- Envia o comando para decolagem até a altitude `targetHeight` com `vehicle.simple_takeoff(targetHeight)`.
- Enquanto o drone não atingir pelo menos 95% da altitude alvo, monitora e imprime a altitude atual.
- Quando a altitude alvo é alcançada, informa no console que o objetivo foi atingido.

### 2. Execução principal

- Conecta-se ao veículo na porta local `127.0.0.1:14550`, normalmente usada por simuladores como o SITL (Software In The Loop).
- Chama a função `arm_and_takeoff(1.5)` para decolar até 1,5 metros.
- Após atingir a altitude, altera o modo de voo para `LAND` para iniciar o pouso automático.
- Aguarda até que o modo seja confirmado como `LAND` e imprime mensagens informativas.

---

## Resumo

O código demonstra um fluxo básico para controlar um drone via DroneKit:

- Conexão ao veículo (simulado ou real),
- Armar e decolar até uma altitude segura,
- Alterar o modo para pouso automático.

---

