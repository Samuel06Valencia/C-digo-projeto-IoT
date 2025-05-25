# Projeto IoT - Monitoramento de Reservatórios com ESP32 e MQTT

## Descrição
Este projeto utiliza o ESP32 para monitorar o nível de água em um reservatório usando um sensor ultrassônico. Quando detecta um nível crítico, aciona um servo motor (simulando uma válvula) e um buzzer (alerta sonoro). O sistema envia dados automaticamente para a nuvem usando o protocolo MQTT via broker HiveMQ.

## Componentes Utilizados
- ESP32 DevKit V4
- Sensor Ultrassônico HC-SR04
- Servo motor SG90
- Buzzer passivo
- Simulador Wokwi

## Funcionamento
- Leitura contínua da distância via HC-SR04.
- Publicação da distância no tópico MQTT `esp32/distancia`.
- Acionamento automático de atuadores quando detectada distância crítica.
- Totalmente funcional sem envio manual de comandos.

## Comunicação
- Protocolo: MQTT v3.1.1 (mqtt.org)
- Broker: HiveMQ (broker.hivemq.com)
- Tópicos usados:
  - `esp32/distancia` (publicação automática de leitura)
  - `esp32/comando` (para comandos futuros)

## Requisitos
- MicroPython no ESP32
- Conexão Wi-Fi
- Broker MQTT (público ou privado)
- IDE de desenvolvimento como Thonny ou Wokwi

## Autor
Samuel Silva Valencia – Universidade Presbiteriana Mackenzie
