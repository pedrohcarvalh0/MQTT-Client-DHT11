
# Projeto: Coleta de Dados DHT11 + FFT + Publicação via MQTT com RP2040 (Raspberry Pi Pico W)

## Descrição

Este código foi desenvolvido para rodar na placa **Raspberry Pi Pico W**, coletando dados de temperatura e umidade através de um sensor **DHT11**, realizando uma **Transformada Rápida de Fourier (FFT)** em tempo real e enviando os resultados via **MQTT** para um broker.

## Pré-requisitos

- **Placa**: Raspberry Pi Pico W (RP2040)
- **Sensor**: DHT11
- **Outros Periféricos**: Display OLED SSD1306, Matriz de LEDs WS2812, Buzzer, LED RGB
- **Broker MQTT**: Um broker ativo (exemplo: Mosquitto)
- **Configuração do broker**: https://youtu.be/UmmK6MiXOqM?si=WFZpqj1_aV5oBVlE

## Configurações obrigatórias antes de compilar

No início do arquivo `mqtt_client_dht11.c`, altere as seguintes definições para refletirem sua rede e broker:

```c
#define WIFI_SSID "NOME DA SUA REDE WIFI"
#define WIFI_PASSWORD "SENHA DA SUA REDE WIFI"
#define MQTT_SERVER "ENDEREÇO IP DO BROKER"
#define MQTT_USERNAME "USERNAME DEFINIDO NO BROKER"
#define MQTT_PASSWORD "SENHA DEFINIDA NO BROKER"
```

## Compilação

Este projeto usa o **Pico SDK** + **LWIP** + **drivers adicionais** (como para o OLED e WS2812).

Exemplo de comando CMake:

```bash
mkdir build
cd build
cmake ..
make
```

Certifique-se de incluir todas as bibliotecas necessárias.

## Como carregar

Após compilar, coloque o Pico em modo BOOTSEL, copie o `.uf2` gerado para a unidade montada.

## Dados publicados via MQTT

| Tópico | Dados |
|---|---|
| `/temperature` | Temperatura em °C |
| `/humidity` | Umidade em % |
| `/temperature/fft` | Frequência dominante + amplitude da temperatura |
| `/humidity/fft` | Frequência dominante + amplitude da umidade |
| `/online` | Status de conexão |