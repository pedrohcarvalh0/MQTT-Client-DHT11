#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "ws2812.pio.h"
#include "pico/bootrom.h"

// Configurações WiFi e MQTT
#define WIFI_SSID "NOME DA SUA REDE WIFI"
#define WIFI_PASSWORD "SENHA DA SUA REDE WIFI"
#define MQTT_SERVER "ENDEREÇO IP DO BROKER"
#define MQTT_USERNAME "USERNAME DEFINIDO NO BROKER"
#define MQTT_PASSWORD "SENHA DEFINIDA NO BROKER"

// Definições de pinos
#define BUTTON_B        6
#define DHT_PIN         28  // Pino de dados do DHT11
#define BUZZER_PIN      10  // Pino do buzzer
#define RGB_RED_PIN     13  // Pino do LED RGB (vermelho)
#define RGB_GREEN_PIN   11  // Pino do LED RGB (verde)
#define RGB_BLUE_PIN    12  // Pino do LED RGB (azul)
#define WS2812_PIN      7   // Pino da matriz de LEDs WS2812
#define NUM_PIXELS      25  // Número de LEDs na matriz (5x5)

// Configuração do display OLED
#define I2C_PORT        i2c1
#define I2C_SDA         14
#define I2C_SCL         15
#define OLED_ADDR       0x3C

// Configurações DHT11
#define DHT_WAIT_RESPONSE_US  100
#define DHT_WAIT_BIT_LOW_US   200
#define DHT_WAIT_BIT_HIGH_US  200
#define DHT_BIT_THRESHOLD_US  50
#define DHT_READ_INTERVAL_MS  2000

// Configurações FFT
#define FFT_SIZE        64      // Tamanho da FFT (deve ser potência de 2)
#define FFT_LOG2_SIZE   6       // log2(64) = 6
#define SAMPLE_RATE     0.5f    // Taxa de amostragem (0.5 Hz = uma amostra a cada 2 segundos)

// Limiares para os estados de temperatura e umidade
#define TEMP_GOOD_MAX   25.0f  // Temperatura máxima para estado "bom"
#define TEMP_ALERT_MAX  30.0f  // Temperatura máxima para estado "alerta"
#define HUMID_GOOD_MIN  40.0f  // Umidade mínima para estado "bom"
#define HUMID_ALERT_MIN 30.0f  // Umidade mínima para estado "alerta"

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

// Estrutura para números complexos
typedef struct {
    float real;
    float imag;
} complex_t;

// Estrutura para dados DHT11
typedef struct {
    float humidity;
    float temperature_celsius;
    bool valid;
} dht_reading_t;

// Enumeração para os estados do sistema
typedef enum {
    STATE_GOOD,    // Estado bom (verde)
    STATE_ALERT,   // Estado de alerta (amarelo)
    STATE_CRITICAL // Estado crítico (vermelho)
} system_state_t;

// Estrutura para análise FFT
typedef struct {
    float temp_buffer[FFT_SIZE];        // Buffer circular para temperatura
    float humid_buffer[FFT_SIZE];       // Buffer circular para umidade
    complex_t temp_fft[FFT_SIZE];       // Resultado FFT temperatura
    complex_t humid_fft[FFT_SIZE];      // Resultado FFT umidade
    int buffer_index;                   // Índice atual no buffer circular
    int samples_count;                  // Contador de amostras coletadas
    bool fft_ready;                     // Flag indicando se FFT está pronta
    float temp_dominant_freq;           // Frequência dominante temperatura
    float humid_dominant_freq;          // Frequência dominante umidade
    float temp_amplitude;               // Amplitude dominante temperatura
    float humid_amplitude;              // Amplitude dominante umidade
} fft_analysis_t;

// Dados do cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
    dht_reading_t last_dht_reading;
    uint32_t reading_count;
    system_state_t current_state;
    bool mqtt_connected;
    fft_analysis_t fft_data;            // Dados para análise FFT
} MQTT_CLIENT_DATA_T;

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

// Configurações MQTT
#define TEMP_WORKER_TIME_S 2
#define MQTT_KEEP_ALIVE_S 60
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

// Variáveis globais para monitoramento local
PIO pio = pio0;
int sm = 0;
uint32_t last_beep_time = 0;
uint32_t last_read_time = 0;
uint32_t last_fft_time = 0;
ssd1306_t oled;

// Padrões para a matriz de LEDs
static const bool led_pattern_good[NUM_PIXELS] = {
    0,0,1,0,0,
    0,1,1,1,0,
    1,1,1,1,1,
    0,1,1,1,0,
    0,0,1,0,0
};

static const bool led_pattern_alert[NUM_PIXELS] = {
    0,0,1,0,0,
    0,0,0,0,0,
    0,0,1,0,0,
    0,0,1,0,0,
    0,0,1,0,0
};

static const bool led_pattern_critical[NUM_PIXELS] = {
    1,0,0,0,1,
    0,1,0,1,0,
    0,0,1,0,0,
    0,1,0,1,0,
    1,0,0,0,1
};

// Protótipos de funções DHT11
static bool dht_read(uint pin, dht_reading_t *result);
static bool read_dht_bit(uint pin, uint8_t *out_bit);
static bool read_dht_data_raw(uint pin, uint8_t data[5]);

// Protótipos de funções FFT
void fft_init(fft_analysis_t *fft);
void fft_add_sample(fft_analysis_t *fft, float temp, float humid);
void fft_compute(fft_analysis_t *fft);
void fft_radix2(complex_t *data, int n, bool inverse);
void fft_analyze_results(fft_analysis_t *fft);
void fft_publish_results(MQTT_CLIENT_DATA_T *state);

// Protótipos de funções de monitoramento local
void update_system_state(MQTT_CLIENT_DATA_T *state, const dht_reading_t *reading);
void update_rgb_led(system_state_t state);
void update_led_matrix(system_state_t state);
void update_display(const dht_reading_t *reading, system_state_t state, bool mqtt_status, fft_analysis_t *fft);
void update_buzzer(system_state_t state);
void buzzer_init();
void play_sound(int frequency, int duration_ms);
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);
void put_pixel(uint32_t pixel_grb);
void display_matrix_pattern(const bool *pattern, uint8_t r, uint8_t g, uint8_t b);

// Protótipos de funções MQTT
static void pub_request_cb(__unused void *arg, err_t err);
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);
static void publish_dht_data(MQTT_CLIENT_DATA_T *state);
static void sub_request_cb(void *arg, err_t err);
static void unsub_request_cb(void *arg, err_t err);
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
static void dht_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void start_client(MQTT_CLIENT_DATA_T *state);
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

static async_at_time_worker_t dht_worker = { .do_work = dht_worker_fn };

// Handler para botão B (reset)
void gpio_irq_handler(uint gpio, uint32_t events) {
    reset_usb_boot(0, 0);
}

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    INFO_printf("MQTT DHT11 + Monitoring + FFT client starting\n");

    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Inicializa DHT11
    gpio_init(DHT_PIN);
    gpio_set_dir(DHT_PIN, GPIO_IN);
    gpio_pull_up(DHT_PIN);

    // Inicializa o buzzer
    buzzer_init();

    // Inicializa os pinos do LED RGB
    gpio_init(RGB_RED_PIN);
    gpio_init(RGB_GREEN_PIN);
    gpio_init(RGB_BLUE_PIN);
    gpio_set_dir(RGB_RED_PIN, GPIO_OUT);
    gpio_set_dir(RGB_GREEN_PIN, GPIO_OUT);
    gpio_set_dir(RGB_BLUE_PIN, GPIO_OUT);

    // Inicializa o I2C para o display OLED
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o display OLED
    ssd1306_init(&oled, WIDTH, HEIGHT, false, OLED_ADDR, I2C_PORT);
    ssd1306_config(&oled);
    ssd1306_fill(&oled, false);
    ssd1306_draw_string(&oled, "Iniciando...", 16, 28);
    ssd1306_send_data(&oled);

    // Inicializa o PIO para a matriz de LEDs WS2812
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);

    // Aguarda um pouco para o sensor estabilizar
    sleep_ms(2000);
    INFO_printf("DHT11 initialized on GPIO %d\n", DHT_PIN);

    static MQTT_CLIENT_DATA_T state;
    // Inicializa contadores
    state.last_dht_reading.valid = false;
    state.reading_count = 0;
    state.current_state = STATE_GOOD;
    state.mqtt_connected = false;
    
    // Inicializa FFT
    fft_init(&state.fft_data);

    if (cyw43_arch_init()) {
        panic("Failed to initialize CYW43");
    }

    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;

    // Exibe status de conexão WiFi no display
    ssd1306_fill(&oled, false);
    ssd1306_draw_string(&oled, "Conectando WiFi...", 8, 28);
    ssd1306_send_data(&oled);

    cyw43_arch_enable_sta_mode();
    INFO_printf("Connecting to WiFi: %s\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }
    INFO_printf("Connected to Wifi\n");

    // Exibe status de conexão MQTT no display
    ssd1306_fill(&oled, false);
    ssd1306_draw_string(&oled, "Conectando MQTT...", 8, 28);
    ssd1306_send_data(&oled);

    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) {
        panic("dns request failed");
    }

    // Inicializa variáveis de tempo
    last_read_time = time_us_32() / 1000;
    last_beep_time = last_read_time;
    last_fft_time = last_read_time;

    INFO_printf("Entering main loop\n");
    
    // Loop principal integrado
    while (true) {
        uint32_t now_ms = time_us_32() / 1000;

        // Processa eventos MQTT
        cyw43_arch_poll();

        // Verifica se é hora de ler o sensor (independente do MQTT)
        if ((now_ms - last_read_time) >= DHT_READ_INTERVAL_MS) {
            last_read_time = now_ms;

            dht_reading_t current_reading;
            if (dht_read(DHT_PIN, &current_reading)) {
                // Adiciona amostra para análise FFT
                fft_add_sample(&state.fft_data, current_reading.temperature_celsius, current_reading.humidity);
                
                // Atualiza o estado do sistema com base na leitura
                update_system_state(&state, &current_reading);
                
                // Atualiza os dispositivos de saída locais
                update_rgb_led(state.current_state);
                update_led_matrix(state.current_state);
                update_display(&current_reading, state.current_state, state.mqtt_connected, &state.fft_data);
                
                // Exibe informações no console
                printf("[ %lums ] Temperatura: %.1f °C   Umidade: %.1f %%   Estado: %d   MQTT: %s   FFT: %s\n",
                       (unsigned long)now_ms,
                       current_reading.temperature_celsius,
                       current_reading.humidity,
                       state.current_state,
                       state.mqtt_connected ? "OK" : "DESCONECTADO",
                       state.fft_data.fft_ready ? "PRONTA" : "COLETANDO");
                
                // Atualiza a leitura no estado para o MQTT
                state.last_dht_reading = current_reading;
            } else {
                printf("[ %lums ] Falha na leitura do DHT11\n", (unsigned long)now_ms);
                
                // Exibe mensagem de erro no display
                ssd1306_fill(&oled, false);
                ssd1306_draw_string(&oled, "Erro de leitura", 8, 28);
                ssd1306_send_data(&oled);
            }
        }

        // Verifica se é hora de calcular FFT (a cada 30 segundos)
        if (state.fft_data.samples_count >= FFT_SIZE && (now_ms - last_fft_time) >= 30000) {
            last_fft_time = now_ms;
            INFO_printf("Computing FFT analysis...\n");
            fft_compute(&state.fft_data);
            fft_analyze_results(&state.fft_data);
            
            // Publica resultados FFT via MQTT se conectado
            if (state.mqtt_connected) {
                fft_publish_results(&state);
            }
        }

        // Atualiza o buzzer conforme o estado atual
        update_buzzer(state.current_state);

        // Verifica se deve sair do loop (quando MQTT desconecta)
        if (state.connect_done && !mqtt_client_is_connected(state.mqtt_client_inst)) {
            state.mqtt_connected = false;
            // Continua funcionando localmente mesmo sem MQTT
        }

        // Permite que o sistema processe outras tarefas
        tight_loop_contents();
        sleep_ms(100); // Pequeno delay para não sobrecarregar o sistema
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}

// ========== IMPLEMENTAÇÃO DAS FUNÇÕES FFT ==========

void fft_init(fft_analysis_t *fft) {
    memset(fft, 0, sizeof(fft_analysis_t));
    fft->buffer_index = 0;
    fft->samples_count = 0;
    fft->fft_ready = false;
    INFO_printf("FFT initialized - collecting %d samples\n", FFT_SIZE);
}

void fft_add_sample(fft_analysis_t *fft, float temp, float humid) {
    // Adiciona amostras ao buffer circular
    fft->temp_buffer[fft->buffer_index] = temp;
    fft->humid_buffer[fft->buffer_index] = humid;
    
    fft->buffer_index = (fft->buffer_index + 1) % FFT_SIZE;
    
    if (fft->samples_count < FFT_SIZE) {
        fft->samples_count++;
        INFO_printf("FFT sample %d/%d collected\n", fft->samples_count, FFT_SIZE);
    }
}

void fft_compute(fft_analysis_t *fft) {
    if (fft->samples_count < FFT_SIZE) {
        INFO_printf("Not enough samples for FFT\n");
        return;
    }
    
    // Prepara dados para FFT (temperatura)
    for (int i = 0; i < FFT_SIZE; i++) {
        int idx = (fft->buffer_index + i) % FFT_SIZE;
        fft->temp_fft[i].real = fft->temp_buffer[idx];
        fft->temp_fft[i].imag = 0.0f;
    }
    
    // Calcula FFT para temperatura
    fft_radix2(fft->temp_fft, FFT_SIZE, false);
    
    // Prepara dados para FFT (umidade)
    for (int i = 0; i < FFT_SIZE; i++) {
        int idx = (fft->buffer_index + i) % FFT_SIZE;
        fft->humid_fft[i].real = fft->humid_buffer[idx];
        fft->humid_fft[i].imag = 0.0f;
    }
    
    // Calcula FFT para umidade
    fft_radix2(fft->humid_fft, FFT_SIZE, false);
    
    fft->fft_ready = true;
    INFO_printf("FFT computation completed\n");
}

void fft_radix2(complex_t *data, int n, bool inverse) {
    // Implementação FFT Radix-2 Decimation-in-Time
    int j = 0;
    
    // Bit-reversal permutation
    for (int i = 1; i < n; i++) {
        int bit = n >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        
        if (i < j) {
            complex_t temp = data[i];
            data[i] = data[j];
            data[j] = temp;
        }
    }
    
    // FFT computation
    for (int len = 2; len <= n; len <<= 1) {
        float angle = (inverse ? 2.0f : -2.0f) * M_PI / len;
        complex_t wlen = {cosf(angle), sinf(angle)};
        
        for (int i = 0; i < n; i += len) {
            complex_t w = {1.0f, 0.0f};
            
            for (int j = 0; j < len / 2; j++) {
                complex_t u = data[i + j];
                complex_t v = {
                    data[i + j + len/2].real * w.real - data[i + j + len/2].imag * w.imag,
                    data[i + j + len/2].real * w.imag + data[i + j + len/2].imag * w.real
                };
                
                data[i + j] = (complex_t){u.real + v.real, u.imag + v.imag};
                data[i + j + len/2] = (complex_t){u.real - v.real, u.imag - v.imag};
                
                float w_temp = w.real * wlen.real - w.imag * wlen.imag;
                w.imag = w.real * wlen.imag + w.imag * wlen.real;
                w.real = w_temp;
            }
        }
    }
    
    if (inverse) {
        for (int i = 0; i < n; i++) {
            data[i].real /= n;
            data[i].imag /= n;
        }
    }
}

void fft_analyze_results(fft_analysis_t *fft) {
    if (!fft->fft_ready) return;
    
    float max_temp_magnitude = 0.0f;
    float max_humid_magnitude = 0.0f;
    int max_temp_idx = 0;
    int max_humid_idx = 0;
    
    // Analisa apenas a primeira metade (frequências positivas)
    for (int i = 1; i < FFT_SIZE / 2; i++) {
        // Magnitude para temperatura
        float temp_magnitude = sqrtf(fft->temp_fft[i].real * fft->temp_fft[i].real + 
                                   fft->temp_fft[i].imag * fft->temp_fft[i].imag);
        
        if (temp_magnitude > max_temp_magnitude) {
            max_temp_magnitude = temp_magnitude;
            max_temp_idx = i;
        }
        
        // Magnitude para umidade
        float humid_magnitude = sqrtf(fft->humid_fft[i].real * fft->humid_fft[i].real + 
                                    fft->humid_fft[i].imag * fft->humid_fft[i].imag);
        
        if (humid_magnitude > max_humid_magnitude) {
            max_humid_magnitude = humid_magnitude;
            max_humid_idx = i;
        }
    }
    
    // Calcula frequências dominantes
    fft->temp_dominant_freq = (float)max_temp_idx * SAMPLE_RATE / FFT_SIZE;
    fft->humid_dominant_freq = (float)max_humid_idx * SAMPLE_RATE / FFT_SIZE;
    fft->temp_amplitude = max_temp_magnitude;
    fft->humid_amplitude = max_humid_magnitude;
    
    INFO_printf("FFT Analysis Results:\n");
    INFO_printf("Temperature - Dominant Freq: %.4f Hz, Amplitude: %.2f\n", 
                fft->temp_dominant_freq, fft->temp_amplitude);
    INFO_printf("Humidity - Dominant Freq: %.4f Hz, Amplitude: %.2f\n", 
                fft->humid_dominant_freq, fft->humid_amplitude);
}

void fft_publish_results(MQTT_CLIENT_DATA_T *state) {
    if (!state->fft_data.fft_ready || !state->mqtt_connected) return;
    
    char fft_data[256];
    
    // Publica análise FFT da temperatura
    snprintf(fft_data, sizeof(fft_data), 
             "{\"freq\":%.4f,\"amplitude\":%.2f,\"period_min\":%.1f}", 
             state->fft_data.temp_dominant_freq,
             state->fft_data.temp_amplitude,
             state->fft_data.temp_dominant_freq > 0 ? (1.0f / state->fft_data.temp_dominant_freq / 60.0f) : 0);
    
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/temperature/fft"), 
                fft_data, strlen(fft_data), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    // Publica análise FFT da umidade
    snprintf(fft_data, sizeof(fft_data), 
             "{\"freq\":%.4f,\"amplitude\":%.2f,\"period_min\":%.1f}", 
             state->fft_data.humid_dominant_freq,
             state->fft_data.humid_amplitude,
             state->fft_data.humid_dominant_freq > 0 ? (1.0f / state->fft_data.humid_dominant_freq / 60.0f) : 0);
    
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/humidity/fft"), 
                fft_data, strlen(fft_data), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    INFO_printf("FFT results published via MQTT\n");
}

// ========== IMPLEMENTAÇÃO DAS FUNÇÕES DHT11 ==========

static bool read_dht_bit(uint pin, uint8_t *out_bit) {
    uint32_t start_time;

    start_time = time_us_32();
    while (gpio_get(pin) == 0) {
        if (time_us_32() - start_time > DHT_WAIT_BIT_LOW_US) {
            return false;
        }
    }

    start_time = time_us_32();
    while (gpio_get(pin) == 1) {
        if (time_us_32() - start_time > DHT_WAIT_BIT_HIGH_US) {
            return false;
        }
    }
    uint32_t high_duration = time_us_32() - start_time;

    *out_bit = (high_duration > DHT_BIT_THRESHOLD_US) ? 1 : 0;
    return true;
}

static bool read_dht_data_raw(uint pin, uint8_t data[5]) {
    for (int byte_idx = 0; byte_idx < 5; byte_idx++) {
        data[byte_idx] = 0;
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) {
            uint8_t bit = 0;
            if (!read_dht_bit(pin, &bit)) {
                return false;
            }
            data[byte_idx] = (uint8_t)((data[byte_idx] << 1) | bit);
        }
    }
    return true;
}

static bool dht_read(uint pin, dht_reading_t *result) {
    uint8_t raw[5] = {0};

    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
    sleep_ms(20);
    gpio_put(pin, 1);
    sleep_us(30);
    gpio_set_dir(pin, GPIO_IN);

    uint32_t t0 = time_us_32();
    while (gpio_get(pin) == 1) {
        if (time_us_32() - t0 > DHT_WAIT_RESPONSE_US) {
            return false;
        }
    }
    t0 = time_us_32();
    while (gpio_get(pin) == 0) {
        if (time_us_32() - t0 > DHT_WAIT_RESPONSE_US) {
            return false;
        }
    }
    t0 = time_us_32();
    while (gpio_get(pin) == 1) {
        if (time_us_32() - t0 > DHT_WAIT_RESPONSE_US) {
            return false;
        }
    }

    if (!read_dht_data_raw(pin, raw)) {
        return false;
    }

    uint8_t checksum = (uint8_t)((raw[0] + raw[1] + raw[2] + raw[3]) & 0xFF);
    if (checksum != raw[4]) {
        return false;
    }

    result->humidity = (float)raw[0] + ((float)raw[1] / 10.0f);
    result->temperature_celsius = (float)(raw[2] & 0x7F) + ((float)raw[3] / 10.0f);
    if (raw[2] & 0x80) {
        result->temperature_celsius *= -1.0f;
    }
    result->valid = true;

    return true;
}

// ========== IMPLEMENTAÇÃO DAS FUNÇÕES DE MONITORAMENTO LOCAL ==========

void update_system_state(MQTT_CLIENT_DATA_T *mqtt_state, const dht_reading_t *reading) {
    if (reading->temperature_celsius > TEMP_ALERT_MAX) {
        mqtt_state->current_state = STATE_CRITICAL;
    } else if (reading->temperature_celsius > TEMP_GOOD_MAX) {
        mqtt_state->current_state = STATE_ALERT;
    } else if (reading->humidity < HUMID_ALERT_MIN) {
        mqtt_state->current_state = STATE_CRITICAL;
    } else if (reading->humidity < HUMID_GOOD_MIN) {
        mqtt_state->current_state = STATE_ALERT;
    } else {
        mqtt_state->current_state = STATE_GOOD;
    }
}

void update_rgb_led(system_state_t state) {
    switch (state) {
        case STATE_GOOD:
            gpio_put(RGB_RED_PIN, 0);
            gpio_put(RGB_GREEN_PIN, 1);
            gpio_put(RGB_BLUE_PIN, 0);
            break;
        case STATE_ALERT:
            gpio_put(RGB_RED_PIN, 1);
            gpio_put(RGB_GREEN_PIN, 1);
            gpio_put(RGB_BLUE_PIN, 0);
            break;
        case STATE_CRITICAL:
            gpio_put(RGB_RED_PIN, 1);
            gpio_put(RGB_GREEN_PIN, 0);
            gpio_put(RGB_BLUE_PIN, 0);
            break;
    }
}

void update_led_matrix(system_state_t state) {
    switch (state) {
        case STATE_GOOD:
            display_matrix_pattern(led_pattern_good, 0, 50, 0);
            break;
        case STATE_ALERT:
            display_matrix_pattern(led_pattern_alert, 50, 50, 0);
            break;
        case STATE_CRITICAL:
            display_matrix_pattern(led_pattern_critical, 50, 0, 0);
            break;
    }
}

void update_display(const dht_reading_t *reading, system_state_t state, bool mqtt_status, fft_analysis_t *fft) {
    char temp_str[20];
    char humid_str[20];
    char status_str[20];
    char mqtt_str[20];
    char fft_str[20];
    
    sprintf(temp_str, "T:%.1fC", reading->temperature_celsius);
    sprintf(humid_str, "H:%.1f%%", reading->humidity);
    sprintf(mqtt_str, "MQTT:%s", mqtt_status ? "OK" : "OFF");
    
    switch (state) {
        case STATE_GOOD:
            sprintf(status_str, "Estado: BOM");
            break;
        case STATE_ALERT:
            sprintf(status_str, "Estado: ALERTA");
            break;
        case STATE_CRITICAL:
            sprintf(status_str, "Estado: CRITICO");
            break;
    }
    
    if (fft->fft_ready) {
        sprintf(fft_str, "FFT: %.3fHz", fft->temp_dominant_freq);
    } else {
        sprintf(fft_str, "FFT: %d/%d", fft->samples_count, FFT_SIZE);
    }
    
    ssd1306_fill(&oled, false);
    ssd1306_rect(&oled, 0, 0, WIDTH, HEIGHT, true, false);
    
    ssd1306_draw_string(&oled, "MONITOR+FFT", 16, 2);
    ssd1306_line(&oled, 0, 12, WIDTH-1, 12, true);
    
    ssd1306_draw_string(&oled, temp_str, 8, 16);
    ssd1306_draw_string(&oled, humid_str, 70, 16);
    ssd1306_draw_string(&oled, status_str, 8, 26);
    ssd1306_draw_string(&oled, mqtt_str, 8, 36);
    ssd1306_draw_string(&oled, fft_str, 8, 46);
    
    ssd1306_send_data(&oled);
}

void update_buzzer(system_state_t state) {
    uint32_t now_ms = time_us_32() / 1000;
    
    switch (state) {
        case STATE_GOOD:
            if (now_ms - last_beep_time >= 10000) {
                last_beep_time = now_ms;
                play_sound(1000, 100);
            }
            break;
            
        case STATE_ALERT:
            if (now_ms - last_beep_time >= 5000) {
                last_beep_time = now_ms;
                play_sound(1500, 100);
            }
            break;
            
        case STATE_CRITICAL:
            if (now_ms - last_beep_time >= 5000) {
                last_beep_time = now_ms;
                play_sound(2000, 100);
                sleep_ms(200);
                play_sound(2000, 100);
            }
            break;
    }
}

void buzzer_init() {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

void play_sound(int frequency, int duration_ms) {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    if (frequency <= 0) {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        sleep_ms(duration_ms);
        return;
    }
    
    float divider = 20.0f;
    pwm_set_clkdiv(slice_num, divider);
    uint16_t wrap = (125000000 / (frequency * divider)) - 1;
    pwm_set_wrap(slice_num, wrap);
    pwm_set_gpio_level(BUZZER_PIN, wrap / 2);
    sleep_ms(duration_ms);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

void display_matrix_pattern(const bool *pattern, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color = urgb_u32(r, g, b);
    
    for (int i = 0; i < NUM_PIXELS; i++) {
        int row = i / 5;
        int col;
        
        if (row % 2 == 0) {
            col = i % 5;
        } else {
            col = 4 - (i % 5);
        }
        
        int pattern_idx = row * 5 + col;
        put_pixel(pattern[pattern_idx] ? color : 0);
    }
}

// ========== IMPLEMENTAÇÃO DAS FUNÇÕES MQTT ==========

static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d\n", err);
    }
}

static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    const char* message = on ? "On" : "Off";
    if (on)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

static void publish_dht_data(MQTT_CLIENT_DATA_T *state) {
    if (!state->last_dht_reading.valid) {
        INFO_printf("No valid DHT reading to publish\n");
        return;
    }

    INFO_printf("Publishing DHT data via MQTT (attempt #%lu)\n", ++state->reading_count);
    
    // Publica temperatura
    char temp_str[16];
    snprintf(temp_str, sizeof(temp_str), "%.1f", state->last_dht_reading.temperature_celsius);
    INFO_printf("Publishing temperature: %s°C to topic %s\n", temp_str, full_topic(state, "/temperature"));
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/temperature"), temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    // Publica umidade
    char hum_str[16];
    snprintf(hum_str, sizeof(hum_str), "%.1f", state->last_dht_reading.humidity);
    INFO_printf("Publishing humidity: %s%% to topic %s\n", hum_str, full_topic(state, "/humidity"));
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/humidity"), hum_str, strlen(hum_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}

static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/led") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "/print") == 0) {
        INFO_printf("%.*s\n", len, data);
    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true;
        sub_unsub_topics(state, false);
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void dht_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    
    INFO_printf("DHT MQTT worker function called\n");
    
    if (mqtt_client_is_connected(state->mqtt_client_inst)) {
        publish_dht_data(state);
    } else {
        INFO_printf("MQTT not connected, skipping MQTT publish\n");
    }
    
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
    INFO_printf("Next MQTT publish scheduled in %d seconds\n", TEMP_WORKER_TIME_S);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        INFO_printf("MQTT connection accepted\n");
        state->connect_done = true;
        state->mqtt_connected = true;
        sub_unsub_topics(state, true);

        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        INFO_printf("Starting DHT11 MQTT worker\n");
        dht_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &dht_worker, 1000);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        INFO_printf("MQTT disconnected\n");
        state->mqtt_connected = false;
        if (!state->connect_done) {
            panic("Failed to connect to mqtt server");
        }
    } else {
        ERROR_printf("Unexpected MQTT status: %d\n", status);
        state->mqtt_connected = false;
        panic("Unexpected status");
    }
}

static void start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}