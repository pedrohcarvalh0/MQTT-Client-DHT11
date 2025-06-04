#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"

#include <math.h>

// Configurações WiFi e MQTT
#define WIFI_SSID "KASATECH CARVALHO"
#define WIFI_PASSWORD "Ph01felix!"
#define MQTT_SERVER "192.168.0.101"
#define MQTT_USERNAME "admin"
#define MQTT_PASSWORD "123456"

#define BUTTON_B 6 // Bottão para o bootsel

// Configurações DHT11
#define DHT_PIN   28 // GPIO conectado ao pino de dados do DHT11
#define DHT_WAIT_RESPONSE_US  100
#define DHT_WAIT_BIT_LOW_US   200
#define DHT_WAIT_BIT_HIGH_US  200
#define DHT_BIT_THRESHOLD_US  50
#define DHT_READ_INTERVAL_MS  2000

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

// Estrutura para dados DHT11
typedef struct {
    float humidity;
    float temperature_celsius;
    bool valid;
} dht_reading_t;

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
#define TEMP_WORKER_TIME_S 5
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

// ========== FUNÇÕES DHT11 ==========

static bool read_dht_bit(uint pin, uint8_t *out_bit) {
    uint32_t start_time;

    // Aguarda fim do pulso baixo
    start_time = time_us_32();
    while (gpio_get(pin) == 0) {
        if (time_us_32() - start_time > DHT_WAIT_BIT_LOW_US) {
            return false;
        }
    }

    // Mede duração do pulso alto
    start_time = time_us_32();
    while (gpio_get(pin) == 1) {
        if (time_us_32() - start_time > DHT_WAIT_BIT_HIGH_US) {
            return false;
        }
    }
    uint32_t high_duration = time_us_32() - start_time;

    // Classifica bit
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

    // Init signal
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
    sleep_ms(20);
    gpio_put(pin, 1);
    sleep_us(30);
    gpio_set_dir(pin, GPIO_IN);

    // Aguarda resposta do sensor
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

    // Lê 40 bits de dados
    if (!read_dht_data_raw(pin, raw)) {
        return false;
    }

    // Verifica checksum
    uint8_t checksum = (uint8_t)((raw[0] + raw[1] + raw[2] + raw[3]) & 0xFF);
    if (checksum != raw[4]) {
        return false;
    }

    // Converte para float
    result->humidity = (float)raw[0] + ((float)raw[1] / 10.0f);
    result->temperature_celsius = (float)(raw[2] & 0x7F) + ((float)raw[3] / 10.0f);
    if (raw[2] & 0x80) {
        result->temperature_celsius *= -1.0f;
    }
    result->valid = true;

    return true;
}

// ========== FUNÇÕES MQTT ==========

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
    dht_reading_t current_reading;
    
    INFO_printf("Reading DHT11 sensor... (attempt #%lu)\n", ++state->reading_count);
    
    if (dht_read(DHT_PIN, &current_reading)) {
        INFO_printf("DHT11 read successful: T=%.1f°C, H=%.1f%%\n", 
                   current_reading.temperature_celsius, current_reading.humidity);
        
        // Sempre publica temperatura (removida a lógica de comparação para debug)
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.1f", current_reading.temperature_celsius);
        INFO_printf("Publishing temperature: %s°C to topic %s\n", temp_str, full_topic(state, "/temperature"));
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/temperature"), temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
        
        // Sempre publica umidade (removida a lógica de comparação para debug)
        char hum_str[16];
        snprintf(hum_str, sizeof(hum_str), "%.1f", current_reading.humidity);
        INFO_printf("Publishing humidity: %s%% to topic %s\n", hum_str, full_topic(state, "/humidity"));
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/humidity"), hum_str, strlen(hum_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
        
        // Atualiza última leitura
        state->last_dht_reading = current_reading;
    } else {
        ERROR_printf("Failed to read DHT11 sensor (attempt #%lu)\n", state->reading_count);
    }
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
    
    INFO_printf("DHT worker function called - scheduling next reading\n");
    
    // Verifica se ainda está conectado ao MQTT
    if (mqtt_client_is_connected(state->mqtt_client_inst)) {
        publish_dht_data(state);
    } else {
        INFO_printf("MQTT not connected, skipping DHT reading\n");
    }
    
    // Reagenda para próxima execução
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
    INFO_printf("Next DHT reading scheduled in %d seconds\n", TEMP_WORKER_TIME_S);
}

static async_at_time_worker_t dht_worker = { .do_work = dht_worker_fn };

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        INFO_printf("MQTT connection accepted\n");
        state->connect_done = true;
        sub_unsub_topics(state, true);

        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publica dados DHT11 imediatamente e depois a cada 10 segundos
        INFO_printf("Starting DHT11 worker\n");
        dht_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &dht_worker, 1000); // Primeira leitura em 1 segundo
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        INFO_printf("MQTT disconnected\n");
        if (!state->connect_done) {
            panic("Failed to connect to mqtt server");
        }
    } else {
        ERROR_printf("Unexpected MQTT status: %d\n", status);
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

void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();
    INFO_printf("MQTT DHT11 client starting\n");

    // Inicializa DHT11
    gpio_init(DHT_PIN);
    gpio_set_dir(DHT_PIN, GPIO_IN);
    gpio_pull_up(DHT_PIN);
    
    // Aguarda um pouco para o sensor estabilizar
    sleep_ms(2000);
    INFO_printf("DHT11 initialized on GPIO %d\n", DHT_PIN);

    static MQTT_CLIENT_DATA_T state;
    // Inicializa contadores
    state.last_dht_reading.valid = false;
    state.reading_count = 0;

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

    cyw43_arch_enable_sta_mode();
    INFO_printf("Connecting to WiFi: %s\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }
    INFO_printf("Connected to Wifi\n");

    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) {
        panic("dns request failed");
    }

    INFO_printf("Entering main loop\n");
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1000)); // Ainda mais responsivo
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}