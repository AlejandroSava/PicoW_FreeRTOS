#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "pico/time.h"
#include "FreeRTOS.h"
#include "task.h"

#define WIFI_SSID     "AlexWifi"
#define WIFI_PASSWORD "Alex1234"
#define SERVER_IP     "192.168.0.104"
#define SERVER_PORT   1234

static struct tcp_pcb *client_pcb = NULL;
static absolute_time_t next_send_time;

// Callback when data is acknowledged
err_t tcp_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    printf("Data acknowledged by server\n");
    return ERR_OK;
}

// Callback when TCP connection is established
err_t tcp_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Connection failed: %d\n", err);
        return err;
    }

    printf("TCP connected. Starting data loop...\n");
    next_send_time = make_timeout_time_ms(1000);
    tcp_sent(tpcb, tcp_sent_cb);
    return ERR_OK;
}

void connect_to_server() {
    ip_addr_t remote_ip;
    ip4addr_aton(SERVER_IP, &remote_ip);

    client_pcb = tcp_new();
    if (!client_pcb) {
        printf("Failed to create TCP PCB\n");
        return;
    }

    tcp_connect(client_pcb, &remote_ip, SERVER_PORT, tcp_connected_cb);
}

void tcp_send_continuous(struct tcp_pcb *tpcb) {
    if (!tpcb) return;

    if (absolute_time_diff_us(get_absolute_time(), next_send_time) <= 0) {
        const char *msg = "Hello from Pico W (loop TCP)\n";
        err_t err = tcp_write(tpcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
        for (int i = 0; i < 10; i++){

            char str[20];
            itoa(i, str, 10); // base 10 for decimal
            printf("Converted string: %s\n", str);

            err_t err = tcp_write(tpcb, str, strlen(str), TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) {
                tcp_output(tpcb);
                printf("Sent: %s", str);
            } else {
                printf("Send failed with error %d\n", err);
            }
            next_send_time = make_timeout_time_ms(1000);

        }
        
    }
}

// FreeRTOS task
void tcp_sender_task(void *pvParameters) {
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
        printf("Wi-Fi init failed\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Wi-Fi connection failed\n");
        vTaskDelete(NULL);
    }
    printf("Connected to Wi-Fi\n");

    connect_to_server();

    while (1) {
        cyw43_arch_poll();
        tcp_send_continuous(client_pcb);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();
    xTaskCreate(tcp_sender_task, "tcp_sender", 4096, NULL, 1, NULL);
    vTaskStartScheduler();
    while (true) {
        tight_loop_contents();
    }
    return 0;
}
