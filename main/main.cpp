#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "protocol_examples_common.h"

extern "C" {
	void app_main(void);
}

extern void task_initI2C(void*);
extern void task_mpu6050_init(void*);
extern void task_udp_client(void*);
extern void task_print_readings(void*);
extern void task_read_sensor(void*);
extern void task_net_init();

void app_main(void)
{
    ESP_LOGD("app_main", "Program Boots.\n");
    xTaskCreate(task_initI2C, "task_initI2C", 4096, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(task_mpu6050_init, "mpu6050_init", 4096, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);

    task_net_init();
    xTaskCreate(task_read_sensor, "task_read_sensor", 4096, NULL, 5, NULL);
    xTaskCreate(task_print_readings, "task_print_readings", 4096, NULL, 4, NULL);
    xTaskCreate(task_udp_client, "task_udp_client", 4096, NULL, 5, NULL);

    // xTaskCreate(task_mpu6050_init2, "task_mpu6050_init2", 4096, NULL, 5, NULL);
    // vTaskDelay(500 / portTICK_PERIOD_MS);
}
