#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <time.h>

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "I2Cdev.h"
#include "protocol_examples_common.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#define PIN_SDA 21
#define PIN_CLK 22

#define HOST_IP_ADDR "Your Server's IP Address"
#define DEST_PORT Your_Port_Number

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_TEMP_OUT_H      0x43
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

/*
 * The following registers contain the primary data we are interested in
 * 0x3B MPU6050_ACCEL_XOUT_H
 * 0x3C MPU6050_ACCEL_XOUT_L
 * 0x3D MPU6050_ACCEL_YOUT_H
 * 0x3E MPU6050_ACCEL_YOUT_L
 * 0x3F MPU6050_ACCEL_ZOUT_H
 * 0x40 MPU6050_ACCEL_ZOUT_L
 * 0x41 MPU6050_TEMP_OUT_H
 * 0x42 MPU6050_TEMP_OUT_L
 * 0x43 MPU6050_GYRO_XOUT_H
 * 0x44 MPU6050_GYRO_XOUT_L
 * 0x45 MPU6050_GYRO_YOUT_H
 * 0x46 MPU6050_GYRO_YOUT_L
 * 0x47 MPU6050_GYRO_ZOUT_H
 * 0x48 MPU6050_GYRO_ZOUT_L
 */

static const char tag[] = "mpu6050";

typedef struct axis {
    /* In degrees/second */
    float rotation = 0.0F;

    /* In meters/second^2 */
    float accel = 0.0F;
} axis;

axis x_read;
axis y_read;
axis z_read;
float tempr_read;

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

using namespace std;

void task_mpu6050_init(void *ignore) {
	printf("mpu6050 initialization\n");

    I2Cdev::writeByte(I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);
    I2Cdev::writeBits(I2C_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
    I2Cdev::writeBits(I2C_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);

    printf("initialized\n");
	vTaskDelete(NULL);
}

void task_initI2C(void *ignore) {
    ESP_LOGD("task_initI2C", "I2C initialization started.");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGD("task_initI2C", "I2C initialization completed.");
	vTaskDelete(NULL);
}

void task_read_sensor(void*) {
    ESP_LOGD("task_read_sensor", "mpu6050 collecting data\n");
    i2c_cmd_handle_t cmd;
    uint8_t data[14];

    while(1) {
        // Tell the MPU6050 to position the internal register pointer to register
        // MPU6050_ACCEL_XOUT_H.
        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS));
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data,    (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+1,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+2,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+3,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+4,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+5,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+6,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+7,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+8,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+9,  (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+10, (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+11, (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+12, (i2c_ack_type_t) 0));
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+13, (i2c_ack_type_t) 1));

        // i2c_master_read(cmd, data, sizeof(112), (i2c_ack_type_t) 1);
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS));
        i2c_cmd_link_delete(cmd);

        x_read.accel = ((int16_t) ((data[0]  << 8) | data[1])) / 16384.0;
        y_read.accel = ((int16_t) ((data[2]  << 8) | data[3])) / 16384.0;
        z_read.accel = ((int16_t) ((data[4]  << 8) | data[5])) / 16384.0;
        tempr_read   = ((int16_t) ((data[6]  << 8) | data[7])) / 340.0 + 36.53;
        x_read.rotation  = ((int16_t) ((data[8]  << 8) | data[9])) / 131.0;
        y_read.rotation  = ((int16_t) ((data[10] << 8) | data[11]))/ 131.0;
        z_read.rotation  = ((int16_t) ((data[12] << 8) | data[13]))/ 131.0;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void task_net_init() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
}

void task_udp_client(void *pvParameters) {
    char buffer[128];
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(DEST_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(tag, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(tag, "Socket created, sending to %s:%d", HOST_IP_ADDR, DEST_PORT);

        struct timeval time_out;
        time_out.tv_sec = 0;
        time_out.tv_usec = 120000;
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &time_out, sizeof(time_out)) < 0) {
            perror("Error Setting timeout.");
        }

        int i = 0;
        struct timeval tv;
        struct timezone tz;
        struct tm *today;

        while (1) {
            gettimeofday(&tv,&tz);
            today = localtime(&tv.tv_sec);

            sprintf(buffer, "%02d:%02d:%02d.%ld,%d,%f,%f,%f,%f,%f,%f,%f", 
                    today->tm_hour,
                    today->tm_min,
                    today->tm_sec,
                    tv.tv_usec,
                    i,
                    x_read.accel, 
                    x_read.rotation, 
                    y_read.accel, 
                    y_read.rotation, 
                    z_read.accel, 
                    z_read.rotation, 
                    tempr_read);

            int err = sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(tag, "Error occurred during sending: errno %d", errno);
                break;
            }

            else {
                ESP_LOGD(tag, "Message sent %d", i);
                i++;
            }
            
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(tag, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void task_print_readings(void*) {
    while(1) {
        std::cout << "x -> accel: " << setw(12) << left << x_read.accel << " , gyro: " << setw(12) << left << x_read.rotation
                  << " / y -> accel: " << setw(12) << left << y_read.accel << " , gyro: " << setw(12) << left << y_read.rotation
                  << " / z -> accel: " << setw(12) << left << z_read.accel << " , gyro: " << setw(12) << left << z_read.rotation
                  << endl;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
