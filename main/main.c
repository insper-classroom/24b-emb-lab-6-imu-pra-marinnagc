#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>


#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
#define SAMPLE_PERIOD (0.02f) // Aumentado para 20ms de intervalo de leitura
#define DEAD_ZONE 5.0f        // Janela morta para ignorar pequenas mudanças
#define SCALE_MAX 255         // Valor máximo da nova escala
#define SCALE_MIN -255        // Valor mínimo da nova escala

void send_uart_data(uint8_t ref, int valor) {
    uint8_t valor1 = (valor >> 8) & 0xFF; 
    uint8_t valor2 = valor & 0xFF;        

    uart_putc_raw(UART_ID, ref);
    uart_putc_raw(UART_ID, valor1);
    uart_putc_raw(UART_ID, valor2);
    uart_putc_raw(UART_ID, 0xFF); 
}
int scale_value(float value, float max_value) {
    // Normalizar o valor para a faixa de -1.0 a +1.0
    float normalized = value / max_value;

    // Reescalonar para a nova faixa (-255 a +255)
    int scaled_value = (int)(normalized * SCALE_MAX);

    // Aplicar a zona morta
    if (scaled_value > -DEAD_ZONE && scaled_value < DEAD_ZONE) {
        return 0; // Valor dentro da zona morta é ignorado
    }
    return scaled_value; // Retornar o valor escalonado fora da zona morta
}
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    float last_roll = 0.0f;   // Variável para armazenar o valor anterior de Roll

    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        //printf("%08d", acceleration[0], acceleration[1], acceleration[2]);
        // printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        // Conversão para fusão de dados
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };


        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        int scaled_roll = scale_value(euler.angle.roll, 2047.0f);  // Supondo que o valor máximo seja +/-2047
        int scaled_yaw = scale_value(euler.angle.yaw, 2047.0f);    // Supondo que o valor máximo seja +/-2047

        //printf("z %0.1f x %0.1f y %0.1f\n", accelerometer.axis.z, accelerometer.axis.x, accelerometer.axis.y);
        //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

        send_uart_data(0, scaled_yaw);
        send_uart_data(1, scaled_roll);

        // Verificar se o movimento de aceleração no eixo Z é significativo para detectar um click
        if (fabs(accelerometer.axis.z) > 1.5f) {  // Ajuste o valor do limiar conforme necessário
            //printf("Mouse Click Detected by Acceleration on Z Axis!\n");
            send_uart_data(2, 1);  // Envia o comando para o "mouse click"
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }


}

int main() {
    stdio_init_all();
    //printf("Start MPU6050 Task with Button\n");

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
