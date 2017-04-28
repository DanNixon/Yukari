#ifndef __MPU6000_H
#define __MPU6000_H

#define SPI1_PORT GPIOA

#define SPI1_SCK_PIN GPIO5
#define SPI1_SCK_MISO GPIO6
#define SPI1_SCK_MOSI GPIO7
#define SPI1_SCK_NSS GPIO4

#define MPU6000_CS_PIN SPI1_SCK_NSS

#define MPU6000_INT_PORT GPIOC
#define MPU6000_INT_PIN GPIO4

void mpu6000_init(void);

#endif /* __MPU6000_H */
