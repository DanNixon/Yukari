#ifndef __MPU6000_H
#define __MPU6000_H

#define MPU6000_CS_PORT GPIOA
#define MPU6000_CS_PIN GPIO4

#define MPU6000_INT_PORT GPIOC
#define MPU6000_INT_PIN GPIO4

#define MPU6000_SPI_INSTANCE SPI1

void mpu6000_init(void);

#endif /* __MPU6000_H */
