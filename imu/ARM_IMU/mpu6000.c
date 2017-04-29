#include "mpu6000.h"

#include <stdint.h>
#include <stdio.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/stm32/f4/spi.h>

#include "clock.h"

static uint32_t spi_read_mode_fault(uint32_t spi)
{
  return SPI_SR(spi) & SPI_SR_MODF;
}

static void spi_clear_mode_fault(uint32_t spi)
{
  if (spi_read_mode_fault(spi))
  {
    SPI_CR1(spi) = SPI_CR1(spi);
  }
}

static void spi_init(void)
{
  printf("SPI init\n");

  /* Chip select pin */
  gpio_mode_setup(SPI1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MPU6000_CS_PIN);
  gpio_set(SPI1_PORT, MPU6000_CS_PIN);

  /* Setup GPIO pins for AF5 for SPI1 signals. */
  gpio_mode_setup(SPI1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN);
  gpio_set_af(SPI1_PORT, GPIO_AF5, SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN);

  /* SPI init */
  spi_disable_crc(MPU6000_SPI);
  spi_init_master(MPU6000_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                  /* high or low for the peripheral device */
                  SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  /* CPHA: Clock phase: read on rising edge of clock */
                  SPI_CR1_CPHA_CLK_TRANSITION_2,
                  /* DFF: Date frame format (8 or 16 bit) */
                  SPI_CR1_DFF_8BIT,
                  /* Most or Least Sig Bit First */
                  SPI_CR1_MSBFIRST);

  spi_enable_software_slave_management(MPU6000_SPI);
  spi_set_nss_high(MPU6000_SPI);

  spi_clear_mode_fault(MPU6000_SPI);

  spi_enable(MPU6000_SPI);
}

static uint16_t spi_read_reg(int reg)
{
  uint16_t d1, d2;

  d1 = 0x80 | (reg & 0x3f); /* Read operation */
  /* Nominallly a register read is a 16 bit operation */
  gpio_clear(SPI1_PORT, MPU6000_CS_PIN);
  d2 = spi_xfer(MPU6000_SPI, d1);
  d2 <<= 8;
  /*
   * You have to send as many bits as you want to read
   * so we send another 8 bits to get the rest of the
   * register.
   */
  d2 |= spi_xfer(MPU6000_SPI, 0);
  gpio_set(SPI1_PORT, MPU6000_CS_PIN);
  return d2;
}

static void spi_write_reg(uint8_t reg, uint8_t value)
{
  gpio_clear(SPI1_PORT, MPU6000_CS_PIN);
  (void)spi_xfer(MPU6000_SPI, reg);
  (void)spi_xfer(MPU6000_SPI, value);
  gpio_set(SPI1_PORT, MPU6000_CS_PIN);
  return;
}

void mpu6000_init(void)
{
  uint8_t id;

  printf("MPU6000 init\n");

  spi_init();

  /* Chip reset */
  spi_write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
  msleep(100);

  /* Wake up device and select GyroZ clock (better performance) */
  spi_write_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);

  /* Disable I2C bus (recommended on datasheet) */
  spi_write_reg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);

  /* Sample rate = 200Hz, Fsample= 1Khz/(4+1) = 200Hz */
  spi_write_reg(MPUREG_SMPLRT_DIV, 0x04);

  spi_write_reg(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
  spi_write_reg(MPUREG_GYRO_CONFIG, BITS_FS_500DPS);
  spi_write_reg(MPUREG_ACCEL_CONFIG, BITS_FS_4G);

  id = spi_read_reg(MPUREG_WHOAMI);
  printf("MPU device ID: %d\n", id);
}

void mpu6000_get_motion_6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t byte_H;
  uint8_t byte_L;

  gpio_clear(SPI1_PORT, MPU6000_CS_PIN);

  (void)spi_xfer(MPU6000_SPI, MPUREG_ACCEL_XOUT_H | BIT_H_RESET);

  /* Read accel X */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *ax = ((int)byte_H << 8) | byte_L;

  /* Read accel Y */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *ay = ((int)byte_H << 8) | byte_L;

  /* Read accel Z */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *az = ((int)byte_H << 8) | byte_L;

  /* Read temperature */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *gx = ((int)byte_H << 8) | byte_L;

  printf("%d, %d\n", byte_H, byte_L);

  /* Read gyro X */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *gx = ((int)byte_H << 8) | byte_L;

  printf("%d, %d\n", byte_H, byte_L);

  /* Read gyro Y */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *gy = ((int)byte_H << 8) | byte_L;

  printf("%d, %d\n", byte_H, byte_L);

  /* Read gyro Z */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *gz = ((int)byte_H << 8) | byte_L;

  gpio_set(SPI1_PORT, MPU6000_CS_PIN);
}
