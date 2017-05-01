#include "mpu6000.h"

#include <stdint.h>
#include <stdio.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/spi.h>

#include "MadgwickAHRS.h"
#include "clock.h"

#define GYRO_FACTOR 16.4f
#define ACCEL_FACTOR 8192.0f

#define DEG_TO_RAD 0.017453292519943295f

volatile uint64_t mpu6000_samples = 0;
volatile float mpu6000_axis[6];
volatile float mpu6000_world_accel[3];

void exti4_isr(void)
{
  exti_reset_request(MPU6000_EXTI);

  static int16_t ax, ay, az, gx, gy, gz;
  mpu6000_get_motion_6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu6000_axis[0] = (gx / GYRO_FACTOR) * DEG_TO_RAD;
  mpu6000_axis[1] = (gy / GYRO_FACTOR) * DEG_TO_RAD;
  mpu6000_axis[2] = (gz / GYRO_FACTOR) * DEG_TO_RAD;
  mpu6000_axis[3] = ax / ACCEL_FACTOR;
  mpu6000_axis[4] = ay / ACCEL_FACTOR;
  mpu6000_axis[5] = az / ACCEL_FACTOR;

  MadgwickAHRSupdateIMU(mpu6000_axis[0], mpu6000_axis[1], mpu6000_axis[2], mpu6000_axis[3],
                        mpu6000_axis[4], mpu6000_axis[5]);

  mpu6000_samples++;
}

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

static uint8_t spi_read_reg(int reg)
{
  uint8_t val;

  gpio_clear(SPI1_PORT, MPU6000_CS_PIN);

  (void)spi_xfer(MPU6000_SPI, reg | 0x80);
  val = spi_xfer(MPU6000_SPI, 0);

  gpio_set(SPI1_PORT, MPU6000_CS_PIN);

  return val;
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
  uint8_t id, mode;

  printf("MPU6000 init\n");

  spi_init();

  /* Chip reset */
  spi_write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
  msleep(150);

  /* Signal path reset */
  spi_write_reg(MPUREG_SIGNAL_PATH_RESET, 0x7);
  msleep(150);

  /* Wake up device and select gyro Z clock (better performance) */
  spi_write_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
  msleep(10);

  /* Disable I2C bus (recommended on datasheet) */
  spi_write_reg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
  msleep(10);

  spi_write_reg(MPUREG_SMPLRT_DIV, 0x04);
  msleep(10);

  spi_write_reg(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
  msleep(10);

  spi_write_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
  msleep(10);

  spi_write_reg(MPUREG_ACCEL_CONFIG, BITS_FS_4G);
  msleep(10);

  spi_write_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
  msleep(10);

  spi_write_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR);
  msleep(10);

  msleep(1000);

  id = spi_read_reg(MPUREG_WHOAMI);
  printf("MPU device ID: %#04x\n", id);

  mode = spi_read_reg(MPUREG_GYRO_CONFIG);
  printf("Gyro config: %#04x\n", mode);

  mode = spi_read_reg(MPUREG_ACCEL_CONFIG);
  printf("Accel config: %#04x\n", mode & BITS_FS_MASK);

  /* Interrupt */
  nvic_enable_irq(NVIC_EXTI4_IRQ);
  gpio_mode_setup(MPU6000_INT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, MPU6000_INT_PIN);
  exti_select_source(MPU6000_EXTI, MPU6000_INT_PORT);
  exti_set_trigger(MPU6000_EXTI, EXTI_TRIGGER_RISING);
  exti_enable_request(MPU6000_EXTI);
}

void mpu6000_get_motion_6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy,
                          int16_t *gz)
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

  /* Read gyro X */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *gx = ((int)byte_H << 8) | byte_L;

  /* Read gyro Y */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *gy = ((int)byte_H << 8) | byte_L;

  /* Read gyro Z */
  byte_H = spi_xfer(MPU6000_SPI, 0);
  byte_L = spi_xfer(MPU6000_SPI, 0);
  *gz = ((int)byte_H << 8) | byte_L;

  gpio_set(SPI1_PORT, MPU6000_CS_PIN);
}
