#include "mpu6000.h"

#include <math.h>
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
#include "leds.h"

#define GYRO_FACTOR 16.4f
#define ACCEL_FACTOR 8192.0f

#define ACCEL_THR 0.01f

#define DEG_TO_RAD 0.017453292519943295f

volatile uint64_t mpu6000_samples = 0;
volatile int16_t mpu6000_acc_calib[3];
volatile float mpu6000_axis[6];
volatile float mpu6000_gravity[3];
volatile float mpu6000_linear_accel[3];
volatile float mpu6000_world_accel[3];
volatile uint16_t mpu6000_world_accel_cons_zeros[3];
volatile float mpu6000_world_velocity[3];
volatile float mpu6000_world_displacement[3];

typedef struct
{
  float w, x, y, z;
} Quat;

static Quat quat_mult(Quat a, Quat b)
{
  Quat o;

  o.w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
  o.x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
  o.y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x);
  o.z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w);

  return o;
}

static void rotate_point_by_quat(float qw, float qx, float qy, float qz, float *x, float *y,
                                 float *z)
{
  Quat q = {qw, qx, qy, qz};
  Quat p = {0.0f, *x, *y, *z};

  Quat qc = {q.w, -q.x, -q.y, -q.z};

  p = quat_mult(q, p);
  p = quat_mult(p, qc);

  *x = p.x;
  *y = p.y;
  *z = p.z;
}

void exti4_isr(void)
{
  exti_reset_request(MPU6000_EXTI);

  /* Sample */
  static int16_t ax, ay, az, gx, gy, gz;
  mpu6000_get_motion_6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu6000_axis[0] = (gx / GYRO_FACTOR) * DEG_TO_RAD;
  mpu6000_axis[1] = (gy / GYRO_FACTOR) * DEG_TO_RAD;
  mpu6000_axis[2] = (gz / GYRO_FACTOR) * DEG_TO_RAD;
  mpu6000_axis[3] = (ax - mpu6000_acc_calib[0]) / ACCEL_FACTOR;
  mpu6000_axis[4] = (ay - mpu6000_acc_calib[1]) / ACCEL_FACTOR;
  mpu6000_axis[5] = (az - mpu6000_acc_calib[2]) / ACCEL_FACTOR;

  /* Update Madgwick filter for orientation quaternion */
  MadgwickAHRSupdateIMU(mpu6000_axis[0], mpu6000_axis[1], mpu6000_axis[2], mpu6000_axis[3],
                        mpu6000_axis[4], mpu6000_axis[5]);

  /* Obtain gravity vector */
  mpu6000_gravity[0] = 2 * (q1 * q3 - q0 * q2);
  mpu6000_gravity[1] = 2 * (q0 * q1 + q2 * q3);
  mpu6000_gravity[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  /* Take a copy of acceleromter data with gravity removed */
  static float axf, ayf, azf;
  axf = mpu6000_axis[3] - mpu6000_gravity[0];
  ayf = mpu6000_axis[4] - mpu6000_gravity[1];
  azf = mpu6000_axis[5] - mpu6000_gravity[2];

  mpu6000_linear_accel[0] = axf;
  mpu6000_linear_accel[1] = ayf;
  mpu6000_linear_accel[2] = azf;

  /* Obtain world acceleration */
  rotate_point_by_quat(q0, q1, q2, q3, &axf, &ayf, &azf);
  mpu6000_world_accel[0] = axf;
  mpu6000_world_accel[1] = ayf;
  mpu6000_world_accel[2] = azf;

  static int i;
  for (i = 0; i < 3; i++)
  {
    /* Apply deadband/discrimination check (filters mechanical noise) */
    if (fabs(mpu6000_world_accel[i]) < ACCEL_THR)
    {
      mpu6000_world_accel[i] = 0.0f;
      mpu6000_world_accel_cons_zeros[i]++;
    }
    else
    {
      mpu6000_world_accel_cons_zeros[i] = 0;
    }

    /* Perform "no motion" check */
    if (mpu6000_world_accel_cons_zeros[i] < 20)
    {
      /* Update integration */
      mpu6000_world_accel[i] *= 9.8f;
      mpu6000_world_velocity[i] += mpu6000_world_accel[i] * (1.0f / sampleFreq);
      mpu6000_world_displacement[i] += mpu6000_world_velocity[i] * (1.0f / sampleFreq);
    }
    else
    {
      /* Reset velocity if sensor has not changed acceleration in a number of samples */
      mpu6000_world_velocity[i] = 0.0f;
    }
  }

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
  int i;

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

  /* F(sample) = F(gyro) / (1 + MPUREG_SMPLRT_DIV) */
  /* F(gyro) = 8kHz (when DLPF is disabled (DLPF_CFG = 0 or 7)) */
  /* F(gyro) = 1kHz (when DLPF is enabled) */
  spi_write_reg(MPUREG_SMPLRT_DIV, 4);
  msleep(10);

  spi_write_reg(MPUREG_CONFIG, BITS_DLPF_CFG_2100HZ_NOLPF);
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

  for (i = 0; i < 3; i++)
    mpu6000_world_accel_cons_zeros[i] = 0;

  mpu6000_reset_integrators();
}

void mpu6000_calibrate_acc(void)
{
  static const int NUM_SAMPLES = 10000;

  int i;
  int64_t x_samples, y_samples, z_samples;
  int16_t dummy, x, y, z;

  exti_disable_request(MPU6000_EXTI);

  gpio_clear(LED0_PORT, LED0_PIN);
  msleep(50);
  gpio_set(LED0_PORT, LED0_PIN);

  x_samples = 0;
  y_samples = 0;
  z_samples = 0;

  for (i = 0; i < NUM_SAMPLES; i++)
  {
    mpu6000_get_motion_6(&x, &y, &z, &dummy, &dummy, &dummy);
    x_samples += x;
    y_samples += y;
    z_samples += z;

    msleep(1);
  }

  x_samples /= NUM_SAMPLES;
  y_samples /= NUM_SAMPLES;
  z_samples /= NUM_SAMPLES;

  mpu6000_acc_calib[0] = x_samples;
  mpu6000_acc_calib[1] = y_samples;
  mpu6000_acc_calib[2] = z_samples - ACCEL_FACTOR;

  gpio_clear(LED0_PORT, LED0_PIN);
  msleep(50);
  gpio_set(LED0_PORT, LED0_PIN);

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

void mpu6000_reset_integrators(void)
{
  int i;
  for (i = 0; i < 3; i++)
  {
    mpu6000_world_velocity[i] = 0.0f;
    mpu6000_world_displacement[i] = 0.0f;
  }
}
