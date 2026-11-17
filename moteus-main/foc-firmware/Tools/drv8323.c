/**
 * @file drv8323.c
 * @brief DRV8323S Gate Driver implementation
 *
 * SPI protocol (16-bit frames):
 *   Read:  send 0x8000 | (reg << 11), return response & 0x7FF
 *   Write: send (reg << 11) | (value & 0x7FF)
 *   CS asserted LOW during transaction, HIGH between transactions
 */

#include "drv8323.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * Lookup tables (from moteus reference fw/drv8323.cc)
 * -------------------------------------------------------------------------*/

/** DRV8323S IDRIVEP table (3-bit field, 8 values) */
static const uint16_t idrivep_table_drv8323[] = {
  10, 30, 60, 80, 120, 140, 170, 190,
  260, 330, 370, 440, 570, 680, 820, 1000,
};

/** DRV8353S IDRIVEP table (3-bit field, 8 values) */
static const uint16_t idrivep_table_drv8353[] = {
  50, 50, 100, 150, 300, 350, 400, 450,
  550, 600, 650, 700, 850, 900, 950, 1000,
};

/** DRV8323S IDRIVEN table (4-bit field, 16 values) */
static const uint16_t idriven_table_drv8323[] = {
  20, 60, 120, 160, 240, 280, 340, 380,
  520, 660, 740, 880, 1140, 1360, 1640, 2000,
};

/** DRV8353S IDRIVEN table (4-bit field, 16 values) */
static const uint16_t idriven_table_drv8353[] = {
  100, 100, 200, 300, 600, 700, 800, 900,
  1100, 1200, 1300, 1400, 1700, 1800, 1900, 2000,
};

/** TDRIVE table (2-bit field, 4 values, ns) */
static const uint16_t tdrive_table[] = { 500, 1000, 2000, 4000 };

/** DEAD_TIME table (2-bit field, 4 values, ns) */
static const uint16_t dead_time_table[] = { 50, 100, 200, 400 };

/** DRV8323S OCP deglitch table (2-bit field, 4 values, us) */
static const uint16_t deglitch_table_drv8323[] = { 2, 4, 6, 8 };

/** DRV8353S OCP deglitch table (2-bit field, 4 values, us) */
static const uint16_t deglitch_table_drv8353[] = { 1, 2, 4, 8 };

/** DRV8323S VDS level table (4-bit field, 16 values, mV) */
static const uint16_t vds_lvl_table_drv8323[] = {
  60, 130, 200, 260, 310, 450, 530, 600,
  680, 750, 940, 1130, 1300, 1500, 1700, 1880,
};

/** DRV8353S VDS level table (4-bit field, 16 values, mV) */
static const uint16_t vds_lvl_table_drv8353[] = {
  60, 70, 80, 90, 100, 200, 300, 400,
  500, 600, 700, 800, 900, 1000, 1500, 2000,
};

/** CSA gain table (2-bit field, 4 values, V/V) */
static const uint16_t csa_gain_table[] = { 5, 10, 20, 40 };

/** SEN level table (2-bit field, 4 values, mV) */
static const uint16_t sen_lvl_table[] = { 250, 500, 750, 1000 };

/* ---------------------------------------------------------------------------
 * Static helpers
 * -------------------------------------------------------------------------*/

/** Return the smallest index i where table[i] >= value, or last index. */
static uint16_t map_choice(const uint16_t *table, uint16_t table_size,
                           uint16_t value) {
  for (uint16_t i = 0; i < table_size; i++) {
    if (value <= table[i]) return i;
  }
  return table_size - 1;
}

/** Create a bitmask: (val ? 1 : 0) << pos */
static inline uint16_t bit(uint8_t val, uint8_t pos) {
  return (val ? 1U : 0U) << pos;
}

/** Microsecond delay using DWT cycle counter (Cortex-M4). */
static void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks);
}

/* ---------------------------------------------------------------------------
 * SPI transactions (16-bit, manual CS)
 * -------------------------------------------------------------------------*/

static uint16_t spi_read(DRV8323_HandleTypeDef *h, uint8_t reg) {
  uint16_t tx = 0x8000U | ((uint16_t)reg << 11);
  uint16_t rx = 0;
  HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(h->hspi, (uint8_t *)&tx, (uint8_t *)&rx, 1,
                          HAL_MAX_DELAY);
  HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);
  delay_us(1);
  return rx & 0x7FFU;
}

static void spi_write(DRV8323_HandleTypeDef *h, uint8_t reg, uint16_t value) {
  uint16_t tx = ((uint16_t)reg << 11) | (value & 0x7FFU);
  uint16_t rx = 0;
  HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(h->hspi, (uint8_t *)&tx, (uint8_t *)&rx, 1,
                          HAL_MAX_DELAY);
  HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);
  delay_us(1);
}

/* ---------------------------------------------------------------------------
 * API implementation
 * -------------------------------------------------------------------------*/

void DRV8323_Init(DRV8323_HandleTypeDef *h,
                  SPI_HandleTypeDef     *hspi,
                  GPIO_TypeDef  *cs_port, uint16_t cs_pin,
                  GPIO_TypeDef  *en_port, uint16_t en_pin,
                  GPIO_TypeDef  *flt_port, uint16_t flt_pin) {
  memset(h, 0, sizeof(*h));

  h->hspi     = hspi;
  h->cs_port  = cs_port;
  h->cs_pin   = cs_pin;
  h->en_port  = en_port;
  h->en_pin   = en_pin;
  h->flt_port = flt_port;
  h->flt_pin  = flt_pin;

  /* Enable DWT cycle counter once */
  static uint8_t dwt_init = 0;
  if (!dwt_init) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_init = 1;
  }

  /* Default config ------------------------------------------------------- */
  h->config.is_drv8323         = 1;     /* DRV8323S (vs DRV8353S) */
  h->config.dis_cpuv           = 0;
  h->config.dis_gdf            = 0;
  h->config.otw_rep            = 0;
  h->config.pwm_mode           = DRV8323_PWM_MODE_3X;
  h->config.pwm_1x_asynchronous = 0;
  h->config.pwm_1x_dir         = 0;

  h->config.idrivep_hs_ma      = 370;
  h->config.idriven_hs_ma      = 740;

  h->config.cbc                = 1;
  h->config.tdrive_ns          = 1000;
  h->config.idrivep_ls_ma      = 370;
  h->config.idriven_ls_ma      = 740;

  h->config.tretry             = 0;
  h->config.dead_time_ns       = 50;
  h->config.ocp_mode           = DRV8323_OCP_LATCHED_FAULT;
  h->config.ocp_deg_us         = 4;
  h->config.vds_lvl_mv         = 260;

  h->config.csa_fet            = 0;
  h->config.vref_div           = 1;
  h->config.ls_ref             = 0;
  h->config.csa_gain           = 20;
  h->config.dis_sen            = 0;
  h->config.sen_lvl_mv         = 500;

  h->status.fault_line         = 1;     /* assume no fault */
  h->status.enabled            = 0;
}

void DRV8323_Enable(DRV8323_HandleTypeDef *h) {
  HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_SET);
  HAL_Delay(1);  /* allow charge pump to stabilise */
  DRV8323_WriteConfig(h);

  /* CSA offset calibration: auto-zero sense amplifier offsets.
   * Without calibration, ±10mV input offset at gain=20 V/V, 0.5mΩ shunt
   * gives ±1A error per phase.  The CAL bits (reg6[4:2]) trigger an
   * internal auto-zero that shorts the amplifier inputs internally. */
  {
    uint16_t reg6 = DRV8323_ReadReg(h, DRV8323_REG_CSA_CTRL);
    /* Start: set CAL_A|CAL_B|CAL_C to begin auto-zero */
    DRV8323_WriteReg(h, DRV8323_REG_CSA_CTRL, reg6 | DRV8323_CSA_CAL_MASK);

    /* Wait ~10 µs for the calibration to settle (DWT cycle counter) */
    delay_us(10);

    /* Stop: clear CAL bits to latch the offset correction */
    DRV8323_WriteReg(h, DRV8323_REG_CSA_CTRL, reg6 & ~DRV8323_CSA_CAL_MASK);

    /* Verify: CAL bits should read back 0 */
    reg6 = DRV8323_ReadReg(h, DRV8323_REG_CSA_CTRL);
    if ((reg6 & DRV8323_CSA_CAL_MASK) != 0) {
      return;  /* calibration failed — leave enabled=0, caller checks status */
    }
  }

  h->status.enabled = 1;
}

void DRV8323_Disable(DRV8323_HandleTypeDef *h) {
  HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_RESET);
  h->status.enabled = 0;
}

uint16_t DRV8323_ReadReg(DRV8323_HandleTypeDef *h, uint8_t reg) {
  return spi_read(h, reg);
}

void DRV8323_WriteReg(DRV8323_HandleTypeDef *h, uint8_t reg, uint16_t value) {
  spi_write(h, reg, value);
}

uint8_t DRV8323_WriteConfig(DRV8323_HandleTypeDef *h) {
  const uint8_t is8323 = h->config.is_drv8323;
  const uint16_t *idrivep_tbl = is8323 ? idrivep_table_drv8323
                                       : idrivep_table_drv8353;
  const uint16_t *idriven_tbl = is8323 ? idriven_table_drv8323
                                       : idriven_table_drv8353;
  const uint16_t *deglitch_tbl = is8323 ? deglitch_table_drv8323
                                         : deglitch_table_drv8353;
  const uint16_t *vds_tbl = is8323 ? vds_lvl_table_drv8323
                                    : vds_lvl_table_drv8353;

  /* Build register values ------------------------------------------------ */
  uint16_t regs[8];

  /* reg 0, 1: read-only fault registers (not written here) */
  regs[0] = 0;
  regs[1] = 0;

  /* reg 2: DRIVE_CTRL */
  regs[2] =
      (is8323 ? 0 : bit(1, 10)) |           /* OCP_ACT (DRV8353 only) */
      bit(h->config.dis_cpuv, 9) |
      bit(h->config.dis_gdf, 8) |
      bit(h->config.otw_rep, 7) |
      ((uint16_t)h->config.pwm_mode << 5) |
      bit(h->config.pwm_1x_asynchronous, 4) |
      bit(h->config.pwm_1x_dir, 3);

  /* reg 3: GATE_DRIVE_HS */
  regs[3] =
      (is8323 ? (3U << 8) : 0) |            /* bits 9:8 = 0b11 (DRV8323) */
      (map_choice(idrivep_tbl, 16, h->config.idrivep_hs_ma) << 4) |
      (map_choice(idriven_tbl, 16, h->config.idriven_hs_ma) << 0);

  /* reg 4: GATE_DRIVE_LS */
  regs[4] =
      bit(h->config.cbc, 10) |
      (map_choice(tdrive_table, 4, h->config.tdrive_ns) << 8) |
      (map_choice(idrivep_tbl, 16, h->config.idrivep_ls_ma) << 4) |
      (map_choice(idriven_tbl, 16, h->config.idriven_ls_ma) << 0);

  /* reg 5: OCP_CTRL */
  regs[5] =
      bit(h->config.tretry, 10) |
      (map_choice(dead_time_table, 4, h->config.dead_time_ns) << 8) |
      ((uint16_t)h->config.ocp_mode << 6) |
      (map_choice(deglitch_tbl, 4, h->config.ocp_deg_us) << 4) |
      (map_choice(vds_tbl, 16, h->config.vds_lvl_mv) << 0);

  /* reg 6: CSA_CTRL */
  regs[6] =
      bit(h->config.csa_fet, 10) |
      bit(h->config.vref_div, 9) |
      bit(h->config.ls_ref, 8) |
      (map_choice(csa_gain_table, 4, h->config.csa_gain) << 6) |
      bit(h->config.dis_sen, 5) |
      (map_choice(sen_lvl_table, 4, h->config.sen_lvl_mv) << 0);

  /* reg 7: reserved (0 for DRV8323, CAL_MODE for DRV8353) */
  regs[7] = is8323 ? 0 : bit(1, 0);

  /* Write all registers 2-7 ---------------------------------------------- */
  for (uint8_t r = 2; r <= 7; r++) {
    spi_write(h, r, regs[r]);
  }

  /* Read back and verify ------------------------------------------------- */
  uint8_t fault_config = 0;
  for (uint8_t r = 2; r <= 7; r++) {
    uint16_t readback = spi_read(h, r);
    if (readback != regs[r]) {
      fault_config |= (uint8_t)(1U << r);
    }
  }

  return fault_config;
}

void DRV8323_ReadStatus(DRV8323_HandleTypeDef *h) {
  uint16_t fsr1 = spi_read(h, DRV8323_REG_FAULT_STATUS1);
  uint16_t fsr2 = spi_read(h, DRV8323_REG_FAULT_STATUS2);

  h->status.fsr1    = fsr1;
  h->status.fault   = (fsr1 >> 10) & 1;
  h->status.vds_ocp = (fsr1 >> 9) & 1;
  h->status.gdf     = (fsr1 >> 8) & 1;
  h->status.uvlo    = (fsr1 >> 7) & 1;
  h->status.otsd    = (fsr1 >> 6) & 1;
  h->status.vds_ha  = (fsr1 >> 5) & 1;
  h->status.vds_la  = (fsr1 >> 4) & 1;
  h->status.vds_hb  = (fsr1 >> 3) & 1;
  h->status.vds_lb  = (fsr1 >> 2) & 1;
  h->status.vds_hc  = (fsr1 >> 1) & 1;
  h->status.vds_lc  = (fsr1 >> 0) & 1;

  h->status.fsr2    = fsr2;
  h->status.sa_oc   = (fsr2 >> 10) & 1;
  h->status.sb_oc   = (fsr2 >> 9) & 1;
  h->status.sc_oc   = (fsr2 >> 8) & 1;
  h->status.otw     = (fsr2 >> 7) & 1;
  h->status.cpuv    = (fsr2 >> 6) & 1;
  h->status.vgs_ha  = (fsr2 >> 5) & 1;
  h->status.vgs_la  = (fsr2 >> 4) & 1;
  h->status.vgs_hb  = (fsr2 >> 3) & 1;
  h->status.vgs_lb  = (fsr2 >> 2) & 1;
  h->status.vgs_hc  = (fsr2 >> 1) & 1;
  h->status.vgs_lc  = (fsr2 >> 0) & 1;

  h->status.fault_line = HAL_GPIO_ReadPin(h->flt_port, h->flt_pin);
}

/* Returns non-zero when nFAULT pin is active-LOW (fault detected).
 * nFAULT is pulled LOW by DRV8323 when a fault condition exists. */
uint8_t DRV8323_IsFault(DRV8323_HandleTypeDef *h) {
  return !HAL_GPIO_ReadPin(h->flt_port, h->flt_pin);
}

/* ---------------------------------------------------------------------------
 * HiZ (high-impedance) pin control
 *
 * The DRV8323 HiZ pin operates as follows:
 *   HIGH → gate drive outputs enabled (normal PWM operation)
 *   LOW  → all gate drive outputs high-impedance (motor coasts freely)
 *
 * Power sequencing (caller's responsibility):
 *   PowerOff()  before Enable()   → ensure safe state during charge-pump startup
 *   PowerOn()   after  Enable()   → enable gate drives once charge pump is ready
 *   PowerOff()  before Disable()  → coast motor before fully disabling driver
 * -------------------------------------------------------------------------*/

void DRV8323_SetHiZPin(DRV8323_HandleTypeDef *h,
                       GPIO_TypeDef *port, uint16_t pin) {
  h->hiz_port = port;
  h->hiz_pin  = pin;
}

void DRV8323_PowerOn(DRV8323_HandleTypeDef *h) {
  if (!h->hiz_port) return;
  HAL_GPIO_WritePin(h->hiz_port, h->hiz_pin, GPIO_PIN_SET);
}

void DRV8323_PowerOff(DRV8323_HandleTypeDef *h) {
  if (!h->hiz_port) return;
  HAL_GPIO_WritePin(h->hiz_port, h->hiz_pin, GPIO_PIN_RESET);
}
