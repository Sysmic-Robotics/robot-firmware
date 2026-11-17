/* DRV8323S Gate Driver — C HAL Implementation for STM32G4
 *
 * Ported from moteus C++ reference fw/drv8323.cc.
 * Uses STM32 HAL SPI (16-bit frames, manual CS) + DWT microsecond delay.
 *
 * SPI protocol (16-bit word):
 *   Bit 15       = R/W#  (1 = read, 0 = write)
 *   Bits 14–11   = Register address (4 bits)
 *   Bits 10–0    = Data (11 bits)
 *
 * Read:  send 0x8000 | (reg << 11)     → response lower 11 bits = register value
 * Write: send (reg << 11) | (value & 0x7ff)
 */

#include "drv8323.h"

#include <string.h>   /* memset */

/* ====================================================================
 *  DWT microsecond delay (Cortex-M4 cycle counter)
 * ==================================================================== */

static int dwt_initialised = 0;

static void DWT_Init(void)
{
    if (dwt_initialised) return;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_initialised = 1;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    /* SystemCoreClock is set by HAL_Init() / SystemClock_Config(). */
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

/* ====================================================================
 *  Look-up tables  (from moteus fw/drv8323.cc WriteConfig)
 * ==================================================================== */

/** IDRIVEP (peak/source) current, mA — DRV8323 (16 levels). */
static const uint16_t idrivep_table_drv8323[] = {
    10, 30, 60, 80, 120, 140, 170, 190,
    260, 330, 370, 440, 570, 680, 820, 1000,
};

/** IDRIVEP (peak/source) current, mA — DRV8353. */
static const uint16_t idrivep_table_drv8353[] = {
    50, 50, 100, 150, 300, 350, 400, 450,
    550, 600, 650, 700, 850, 900, 950, 1000,
};

/** IDRIVEN (sink) current, mA — DRV8323 (16 levels). */
static const uint16_t idriven_table_drv8323[] = {
    20, 60, 120, 160, 240, 280, 340, 380,
    520, 660, 740, 880, 1140, 1360, 1640, 2000,
};

/** IDRIVEN (sink) current, mA — DRV8353. */
static const uint16_t idriven_table_drv8353[] = {
    100, 100, 200, 300, 600, 700, 800, 900,
    1100, 1200, 1300, 1400, 1700, 1800, 1900, 2000,
};

/** TDRIVE peak-gate drive time, ns (4 levels). */
static const uint16_t tdrive_ns_table[] = {
    500, 1000, 2000, 4000,
};

/** DEAD_TIME, ns (4 levels). */
static const uint16_t dead_time_table[] = {
    50, 100, 200, 400,
};

/** OCP deglitch time, us — DRV8323 (4 levels). */
static const uint16_t deglitch_table_drv8323[] = {
    2, 4, 6, 8,
};

/** OCP deglitch time, us — DRV8353. */
static const uint16_t deglitch_table_drv8353[] = {
    1, 2, 4, 8,
};

/** VDS overcurrent threshold, mV — DRV8323 (16 levels). */
static const uint16_t vds_lvl_table_drv8323[] = {
    60, 130, 200, 260, 310, 450, 530, 600,
    680, 750, 940, 1130, 1300, 1500, 1700, 1880,
};

/** VDS overcurrent threshold, mV — DRV8353. */
static const uint16_t vds_lvl_table_drv8353[] = {
    60, 70, 80, 90, 100, 200, 300, 400,
    500, 600, 700, 800, 900, 1000, 1500, 2000,
};

/** CSA gain, V/V (4 levels). */
static const uint16_t csa_gain_table[] = {
    5, 10, 20, 40,
};

/** SEN_LVL sense OCP threshold, mV (4 levels). */
static const uint16_t sen_lvl_table[] = {
    250, 500, 750, 1000,
};

/* ====================================================================
 *  Static helpers
 * ==================================================================== */

/** Return (val << pos) — convenience for building register bitfields. */
static inline uint16_t bit(uint8_t val, uint8_t pos)
{
    return (val ? 1U : 0U) << pos;
}

/**
 * Find the smallest table index whose entry is >= ma.
 * If no entry satisfies the condition, return the last index.
 * Mirrors the C++ lambda `map_choice` in the moteus reference.
 */
static uint16_t map_choice(const uint16_t *table, uint16_t table_len,
                           uint16_t ma)
{
    for (uint16_t i = 0; i < table_len; i++) {
        if (ma <= table[i]) {
            return i;
        }
    }
    return table_len - 1;
}

/** Convenience macro for static-sized tables. */
#define MAP_CHOICE(TABLE, MA) \
    map_choice(TABLE, sizeof(TABLE) / sizeof((TABLE)[0]), MA)

/* ====================================================================
 *  Low-level SPI helpers
 * ==================================================================== */

/**
 * Assert chip-select (active low).
 */
static inline void cs_low(DRV8323_HandleTypeDef *hdrv)
{
    HAL_GPIO_WritePin(hdrv->cs_port, hdrv->cs_pin, GPIO_PIN_RESET);
}

/**
 * De-assert chip-select.
 */
static inline void cs_high(DRV8323_HandleTypeDef *hdrv)
{
    HAL_GPIO_WritePin(hdrv->cs_port, hdrv->cs_pin, GPIO_PIN_SET);
}

/**
 * Transmit one 16-bit frame and receive the response.
 * SPI must be configured for 16-bit data size.
 */
static uint16_t spi_transceive(DRV8323_HandleTypeDef *hdrv, uint16_t tx_data)
{
    uint16_t rx_data = 0;
    HAL_SPI_TransmitReceive(hdrv->hspi,
                            (uint8_t *)&tx_data,
                            (uint8_t *)&rx_data,
                            1,
                            HAL_MAX_DELAY);
    return rx_data;
}

/* ====================================================================
 *  API implementation
 * ==================================================================== */

/* ------------------------------------------------------------------ */
/*  DRV8323_Init                                                       */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef DRV8323_Init(DRV8323_HandleTypeDef *hdrv,
                               SPI_HandleTypeDef    *hspi,
                               GPIO_TypeDef *cs_port,    uint16_t cs_pin,
                               GPIO_TypeDef *en_port,    uint16_t en_pin,
                               GPIO_TypeDef *fault_port, uint16_t fault_pin)
{
    if (hdrv == NULL) return HAL_ERROR;

    memset(hdrv, 0, sizeof(*hdrv));

    /* Store handles. */
    hdrv->hspi       = hspi;
    hdrv->cs_port    = cs_port;
    hdrv->cs_pin     = cs_pin;
    hdrv->en_port    = en_port;
    hdrv->en_pin     = en_pin;
    hdrv->fault_port = fault_port;
    hdrv->fault_pin  = fault_pin;

    hdrv->is_drv8323 = 1;   /* Default: DRV8323 register tables */

    /* Start DWT cycle counter for microsecond delays. */
    DWT_Init();

    /* Load safe defaults into config. */
    DRV8323_SetDefaultConfig(hdrv);

    /* Ensure CS is de-asserted and EN is off. */
    cs_high(hdrv);
    HAL_GPIO_WritePin(hdrv->en_port, hdrv->en_pin, GPIO_PIN_RESET);

    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/*  DRV8323_SetDefaultConfig                                           */
/* ------------------------------------------------------------------ */
void DRV8323_SetDefaultConfig(DRV8323_HandleTypeDef *hdrv)
{
    DRV8323_Config *cfg = &hdrv->config;

    /* Driver Control (register 2) */
    cfg->dis_cpuv            = 0;
    cfg->dis_gdf             = 0;
    cfg->otw_rep             = 0;
    cfg->pwm_mode            = DRV8323_PWM_MODE_3X;
    cfg->pwm_1x_asynchronous = 0;
    cfg->pwm_1x_dir          = 0;

    /* Gate Drive HS (register 3) — typical for DRV8323 */
    cfg->idrivep_hs_ma = 370;
    cfg->idriven_hs_ma = 740;

    /* Gate Drive LS (register 4) */
    cfg->cbc           = 1;
    cfg->tdrive_ns     = 1000;
    cfg->idrivep_ls_ma = 370;
    cfg->idriven_ls_ma = 740;

    /* OCP Control (register 5) */
    cfg->tretry       = 0;
    cfg->dead_time_ns = 50;
    cfg->ocp_mode     = DRV8323_OCP_MODE_LATCHED;
    cfg->ocp_deg_us   = 4;
    cfg->vds_lvl_mv   = 260;

    /* CSA Control (register 6) */
    cfg->csa_fet   = 0;
    cfg->vref_div  = 1;
    cfg->ls_ref    = 0;
    cfg->csa_gain  = 20;
    cfg->dis_sen   = 0;
    cfg->sen_lvl_mv = 500;
}

/* ------------------------------------------------------------------ */
/*  DRV8323_ReadReg                                                    */
/* ------------------------------------------------------------------ */
uint16_t DRV8323_ReadReg(DRV8323_HandleTypeDef *hdrv, uint8_t reg)
{
    const uint16_t cmd = 0x8000 | ((uint16_t)(reg & 0x0F) << 11);

    cs_low(hdrv);
    const uint16_t result = spi_transceive(hdrv, cmd) & 0x7FF;
    cs_high(hdrv);

    /* ≥ 1 µs CS deselect time (the reference uses wait_us(1)). */
    delay_us(1);

    return result;
}

/* ------------------------------------------------------------------ */
/*  DRV8323_WriteReg                                                   */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef DRV8323_WriteReg(DRV8323_HandleTypeDef *hdrv,
                                   uint8_t reg, uint16_t value)
{
    const uint16_t cmd = ((uint16_t)(reg & 0x0F) << 11) | (value & 0x7FF);

    cs_low(hdrv);
    spi_transceive(hdrv, cmd);
    cs_high(hdrv);

    delay_us(1);

    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/*  DRV8323_WriteConfig                                                */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef DRV8323_WriteConfig(DRV8323_HandleTypeDef *hdrv)
{
    const DRV8323_Config *cfg = &hdrv->config;
    const uint16_t *idrivep_tbl = hdrv->is_drv8323
                                      ? idrivep_table_drv8323
                                      : idrivep_table_drv8353;
    const uint16_t *idriven_tbl = hdrv->is_drv8323
                                      ? idriven_table_drv8323
                                      : idriven_table_drv8353;
    const uint16_t *deglitch_tbl = hdrv->is_drv8323
                                       ? deglitch_table_drv8323
                                       : deglitch_table_drv8353;
    const uint16_t *vds_lvl_tbl  = hdrv->is_drv8323
                                       ? vds_lvl_table_drv8323
                                       : vds_lvl_table_drv8353;

    /* ---- Register 2: Driver Control ---- */
    const uint16_t reg2 =
        (hdrv->is_drv8323 ? 0 : bit(1, 10)) |  /* OCP_ACT (DRV8353 only) */
        bit(cfg->dis_cpuv, 9) |
        bit(cfg->dis_gdf,  8) |
        bit(cfg->otw_rep,  7) |
        (((uint16_t)cfg->pwm_mode & 0x03) << 5) |
        bit(cfg->pwm_1x_asynchronous, 4) |
        bit(cfg->pwm_1x_dir, 3);

    /* ---- Register 3: Gate Drive HS ---- */
    const uint16_t reg3 =
        (3U << 8) |                                    /* reserved = 0b11 */
        (MAP_CHOICE(idrivep_tbl, cfg->idrivep_hs_ma) << 4) |
        (MAP_CHOICE(idriven_tbl, cfg->idriven_hs_ma) << 0);

    /* ---- Register 4: Gate Drive LS ---- */
    const uint16_t reg4 =
        bit(cfg->cbc, 10) |
        (MAP_CHOICE(tdrive_ns_table, cfg->tdrive_ns) << 8) |
        (MAP_CHOICE(idrivep_tbl, cfg->idrivep_ls_ma) << 4) |
        (MAP_CHOICE(idriven_tbl, cfg->idriven_ls_ma) << 0);

    /* ---- Register 5: OCP Control ---- */
    const uint16_t reg5 =
        bit(cfg->tretry, 10) |
        (MAP_CHOICE(dead_time_table, cfg->dead_time_ns) << 8) |
        (((uint16_t)cfg->ocp_mode & 0x03) << 6) |
        (MAP_CHOICE(deglitch_tbl, cfg->ocp_deg_us) << 4) |
        (MAP_CHOICE(vds_lvl_tbl,  cfg->vds_lvl_mv)  << 0);

    /* ---- Register 6: CSA Control ---- */
    const uint16_t reg6 =
        bit(cfg->csa_fet,  10) |
        bit(cfg->vref_div,  9) |
        bit(cfg->ls_ref,    8) |
        (MAP_CHOICE(csa_gain_table, cfg->csa_gain) << 6) |
        bit(cfg->dis_sen,   5) |
        (MAP_CHOICE(sen_lvl_table, cfg->sen_lvl_mv) << 0);

    /* ---- Register 7: Reserved / CAL_MODE ---- */
    const uint16_t reg7 =
        (hdrv->is_drv8323 ? 0 : bit(1, 0));  /* CAL_MODE (DRV8353 only) */

    const uint16_t regs[] = { 0, 0, reg2, reg3, reg4, reg5, reg6, reg7 };

    /* First pass: write all registers 2–7. */
    for (uint8_t i = 2; i <= 7; i++) {
        DRV8323_WriteReg(hdrv, i, regs[i]);
    }

    /* Second pass: read back and verify. */
    uint8_t fault_config = 0;
    for (uint8_t i = 2; i <= 7; i++) {
        const uint16_t readback = DRV8323_ReadReg(hdrv, i);
        if (readback != regs[i]) {
            fault_config |= (uint8_t)(1U << i);
        }
    }

    hdrv->status.fault_config = fault_config;
    hdrv->status.config_count++;

    return (fault_config == 0) ? HAL_OK : HAL_ERROR;
}

/* ------------------------------------------------------------------ */
/*  DRV8323_Enable / Disable                                           */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef DRV8323_Enable(DRV8323_HandleTypeDef *hdrv)
{
    /* Assert enable pin. */
    HAL_GPIO_WritePin(hdrv->en_port, hdrv->en_pin, GPIO_PIN_SET);

    /* Wait for the driver to stabilise (datasheet t_ENABLE ~ 10 µs,
     * we use 1 ms to be safe — matching the reference's kEnabling1 wait). */
    delay_us(1000);

    /* Write all config registers and verify. */
    HAL_StatusTypeDef ret = DRV8323_WriteConfig(hdrv);

    if (ret == HAL_OK) {
        hdrv->enabled = 1;
    }

    return ret;
}

void DRV8323_Disable(DRV8323_HandleTypeDef *hdrv)
{
    HAL_GPIO_WritePin(hdrv->en_port, hdrv->en_pin, GPIO_PIN_RESET);
    hdrv->enabled          = 0;
    hdrv->status.fault_config = 0;
}

/* ------------------------------------------------------------------ */
/*  DRV8323_ReadStatus                                                 */
/* ------------------------------------------------------------------ */
void DRV8323_ReadStatus(DRV8323_HandleTypeDef *hdrv)
{
    DRV8323_Status *s = &hdrv->status;

    /* Sample GPIOs first. */
    s->fault_line = (HAL_GPIO_ReadPin(hdrv->fault_port, hdrv->fault_pin)
                     == GPIO_PIN_RESET) ? 1 : 0;   /* nFAULT is active-low */
    s->enabled    = hdrv->enabled;

    /* If the driver is not enabled, we cannot communicate over SPI. */
    if (!hdrv->enabled) {
        return;
    }

    /* Read fault registers. */
    const uint16_t fsr1 = DRV8323_ReadReg(hdrv, DRV8323_FAULT_STATUS1);
    const uint16_t fsr2 = DRV8323_ReadReg(hdrv, DRV8323_FAULT_STATUS2);

    /* Decode FSR1 (register 0). */
    s->fault   = (fsr1 >> DRV8323_FSR1_FAULT  ) & 1U;
    s->vds_ocp = (fsr1 >> DRV8323_FSR1_VDS_OCP) & 1U;
    s->gdf     = (fsr1 >> DRV8323_FSR1_GDF    ) & 1U;
    s->uvlo    = (fsr1 >> DRV8323_FSR1_UVLO   ) & 1U;
    s->otsd    = (fsr1 >> DRV8323_FSR1_OTSD   ) & 1U;
    s->vds_ha  = (fsr1 >> DRV8323_FSR1_VDS_HA ) & 1U;
    s->vds_la  = (fsr1 >> DRV8323_FSR1_VDS_LA ) & 1U;
    s->vds_hb  = (fsr1 >> DRV8323_FSR1_VDS_HB ) & 1U;
    s->vds_lb  = (fsr1 >> DRV8323_FSR1_VDS_LB ) & 1U;
    s->vds_hc  = (fsr1 >> DRV8323_FSR1_VDS_HC ) & 1U;
    s->vds_lc  = (fsr1 >> DRV8323_FSR1_VDS_LC ) & 1U;
    s->fsr1    = fsr1;

    /* Decode FSR2 (register 1). */
    s->sa_oc  = (fsr2 >> DRV8323_FSR2_SA_OC  ) & 1U;
    s->sb_oc  = (fsr2 >> DRV8323_FSR2_SB_OC  ) & 1U;
    s->sc_oc  = (fsr2 >> DRV8323_FSR2_SC_OC  ) & 1U;
    s->otw    = (fsr2 >> DRV8323_FSR2_OTW    ) & 1U;
    s->cpuv   = (fsr2 >> DRV8323_FSR2_CPUV   ) & 1U;
    s->vgs_ha = (fsr2 >> DRV8323_FSR2_VGS_HA ) & 1U;
    s->vgs_la = (fsr2 >> DRV8323_FSR2_VGS_LA ) & 1U;
    s->vgs_hb = (fsr2 >> DRV8323_FSR2_VGS_HB ) & 1U;
    s->vgs_lb = (fsr2 >> DRV8323_FSR2_VGS_LB ) & 1U;
    s->vgs_hc = (fsr2 >> DRV8323_FSR2_VGS_HC ) & 1U;
    s->vgs_lc = (fsr2 >> DRV8323_FSR2_VGS_LC ) & 1U;
    s->fsr2   = fsr2;

    s->status_count++;
}

/* ------------------------------------------------------------------ */
/*  DRV8323_IsFault                                                    */
/* ------------------------------------------------------------------ */
uint8_t DRV8323_IsFault(DRV8323_HandleTypeDef *hdrv)
{
    /* nFAULT is active-low — return 1 when asserted. */
    return (HAL_GPIO_ReadPin(hdrv->fault_port, hdrv->fault_pin)
            == GPIO_PIN_RESET) ? 1 : 0;
}
