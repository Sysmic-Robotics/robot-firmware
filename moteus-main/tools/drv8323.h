/* DRV8323S Gate Driver — C HAL for STM32G4
 *
 * Ported from the moteus C++ reference at fw/drv8323.h / fw/drv8323.cc.
 * Uses STM32 HAL_SPI in 16-bit mode with manual CS control.
 *
 * Pin mapping (configured externally by CubeMX):
 *   SPI3: SCK=PC10, MISO=PC11, MOSI=PB5
 *   CS   = PC13,  EN = PC14,  nFAULT = PB13
 *
 * SPI3 settings: Master, 16-bit, CPOL=LOW, CPHA=2EDGE, ~10.6 MB/s, NSS Soft
 */

#ifndef DRV8323_H
#define DRV8323_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Register addresses ──────────────────────────────────────────────── */
#define DRV8323_FAULT_STATUS1  0
#define DRV8323_FAULT_STATUS2  1
#define DRV8323_DRIVE_CTRL     2
#define DRV8323_GATE_DRIVE_HS  3
#define DRV8323_GATE_DRIVE_LS  4
#define DRV8323_OCP_CTRL       5
#define DRV8323_CSA_CTRL       6

/* ── Fault Status Register 1 bit positions (0x00) ──────────────────── */
#define DRV8323_FSR1_FAULT   10
#define DRV8323_FSR1_VDS_OCP  9
#define DRV8323_FSR1_GDF      8
#define DRV8323_FSR1_UVLO     7
#define DRV8323_FSR1_OTSD     6
#define DRV8323_FSR1_VDS_HA   5
#define DRV8323_FSR1_VDS_LA   4
#define DRV8323_FSR1_VDS_HB   3
#define DRV8323_FSR1_VDS_LB   2
#define DRV8323_FSR1_VDS_HC   1
#define DRV8323_FSR1_VDS_LC   0

/* ── Fault Status Register 2 bit positions (0x01) ──────────────────── */
#define DRV8323_FSR2_SA_OC   10
#define DRV8323_FSR2_SB_OC    9
#define DRV8323_FSR2_SC_OC    8
#define DRV8323_FSR2_OTW      7
#define DRV8323_FSR2_CPUV     6
#define DRV8323_FSR2_VGS_HA   5
#define DRV8323_FSR2_VGS_LA   4
#define DRV8323_FSR2_VGS_HB   3
#define DRV8323_FSR2_VGS_LB   2
#define DRV8323_FSR2_VGS_HC   1
#define DRV8323_FSR2_VGS_LC   0

/* ── Enumerations ────────────────────────────────────────────────────── */

/** PWM mode (DRIVE_CTRL bits 6:5). */
typedef enum {
    DRV8323_PWM_MODE_6X          = 0x0,
    DRV8323_PWM_MODE_3X          = 0x1,
    DRV8323_PWM_MODE_1X          = 0x2,
    DRV8323_PWM_MODE_INDEPENDENT = 0x3,
} DRV8323_PwmMode;

/** OCP action / retry mode (OCP_CTRL bits 7:6). */
typedef enum {
    DRV8323_OCP_MODE_LATCHED      = 0,  /* Latched fault — requires reset */
    DRV8323_OCP_MODE_AUTO_RETRY   = 1,  /* Automatic retry */
    DRV8323_OCP_MODE_REPORT       = 2,  /* Report only, no action taken */
    DRV8323_OCP_MODE_IGNORED      = 3,  /* Ignored */
} DRV8323_OcpMode;

/* ── Status structure (decoded fault + housekeeping) ─────────────────── */

typedef struct {
    /* --- Fault Status Register 1 (0x00) --- */
    uint8_t fault   : 1;   /* [10] Combined fault flag */
    uint8_t vds_ocp : 1;   /* [ 9] VDS monitor OCP */
    uint8_t gdf     : 1;   /* [ 8] Gate drive fault */
    uint8_t uvlo    : 1;   /* [ 7] Undervoltage lockout */
    uint8_t otsd    : 1;   /* [ 6] Overtemperature shutdown */
    uint8_t vds_ha  : 1;   /* [ 5] VDS OCP — A high-side */
    uint8_t vds_la  : 1;   /* [ 4] VDS OCP — A low-side */
    uint8_t vds_hb  : 1;   /* [ 3] VDS OCP — B high-side */
    uint8_t vds_lb  : 1;   /* [ 2] VDS OCP — B low-side */
    uint8_t vds_hc  : 1;   /* [ 1] VDS OCP — C high-side */
    uint8_t vds_lc  : 1;   /* [ 0] VDS OCP — C low-side */

    uint16_t fsr1;          /* Raw register 0 value */

    /* --- Fault Status Register 2 (0x01) --- */
    uint8_t sa_oc   : 1;   /* [10] Sense amp OCP — phase A */
    uint8_t sb_oc   : 1;   /* [ 9] Sense amp OCP — phase B */
    uint8_t sc_oc   : 1;   /* [ 8] Sense amp OCP — phase C */
    uint8_t otw     : 1;   /* [ 7] Overtemperature warning */
    uint8_t cpuv    : 1;   /* [ 6] Charge pump undervoltage */
    uint8_t vgs_ha  : 1;   /* [ 5] Gate-drive VGS — A high */
    uint8_t vgs_la  : 1;   /* [ 4] Gate-drive VGS — A low */
    uint8_t vgs_hb  : 1;   /* [ 3] Gate-drive VGS — B high */
    uint8_t vgs_lb  : 1;   /* [ 2] Gate-drive VGS — B low */
    uint8_t vgs_hc  : 1;   /* [ 1] Gate-drive VGS — C high */
    uint8_t vgs_lc  : 1;   /* [ 0] Gate-drive VGS — C low */

    uint16_t fsr2;          /* Raw register 1 value */

    /* --- Housekeeping --- */
    uint8_t fault_line   : 1;  /* nFAULT GPIO pin state (0 = asserted) */
    uint8_t power        : 1;  /* Motors powered (HiZ state) */
    uint8_t enabled      : 1;  /* Driver enabled */

    uint8_t  fault_config;      /* Bitmask: bit i set = register i write-verify failed */
    uint16_t config_count;
    uint16_t status_count;
} DRV8323_Status;

/* ── Configuration structure (all config registers) ──────────────────── */

typedef struct {
    /* Driver Control (register 2) */
    uint8_t  dis_cpuv;              /* Disable charge-pump UVLO fault */
    uint8_t  dis_gdf;               /* Disable gate-drive fault */
    uint8_t  otw_rep;               /* Report OTW on nFAULT pin */
    DRV8323_PwmMode pwm_mode;
    uint8_t  pwm_1x_asynchronous;   /* 1X PWM asynchronous mode */
    uint8_t  pwm_1x_dir;            /* 1X PWM direction */

    /* Gate Drive HS (register 3) */
    uint16_t idrivep_hs_ma;         /* Peak (source) gate current, mA */
    uint16_t idriven_hs_ma;         /* Sink gate current, mA */

    /* Gate Drive LS (register 4) */
    uint8_t  cbc;                   /* Cycle-by-cycle operation */
    uint16_t tdrive_ns;             /* Peak gate drive time, ns */
    uint16_t idrivep_ls_ma;         /* Peak (source) gate current, mA */
    uint16_t idriven_ls_ma;         /* Sink gate current, mA */

    /* OCP Control (register 5) */
    uint8_t         tretry;         /* 0 = 4 ms retry, 1 = 50 us retry */
    uint16_t        dead_time_ns;   /* Dead time, ns */
    DRV8323_OcpMode ocp_mode;
    uint8_t         ocp_deg_us;     /* OCP deglitch time, us */
    uint16_t        vds_lvl_mv;     /* VDS overcurrent threshold, mV */

    /* CSA Control (register 6) */
    uint8_t  csa_fet;               /* 0 = shunt, 1 = FET sense */
    uint8_t  vref_div;              /* 1 = VREF/2 (bidirectional) */
    uint8_t  ls_ref;                /* OCP reference selection */
    uint8_t  csa_gain;              /* 5, 10, 20, or 40 V/V */
    uint8_t  dis_sen;               /* Disable sense overcurrent */
    uint16_t sen_lvl_mv;            /* Sense OCP level (250/500/750/1000) mV */
} DRV8323_Config;

/* ── Handle type (opaque-ish) ────────────────────────────────────────── */

typedef struct {
    /* HAL handles */
    SPI_HandleTypeDef *hspi;

    /* GPIOs (set once at init, driven externally) */
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
    GPIO_TypeDef *en_port;
    uint16_t      en_pin;
    GPIO_TypeDef *fault_port;
    uint16_t      fault_pin;

    /** Current configuration (set defaults in Init, modify before Enable). */
    DRV8323_Config config;

    /** Decoded fault / status (updated by ReadStatus). */
    DRV8323_Status status;

    /** 1 = DRV8323 tables/behaviour, 0 = DRV8353 (default: 1). */
    uint8_t is_drv8323;

    /** Currently enabled. */
    uint8_t enabled;
} DRV8323_HandleTypeDef;

/* ── API ─────────────────────────────────────────────────────────────── */

/** Initialise handle, set config defaults, initialise DWT cycle counter. */
HAL_StatusTypeDef DRV8323_Init(DRV8323_HandleTypeDef *hdrv,
                               SPI_HandleTypeDef    *hspi,
                               GPIO_TypeDef *cs_port,    uint16_t cs_pin,
                               GPIO_TypeDef *en_port,    uint16_t en_pin,
                               GPIO_TypeDef *fault_port, uint16_t fault_pin);

/** Load safe register defaults into hdrv->config (idempotent). */
void DRV8323_SetDefaultConfig(DRV8323_HandleTypeDef *hdrv);

/**
 * Enable the driver: assert EN, wait 1 ms, write all config registers
 * via SPI, verify by readback.  Returns HAL_OK on success.
 */
HAL_StatusTypeDef DRV8323_Enable(DRV8323_HandleTypeDef *hdrv);

/** Disable the driver: de-assert EN, clear fault_config. */
void DRV8323_Disable(DRV8323_HandleTypeDef *hdrv);

/**
 * Read a register (0–6).
 * @return 11-bit register value (bits 10:0).
 */
uint16_t DRV8323_ReadReg(DRV8323_HandleTypeDef *hdrv, uint8_t reg);

/** Write an 11-bit value to a register (0–6). */
HAL_StatusTypeDef DRV8323_WriteReg(DRV8323_HandleTypeDef *hdrv,
                                   uint8_t reg, uint16_t value);

/**
 * Write all active config registers (2–7), then read back and verify.
 * Sets hdrv->status.fault_config on mismatch.
 */
HAL_StatusTypeDef DRV8323_WriteConfig(DRV8323_HandleTypeDef *hdrv);

/**
 * Read fault status registers (0–1), decode bits into hdrv->status,
 * also sample nFAULT GPIO and enable state.
 */
void DRV8323_ReadStatus(DRV8323_HandleTypeDef *hdrv);

/** Read the nFAULT GPIO pin directly.  Returns 1 if fault is asserted. */
uint8_t DRV8323_IsFault(DRV8323_HandleTypeDef *hdrv);

#ifdef __cplusplus
}
#endif

#endif /* DRV8323_H */
