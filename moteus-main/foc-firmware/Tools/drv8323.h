/**
 * @file drv8323.h
 * @brief DRV8323S Gate Driver (SPI mode, STM32G4 HAL)
 *
 * Pin connections (configured by CubeMX):
 *   SPI3: SCK=PC10, MISO=PC11, MOSI=PB5
 *   CS   = PC13  (GPIO output)
 *   EN   = PC14  (GPIO output)
 *   FAULT = PB13 (GPIO input, active LOW)
 *
 * SPI3 config: Master, 16-bit, CPOL=LOW, CPHA=2EDGE, NSS Soft, ~10.6 MB/s
 */

#ifndef DRV8323_H
#define DRV8323_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Register addresses ------------------------------------------------------ */
#define DRV8323_CSA_CAL_MASK      0x1C  /* bits 2:4 = CAL_A|CAL_B|CAL_C */
#define DRV8323_REG_FAULT_STATUS1  0
#define DRV8323_REG_FAULT_STATUS2  1
#define DRV8323_REG_DRIVE_CTRL     2
#define DRV8323_REG_GATE_DRIVE_HS  3
#define DRV8323_REG_GATE_DRIVE_LS  4
#define DRV8323_REG_OCP_CTRL       5
#define DRV8323_REG_CSA_CTRL       6

/* PWM mode ---------------------------------------------------------------- */
typedef enum {
  DRV8323_PWM_MODE_6X          = 0,
  DRV8323_PWM_MODE_3X          = 1,
  DRV8323_PWM_MODE_1X          = 2,
  DRV8323_PWM_MODE_INDEPENDENT = 3,
} DRV8323_PwmMode;

/* OCP mode ---------------------------------------------------------------- */
typedef enum {
  DRV8323_OCP_LATCHED_FAULT  = 0,
  DRV8323_OCP_AUTO_RETRY     = 1,
  DRV8323_OCP_REPORT_ONLY    = 2,
  DRV8323_OCP_IGNORED        = 3,
} DRV8323_OcpMode;

/* Decoded fault status ---------------------------------------------------- */
typedef struct {
  /* reg 0: FAULT_STATUS1 */
  uint8_t  fault;        /* bit 10: summary fault */
  uint8_t  vds_ocp;      /* bit  9: VDS overcurrent */
  uint8_t  gdf;          /* bit  8: gate drive fault */
  uint8_t  uvlo;         /* bit  7: undervoltage lockout */
  uint8_t  otsd;         /* bit  6: overtemperature shutdown */
  uint8_t  vds_ha;       /* bit  5: VDS overcurrent phase A high */
  uint8_t  vds_la;       /* bit  4: VDS overcurrent phase A low */
  uint8_t  vds_hb;       /* bit  3: VDS overcurrent phase B high */
  uint8_t  vds_lb;       /* bit  2: VDS overcurrent phase B low */
  uint8_t  vds_hc;       /* bit  1: VDS overcurrent phase C high */
  uint8_t  vds_lc;       /* bit  0: VDS overcurrent phase C low */
  uint16_t fsr1;         /* raw register 0 */

  /* reg 1: FAULT_STATUS2 */
  uint8_t  sa_oc;        /* bit 10: sense amplifier overcurrent A */
  uint8_t  sb_oc;        /* bit  9: sense amplifier overcurrent B */
  uint8_t  sc_oc;        /* bit  8: sense amplifier overcurrent C */
  uint8_t  otw;          /* bit  7: overtemperature warning */
  uint8_t  cpuv;         /* bit  6: charge pump undervoltage */
  uint8_t  vgs_ha;       /* bit  5: VGS gate drive fault A high */
  uint8_t  vgs_la;       /* bit  4: VGS gate drive fault A low */
  uint8_t  vgs_hb;       /* bit  3: VGS gate drive fault B high */
  uint8_t  vgs_lb;       /* bit  2: VGS gate drive fault B low */
  uint8_t  vgs_hc;       /* bit  1: VGS gate drive fault C high */
  uint8_t  vgs_lc;       /* bit  0: VGS gate drive fault C low */
  uint16_t fsr2;         /* raw register 1 */

  uint8_t  fault_line;   /* nFAULT GPIO pin (0 = fault active) */
  uint8_t  enabled;      /* driver enabled */
} DRV8323_Status;

/* Configuration ----------------------------------------------------------- */
typedef struct {
  /* DRIVE_CTRL (reg 2) */
  uint8_t         dis_cpuv;            /* disable charge pump UVLO */
  uint8_t         dis_gdf;             /* disable gate drive fault */
  uint8_t         otw_rep;             /* report OTW on nFAULT */
  DRV8323_PwmMode pwm_mode;            /* PWM operating mode */
  uint8_t         pwm_1x_asynchronous; /* 1x PWM asynchronous */
  uint8_t         pwm_1x_dir;          /* 1x PWM direction */

  /* GATE_DRIVE_HS (reg 3) */
  uint16_t idrivep_hs_ma;  /* peak source current (mA) */
  uint16_t idriven_hs_ma;  /* peak sink current   (mA) */

  /* GATE_DRIVE_LS (reg 4) */
  uint8_t  cbc;            /* cycle-by-cycle retry */
  uint16_t tdrive_ns;      /* peak gate drive time (ns) */
  uint16_t idrivep_ls_ma;  /* peak source current (mA) */
  uint16_t idriven_ls_ma;  /* peak sink current   (mA) */

  /* OCP_CTRL (reg 5) */
  uint8_t         tretry;   /* retry time: 0=4 ms, 1=50 us */
  uint16_t        dead_time_ns;
  DRV8323_OcpMode ocp_mode;
  uint8_t         ocp_deg_us;    /* deglitch time (us) */
  uint16_t        vds_lvl_mv;    /* VDS overcurrent level (mV) */

  /* CSA_CTRL (reg 6) */
  uint8_t  csa_fet;    /* 0 = shunt, 1 = FET sense */
  uint8_t  vref_div;   /* 1 = VREF/2 (bidirectional) */
  uint8_t  ls_ref;     /* low-side reference */
  uint8_t  csa_gain;   /* 5, 10, 20, or 40 V/V */
  uint8_t  dis_sen;    /* disable sense OCP */
  uint16_t sen_lvl_mv; /* sense OCP level (mV) */

  /* Platform variant */
  uint8_t is_drv8323;  /* 1 = DRV8323S, 0 = DRV8353S */
} DRV8323_Config;

/* Handle ------------------------------------------------------------------ */
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef      *cs_port;
  uint16_t           cs_pin;
  GPIO_TypeDef      *en_port;
  uint16_t           en_pin;
  GPIO_TypeDef      *flt_port;
  uint16_t           flt_pin;
  GPIO_TypeDef      *hiz_port;   /**< HiZ (high-impedance) control GPIO port */
  uint16_t           hiz_pin;    /**< HiZ control GPIO pin */

  DRV8323_Config config;
  DRV8323_Status status;
} DRV8323_HandleTypeDef;

/* API --------------------------------------------------------------------- */

/** @brief Initialise handle, store GPIO/SPI pointers, set config defaults */
void DRV8323_Init(DRV8323_HandleTypeDef *h,
                  SPI_HandleTypeDef     *hspi,
                  GPIO_TypeDef  *cs_port, uint16_t cs_pin,
                  GPIO_TypeDef  *en_port, uint16_t en_pin,
                  GPIO_TypeDef  *flt_port, uint16_t flt_pin);

/** @brief Enable / disable the driver (EN pin) */
void DRV8323_Enable(DRV8323_HandleTypeDef *h);
void DRV8323_Disable(DRV8323_HandleTypeDef *h);

/** @brief SPI register access */
uint16_t DRV8323_ReadReg(DRV8323_HandleTypeDef *h, uint8_t reg);
void     DRV8323_WriteReg(DRV8323_HandleTypeDef *h, uint8_t reg,
                          uint16_t value);

/** @brief Apply all config registers (2-7) and verify by readback.
 *  @return bitmask of failed registers (bit N = reg N mismatch), 0 = OK */
uint8_t DRV8323_WriteConfig(DRV8323_HandleTypeDef *h);

/** @brief Read FAULT_STATUS1, FAULT_STATUS2, nFAULT pin, populate status */
void    DRV8323_ReadStatus(DRV8323_HandleTypeDef *h);

/** @brief Return nFAULT pin state (0 = fault active) */
uint8_t DRV8323_IsFault(DRV8323_HandleTypeDef *h);

/** @brief Assign HiZ control pin after init (nullable — PowerOn/PowerOff
 *         are no-ops until a pin is set). */
void DRV8323_SetHiZPin(DRV8323_HandleTypeDef *h,
                       GPIO_TypeDef *port, uint16_t pin);

/** @brief Power on: set HiZ HIGH → gate drives enabled (PWM controls FETs).
 *         Safe to call before SetHiZPin (no-op when pin not configured). */
void DRV8323_PowerOn(DRV8323_HandleTypeDef *h);

/** @brief Power off: set HiZ LOW → all gates in high-impedance, motor coasts.
 *         Safe to call before SetHiZPin (no-op when pin not configured). */
void DRV8323_PowerOff(DRV8323_HandleTypeDef *h);

/** @brief HW fault flag — set by EXTI15_10 ISR on nFAULT falling edge.
 *         Main loop clears after reading DRV8323 status and initiating recovery. */
extern volatile uint8_t g_drv8323_hw_fault;

#ifdef __cplusplus
}
#endif

#endif /* DRV8323_H */
