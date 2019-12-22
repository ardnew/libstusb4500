/*******************************************************************************
 *
 *  name: libstusb4500.h
 *  date: Dec 12, 2019
 *  auth: andrew
 *  desc:
 *
 ******************************************************************************/

#ifndef __LIBSTUSB4500_H
#define __LIBSTUSB4500_H

#define __ARDUINO__
//#define __STM32_HAL__

// ----------------------------------------------------------------- includes --

#if defined(__STM32_HAL__)
#if defined (STM32L476xx)
#include "stm32l4xx_hal.h"
#elif defined(STM32F072xx)
#include "stm32f0xx_hal.h"
#elif defined(STM32F401xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32G431xx)
#include "stm32g4xx_hal.h"
#elif defined(STM32G031xx)
#include "stm32g0xx_hal.h"
#endif

#elif defined(__ARDUINO__)
#include <Arduino.h>
#include <Wire.h>
#endif // defined(__STM32_HAL__)

#include "libstusb4500_registers.h"

// ------------------------------------------------------------------ defines --

#define __STUSB4500_I2C_CLOCK_FREQUENCY__ 400000 // Hertz
#define __STUSB4500_I2C_SLAVE_BASE_ADDR__   0x28

#define __STUSB4500_USBPD_REV_3_0_SUPPORT__     1
#define __STUSB4500_USBPD_MESSAGE_QUEUE_SZ__   32
#define __STUSB4500_USBPD_INTERRUPT_QUEUE_SZ__ 32
#define __STUSB4500_NVM_SINK_PDO_COUNT__        3
#define __STUSB4500_NVM_SOURCE_PDO_MAX__       10
#define __STUSB4500_NVM_INVALID_PDO_INDEX__    -1

// ------------------------------------------------------------------- macros --

/* nothing */

// ----------------------------------------------------------- exported types --

typedef struct stusb4500_device stusb4500_device_t;
typedef struct stusb4500_usbpd_status stusb4500_usbpd_status_t;
typedef struct stusb4500_usbpd_state_machine stusb4500_usbpd_state_machine_t;
typedef struct stusb4500_event_handler stusb4500_event_handler_t;

typedef enum
{
  sccNONE = -1,
  sccNotConnected, // = 0
  sccCC1Connected, // = 1
  sccCC2Connected, // = 2
  sccCOUNT         // = 3
}
stusb4500_cable_connected_t;

typedef enum
{
  srwNONE = -1,
  srwDoNotWait, // = 0
  srwWaitReady, // = 1
  srwCOUNT      // = 2
}
stusb4500_reset_wait_t;

typedef enum
{
  sptNONE = -1,
  sptPowerOff, // = 0
  sptPowerOn,  // = 1
  sptCOUNT     // = 2
}
stusb4500_power_toggle_t;

typedef enum
{
  ssfNONE = -1,
  ssfFixed,    // = 0
  ssfVariable, // = 1
  ssfBattery,  // = 2
  ssfCOUNT     // = 3
}
stusb4500_supply_fix_t;

typedef struct
{
  int8_t   number;
  uint16_t voltage_mv;
  uint16_t current_ma;
  uint16_t max_current_ma;
}
stusb4500_pdo_description_t;

typedef void (*stusb4500_event_callback_t)(stusb4500_device_t *);

#if defined(__ARDUINO__)
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;
#endif

typedef HAL_StatusTypeDef stusb4500_status_t;

struct stusb4500_usbpd_status
{
    uint8_t                                    hw_reset;
    STUSB_GEN1S_HW_FAULT_STATUS_RegTypeDef     hw_fault_status;     // 8-bit
    STUSB_GEN1S_MONITORING_STATUS_RegTypeDef   monitoring_status;   // 8-bit
    STUSB_GEN1S_CC_DETECTION_STATUS_RegTypeDef cc_detection_status; // 8-bit
    STUSB_GEN1S_CC_STATUS_RegTypeDef           cc_status;           // 8-bit
    STUSB_GEN1S_PRT_STATUS_RegTypeDef          prt_status;          // 8-bit
    STUSB_GEN1S_PHY_STATUS_RegTypeDef          phy_status;          // 8-bit

    uint8_t pdo_snk_count;
    USB_PD_SNK_PDO_TypeDef pdo_snk[__STUSB4500_NVM_SINK_PDO_COUNT__];

    uint8_t pdo_src_count;
    USB_PD_SRC_PDO_TypeDef pdo_src[__STUSB4500_NVM_SOURCE_PDO_MAX__];

    STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef rdo_neg;
};

struct stusb4500_usbpd_state_machine
{
  volatile uint8_t alert_received;
  volatile uint8_t attach_received;

  uint16_t irq_received;
  uint16_t irq_hard_reset;
  uint16_t attach_transition;
  uint16_t src_pdo_received;
  uint16_t src_pdo_requesting;
  uint16_t psrdy_received;
  uint16_t msg_received;
  uint16_t msg_accept;
  uint16_t msg_reject;
  uint16_t msg_goodcrc;

  uint8_t msg[__STUSB4500_USBPD_MESSAGE_QUEUE_SZ__];
  uint8_t msg_head;
  uint8_t msg_tail;

  uint8_t irq[__STUSB4500_USBPD_INTERRUPT_QUEUE_SZ__];
  uint8_t irq_head;
  uint8_t irq_tail;
};

struct stusb4500_event_handler
{
  stusb4500_event_callback_t cable_attached;
  stusb4500_event_callback_t cable_detached;
  stusb4500_event_callback_t source_capabilities_received;
};

struct stusb4500_device
{
#if defined(__STM32_HAL__)
  I2C_HandleTypeDef *i2c_hal;
  GPIO_TypeDef      *reset_port;
#elif defined(__ARDUINO__)
  TwoWire           *i2c_hal;
#endif
  uint8_t            i2c_slave_addr; // real address, NOT shifted
  uint16_t           reset_pin;

  stusb4500_usbpd_status_t        usbpd_status;
  stusb4500_usbpd_state_machine_t usbpd_state_machine;
  stusb4500_event_handler_t       event_handler;
};

// ------------------------------------------------------- exported variables --

/* nothing */

// ------------------------------------------------------- exported functions --

#ifdef __cplusplus
extern "C" {
#endif

stusb4500_device_t *stusb4500_device_new(
#if defined(__STM32_HAL__)
    I2C_HandleTypeDef *i2c_hal,
    uint8_t            i2c_slave_addr,
    GPIO_TypeDef      *reset_port,
#elif defined(__ARDUINO__)
    TwoWire           *i2c_hal,
    uint8_t            i2c_slave_addr,
#endif
    uint16_t           reset_pin);

stusb4500_status_t stusb4500_device_init(stusb4500_device_t *dev);
stusb4500_status_t stusb4500_ready(stusb4500_device_t *dev);

void stusb4500_wait_until_ready(stusb4500_device_t *dev);
void stusb4500_hard_reset(stusb4500_device_t *dev, stusb4500_reset_wait_t wait);
void stusb4500_soft_reset(stusb4500_device_t *dev, stusb4500_reset_wait_t wait);
void stusb4500_power_toggle(stusb4500_device_t *dev,
    stusb4500_power_toggle_t toggle, stusb4500_reset_wait_t wait);

// main loop event
void stusb4500_process_events(stusb4500_device_t *dev);

// interrupt handlers. careful not to perform any I2C comms here as they depend
// on system tick to timeout, and arduino external interrupt priorities are
// higher than the tick interrupt, which cannot be changed. STM32 doesn't have
// this limitation.
void stusb4500_alert(stusb4500_device_t *dev);
void stusb4500_attach(stusb4500_device_t *dev);

stusb4500_cable_connected_t stusb4500_cable_connected(stusb4500_device_t *dev);

stusb4500_status_t stusb4500_get_source_capabilities(stusb4500_device_t *dev);
stusb4500_status_t stusb4500_get_sink_capabilities(stusb4500_device_t *dev);
stusb4500_pdo_description_t stusb4500_power_requested(stusb4500_device_t *dev);
stusb4500_status_t stusb4500_set_power(stusb4500_device_t *dev,
    uint32_t voltage_mv, uint32_t current_ma);
stusb4500_status_t stusb4500_select_power_usb_default(stusb4500_device_t *dev);

void stusb4500_set_cable_attached(stusb4500_device_t *dev,
    stusb4500_event_callback_t callback);
void stusb4500_set_cable_detached(stusb4500_device_t *dev,
    stusb4500_event_callback_t callback);
void stusb4500_set_source_capabilities_received(stusb4500_device_t *dev,
    stusb4500_event_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __LIBSTUSB4500_H */
