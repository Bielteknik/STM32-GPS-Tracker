/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    templates/uart_lld.h
 * @brief   UART Driver subsystem low level driver header template.
 *
 * @addtogroup UART_LLD
 * @{
 */

#ifndef _UART_LLD_H_
#define _UART_LLD_H_

#if CH_HAL_USE_UART || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   UART driver condition flags type.
 */
typedef uint32_t uartflags_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /** @brief End of transmission buffer callback.*/
  uartcb_t                  uc_txend1;
  /** @brief Physical end of transmission callback.*/
  uartcb_t                  uc_txend2;
  /** @brief Receive buffer filled callback.*/
  uartcb_t                  uc_rxend;
  /** @brief Character received while out if the @p UART_RECEIVE state.*/
  uartcb_t                  uc_rxchar;
  /** @brief Receive error callback.*/  
  uartcb_t                  uc_rxerr;
  /* End of the mandatory fields.*/
} UARTConfig;

/**
 * @brief   Structure representing an UART driver.
 */
typedef struct {
  /**
   * @brief Driver state.
   */
  uartstate_t               ud_state;
  /**
   * @brief Transmitter state.
   */
  uarttxstate_t             ud_txstate;
  /**
   * @brief Receiver state.
   */
  uartrxstate_t             ud_rxstate;
  /**
   * @brief Current configuration data.
   */
  const UARTConfig          *ud_config;
  /* End of the mandatory fields.*/
} UARTDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void uart_lld_init(void);
  void uart_lld_start(UARTDriver *uartp);
  void uart_lld_stop(UARTDriver *uartp);
  void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf);
  void uart_lld_stop_send(UARTDriver *uartp);
  void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf);
  void uart_lld_stop_receive(UARTDriver *uartp);
#ifdef __cplusplus
}
#endif

#endif /* CH_HAL_USE_UART */

#endif /* _UART_LLD_H_ */

/** @} */