/*
    ChibiOS/RT - Copyright (C) 2006-2007 Giovanni Di Sirio.

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
 * @file mmc_spi.h
 * @brief MMC over SPI driver header
 * @addtogroup MMC_SPI
 * @{
 */

#ifndef _MMC_SPI_H_
#define _MMC_SPI_H_

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief Number of positive insertion queries before generating the
 *        insertion event.
 */
#if !defined(MMC_POLLING_INTERVAL) || defined(__DOXYGEN__)
#define MMC_POLLING_INTERVAL    10
#endif

/**
 * @brief Interval, in milliseconds, between insertion queries.
 */
#if !defined(MMC_POLLING_DELAY) || defined(__DOXYGEN__)
#define MMC_POLLING_DELAY       10
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Driver state machine possible states.
 */
typedef enum {
  MMC_UNINIT = 0,                           /**< @brief Not initialized.    */
  MMC_STOP = 1,                             /**< @brief Stopped.            */
  MMC_WAIT = 2,                             /**< @brief Waiting card.       */
  MMC_INSERTED = 3,                         /**< @brief Card inserted.      */
  MMC_READY = 4,                            /**< @brief Card ready.         */
  MMC_RUNNING = 5                           /**< @brief Reading or writing. */
} mmcstate_t;

/**
 * @brief Function used to query some hardware status bits.
 *
 * @return The status.
 */
typedef bool_t (*mmcquery_t)(void);

/**
 * @brief Driver configuration structure.
 */
typedef struct {

} MMCConfig;

/**
 * @brief Structure representing a MMC driver.
 */
typedef struct {
  /**
   * @brief Driver state.
   */
  mmcstate_t            mmc_state;
  /**
   * @brief Current configuration data.
   */
  const MMCConfig       *mmc_config;
  /**
   * @brief SPI driver associated to this MMC driver.
   */
  SPIDriver             *mmc_spip;
  /**
   * @brief SPI low speed configuration used during initialization.
   */
  const SPIConfig       *mmc_lscfg;
  /**
   * @brief SPI high speed configuration used during transfers.
   */
  const SPIConfig       *mmc_hscfg;
  /**
   * @brief Write protect status query function.
   */
  mmcquery_t            mmc_is_protected;
  /**
   * @brief Insertion status query function.
   */
  mmcquery_t            mmc_is_inserted;
  /**
   * @brief Card insertion event source.
   */
  EventSource           mmc_inserted_event;
  /**
   * @brief Card removal event source.
   */
  EventSource           mmc_removed_event;
  /**
   * @brief MMC insertion polling timer.
   */
  VirtualTimer          mmc_vt;
  /**
   * @brief Insertion counter.
   */
  uint_fast8_t          mmc_cnt;
} MMCDriver;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void mmcInit(void);
  void mmcObjectInit(MMCDriver *mmcp, SPIDriver *spip,
                     const SPIConfig *lscfg, const SPIConfig *hscfg,
                     mmcquery_t is_protected, mmcquery_t is_inserted);
  void mmcStart(MMCDriver *mmcp, const MMCConfig *config);
  void mmcStop(MMCDriver *mmcp);
#ifdef __cplusplus
}
#endif

#endif /* _MMC_SPI_H_ */

/** @} */