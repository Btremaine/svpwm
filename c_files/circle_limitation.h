/**
  ******************************************************************************
  * @file    circle_limitation.h
  * @author  Brian Tremaine
  * @brief   This file contains all definitions and functions prototypes for the
  *          Circle Limitation
  ******************************************************************************
  * @attention
  *
  * license
  ******************************************************************************
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CIRCLELIMITATION_H
#define __CIRCLELIMITATION_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* MMI Table Motor 1 MAX_MODULATION_94_PER_CENT */
#define START_INDEX 56
#define MAX_MODULE 30800
#define MMITABLE  {\
32607,32293,31988,31691,31546,31261,30984,30714,30451,30322,\
30069,29822,29581,29346,29231,29004,28782,28565,28353,28249,\
28044,27843,27647,27455,27360,27174,26991,26812,26724,26550,\
26380,26213,26049,25968,25808,25652,25498,25347,25272,25125,\
24981,24839,24699,24630,24494,24360,24228,24098,24034,23908,\
23783,23660,23600,23480,23361,23245,23131,23074,22962,22851,\
22742,22635,22582,22477,22373,22271,22170,22120,22021,21924,\
21827,21732\
}

typedef struct
{
  int16_t q;
  int16_t d;
} qd_t;

typedef struct
{
  uint16_t MaxModule;               /**<  Circle limitation maximum allowed module */
  uint16_t MaxVd;                   /**<  Circle limitation maximum allowed module */
  uint16_t Circle_limit_table[87];  /**<  Circle limitation table */
  uint8_t  Start_index;             /**<  Circle limitation table indexing
                                         start */
} CircleLimitation_Handle_t;

CircleLimitation_Handle_t CircleLimitationM1;

/* Exported functions ------------------------------------------------------- */

qd_t Circle_Limitation( CircleLimitation_Handle_t * pHandle, qd_t Vqd );

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __Circle Limitation_H */

/* *****END OF FILE****/

