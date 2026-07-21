/*****************************************************************************
 * @file     FlashDev.c
 * @brief    Flash Device Description for New Device Flash
 * @version  V1.0.0
 * @date     10. January 2018
 *****************************************************************************
 *
 * Copyright (c) 2010-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "FlashOS.h" /* FlashOS Structures*/

__attribute__((used)) struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS, /* Driver Version, do not modify! */
    LOADER_TARGET,  /* Device Name */
    EXTSPI,         /* Device Type */
    0x18000000,     /* Device Start Address */
    0x04000000,     /* Device Size in Bytes (64 MiB) */
    512,            /* Programming Page Size */
    0,              /* Reserved, must be 0 */
    0xFF,           /* Initial Content of Erased Memory */
    4000,           /* Program Page Timeout 4000 mSec */
    4000,           /* Erase Sector Timeout 4000 mSec */
    {
#if (SECTOR_ARCH == SECTORS_UNIFORM)
        {0x00040000, 0x00000000}, /* Set Sector size */
#elif (SECTOR_ARCH == SECTORS_HYBRID_B)
        {0x00001000, 0x00000000}, /* Set Sector size */
        {0x00020000, 0x00020000}, /* Set Sector size */
        {0x00040000, 0x00040000}, /* Set Sector size */
#elif (SECTOR_ARCH == SECTORS_HYBRID_T)
        {0x00040000, 0x00000000}, /* Set Sector size */
        {0x00020000, 0x03FC0000}, /* Set Sector size */
        {0x00001000, 0x03FE0000}, /* Set Sector size */
#elif (SECTOR_ARCH == SECTORS_HYBRID_BT)
        {0x00001000, 0x00000000}, /* Set Sector size */
        {0x00030000, 0x00010000}, /* Set Sector size */
        {0x00040000, 0x00040000}, /* Set Sector size */
        {0x00030000, 0x03FC0000}, /* Set Sector size */
        {0x00001000, 0x03FF0000}, /* Set Sector size */
#endif
        {SECTOR_END},
    },
};
