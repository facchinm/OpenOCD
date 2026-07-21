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
    0x60000000,     /* Device Start Address */
    0x10000000,     /* Device Size in Bytes (64 MiB) */
    ROW_SIZE,       /* Programming Page Size */
    0,              /* Reserved, must be 0 */
    0x00,           /* Initial Content of Erased Memory */
    4000,           /* Program Page Timeout 4000 mSec */
    4000,           /* Erase Sector Timeout 4000 mSec */
    {
        {0x00080000, 0x00000000}, /* Set Sector size */
        {SECTOR_END},
    },
};
