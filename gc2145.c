// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sccb.h"
#include "gc2145.h"
#include "gc2145_regs.h"
#include "gc2145_settings.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "gc2145";
#endif

#define H8(v) ((v)>>8)
#define L8(v) ((v)&0xff)
static const uint8_t b0462_default_regs[][2] = {
    {0xfe, 0xf0},
    {0xfe, 0xf0},
    {0xfe, 0xf0},
    {0xfc, 0x06},
    {0xf6, 0x00},
    {0xf7, 0x1d},
    {0xf8, 0x85},
    {0xfa, 0x00},
    {0xf9, 0xfe},
    {0xf2, 0x00},
    /////////////////////////////////////////////////
    //////////////////ISP reg//////////////////////
    ////////////////////////////////////////////////////
    {0xfe, 0x00},
    {0x03, 0x04},
    {0x04, 0xe2},

    {0x09, 0x00},   // row start
    {0x0a, 0x00},

    {0x0b, 0x00},   // col start
    {0x0c, 0x00},

    {0x0d, 0x04},   // Window height
    {0x0e, 0xc0},

    {0x0f, 0x06},   // Window width
    {0x10, 0x52},

    {0x99, 0x11},   // Subsample
    {0x9a, 0x0E},   // Subsample mode

    {0x12, 0x2e},   //
    {0x17, 0x14},   // Analog Mode 1 (vflip/mirror[1:0])
    {0x18, 0x22},   // Analog Mode 2
    {0x19, 0x0e},
    {0x1a, 0x01},
    {0x1b, 0x4b},
    {0x1c, 0x07},
    {0x1d, 0x10},
    {0x1e, 0x88},
    {0x1f, 0x78},
    {0x20, 0x03},
    {0x21, 0x40},
    {0x22, 0xa0},
    {0x24, 0x16},
    {0x25, 0x01},
    {0x26, 0x10},
    {0x2d, 0x60},
    {0x30, 0x01},
    {0x31, 0x90},
    {0x33, 0x06},
    {0x34, 0x01},
    {0x80, 0x7f},
    {0x81, 0x26},
    {0x82, 0xfa},
    {0x83, 0x00},
    {0x84, 0x06},   //RGB565
    {0x86, 0x23},
    {0x88, 0x03},
    {0x89, 0x03},
    {0x85, 0x08},
    {0x8a, 0x00},
    {0x8b, 0x00},
    {0xb0, 0x55},
    {0xc3, 0x00},
    {0xc4, 0x80},
    {0xc5, 0x90},
    {0xc6, 0x3b},
    {0xc7, 0x46},
    {0xec, 0x06},
    {0xed, 0x04},
    {0xee, 0x60},
    {0xef, 0x90},
    {0xb6, 0x01},

    {0x90, 0x01},   // Enable crop
    {0x91, 0x00},   // Y offset
    {0x92, 0x00},
    {0x93, 0x00},   // X offset
    {0x94, 0x00},
    {0x95, 0x02},   // Window height
    {0x96, 0x58},
    {0x97, 0x03},   // Window width
    {0x98, 0x20},
    {0x99, 0x22},   // Subsample
    {0x9a, 0x0E},   // Subsample mode

    {0x9b, 0x00},
    {0x9c, 0x00},
    {0x9d, 0x00},
    {0x9e, 0x00},
    {0x9f, 0x00},
    {0xa0, 0x00},
    {0xa1, 0x00},
    {0xa2, 0x00},
    /////////////////////////////////////////
    /////////// BLK ////////////////////////
    /////////////////////////////////////////
    {0xfe, 0x00},
    {0x40, 0x42},
    {0x41, 0x00},
    {0x43, 0x5b},
    {0x5e, 0x00},
    {0x5f, 0x00},
    {0x60, 0x00},
    {0x61, 0x00},
    {0x62, 0x00},
    {0x63, 0x00},
    {0x64, 0x00},
    {0x65, 0x00},
    {0x66, 0x20},
    {0x67, 0x20},
    {0x68, 0x20},
    {0x69, 0x20},
    {0x76, 0x00},
    {0x6a, 0x08},
    {0x6b, 0x08},
    {0x6c, 0x08},
    {0x6d, 0x08},
    {0x6e, 0x08},
    {0x6f, 0x08},
    {0x70, 0x08},
    {0x71, 0x08},
    {0x76, 0x00},
    {0x72, 0xf0},
    {0x7e, 0x3c},
    {0x7f, 0x00},
    {0xfe, 0x02},
    {0x48, 0x15},
    {0x49, 0x00},
    {0x4b, 0x0b},
    {0xfe, 0x00},
    ////////////////////////////////////////
    /////////// AEC ////////////////////////
    ////////////////////////////////////////
    {0xfe, 0x01},
    {0x01, 0x04},
    {0x02, 0xc0},
    {0x03, 0x04},
    {0x04, 0x90},
    {0x05, 0x30},
    {0x06, 0x90},
    {0x07, 0x30},
    {0x08, 0x80},
    {0x09, 0x00},
    {0x0a, 0x82},
    {0x0b, 0x11},
    {0x0c, 0x10},
    {0x11, 0x10},
    {0x13, 0x68}, //7b->68 bob
    {0x17, 0x00},
    {0x1c, 0x11},
    {0x1e, 0x61},
    {0x1f, 0x35},
    {0x20, 0x40},
    {0x22, 0x40},
    {0x23, 0x20},
    {0xfe, 0x02},
    {0x0f, 0x04},
    {0xfe, 0x01},
    {0x12, 0x30}, //35
    {0x15, 0xb0},
    {0x10, 0x31},
    {0x3e, 0x28},
    {0x3f, 0xb0},
    {0x40, 0x90},
    {0x41, 0x0f},
    /////////////////////////////
    //////// INTPEE /////////////
    /////////////////////////////
    {0xfe, 0x02},
    {0x90, 0x6c},
    {0x91, 0x03},
    {0x92, 0xcb},
    {0x94, 0x33},
    {0x95, 0x84},
    {0x97, 0x65}, // 54->65 bob
    {0xa2, 0x11},
    {0xfe, 0x00},
    /////////////////////////////
    //////// DNDD///////////////
    /////////////////////////////
    {0xfe, 0x02},
    {0x80, 0xc1},
    {0x81, 0x08},
    {0x82, 0x05}, //05
    {0x83, 0x08}, //08
    {0x84, 0x0a},
    {0x86, 0xf0},
    {0x87, 0x50},
    {0x88, 0x15},
    {0x89, 0xb0},
    {0x8a, 0x30},
    {0x8b, 0x10},
    /////////////////////////////////////////
    /////////// ASDE ////////////////////////
    /////////////////////////////////////////
    {0xfe, 0x01},
    {0x21, 0x04},
    {0xfe, 0x02},
    {0xa3, 0x50},
    {0xa4, 0x20},
    {0xa5, 0x40},
    {0xa6, 0x80},
    {0xab, 0x40},
    {0xae, 0x0c},
    {0xb3, 0x46},
    {0xb4, 0x64},
    {0xb6, 0x38},
    {0xb7, 0x01}, //01
    {0xb9, 0x2b}, //2b
    {0x3c, 0x04}, //04
    {0x3d, 0x15}, //15
    {0x4b, 0x06}, //06
    {0x4c, 0x20},
    {0xfe, 0x00},
    /////////////////////////////////////////
    /////////// GAMMA   ////////////////////////
    /////////////////////////////////////////
    ///////////////////gamma1////////////////////
    {0xfe, 0x02},
    {0x10, 0x09},
    {0x11, 0x0d},
    {0x12, 0x13},
    {0x13, 0x19},
    {0x14, 0x27},
    {0x15, 0x37},
    {0x16, 0x45},
    {0x17, 0x53},
    {0x18, 0x69},
    {0x19, 0x7d},
    {0x1a, 0x8f},
    {0x1b, 0x9d},
    {0x1c, 0xa9},
    {0x1d, 0xbd},
    {0x1e, 0xcd},
    {0x1f, 0xd9},
    {0x20, 0xe3},
    {0x21, 0xea},
    {0x22, 0xef},
    {0x23, 0xf5},
    {0x24, 0xf9},
    {0x25, 0xff},
    {0xfe, 0x00},
    {0xc6, 0x20},
    {0xc7, 0x2b},
    ///////////////////gamma2////////////////////
    {0xfe, 0x02},
    {0x26, 0x0f},
    {0x27, 0x14},
    {0x28, 0x19},
    {0x29, 0x1e},
    {0x2a, 0x27},
    {0x2b, 0x33},
    {0x2c, 0x3b},
    {0x2d, 0x45},
    {0x2e, 0x59},
    {0x2f, 0x69},
    {0x30, 0x7c},
    {0x31, 0x89},
    {0x32, 0x98},
    {0x33, 0xae},
    {0x34, 0xc0},
    {0x35, 0xcf},
    {0x36, 0xda},
    {0x37, 0xe2},
    {0x38, 0xe9},
    {0x39, 0xf3},
    {0x3a, 0xf9},
    {0x3b, 0xff},
    ///////////////////////////////////////////////
    ///////////YCP ///////////////////////
    ///////////////////////////////////////////////
    {0xfe, 0x02},
    {0xd1, 0x32}, //32->2d
    {0xd2, 0x32}, //32->2d bob
    {0xd3, 0x40},
    {0xd6, 0xf0},
    {0xd7, 0x10},
    {0xd8, 0xda},
    {0xdd, 0x14},
    {0xde, 0x86},
    {0xed, 0x80}, //80
    {0xee, 0x00}, //00
    {0xef, 0x3f},
    {0xd8, 0xd8},
    ///////////////////abs/////////////////
    {0xfe, 0x01},
    {0x9f, 0x40},
    /////////////////////////////////////////////
    //////////////////////// LSC ///////////////
    //////////////////////////////////////////
    {0xfe, 0x01},
    {0xc2, 0x14},
    {0xc3, 0x0d},
    {0xc4, 0x0c},
    {0xc8, 0x15},
    {0xc9, 0x0d},
    {0xca, 0x0a},
    {0xbc, 0x24},
    {0xbd, 0x10},
    {0xbe, 0x0b},
    {0xb6, 0x25},
    {0xb7, 0x16},
    {0xb8, 0x15},
    {0xc5, 0x00},
    {0xc6, 0x00},
    {0xc7, 0x00},
    {0xcb, 0x00},
    {0xcc, 0x00},
    {0xcd, 0x00},
    {0xbf, 0x07},
    {0xc0, 0x00},
    {0xc1, 0x00},
    {0xb9, 0x00},
    {0xba, 0x00},
    {0xbb, 0x00},
    {0xaa, 0x01},
    {0xab, 0x01},
    {0xac, 0x00},
    {0xad, 0x05},
    {0xae, 0x06},
    {0xaf, 0x0e},
    {0xb0, 0x0b},
    {0xb1, 0x07},
    {0xb2, 0x06},
    {0xb3, 0x17},
    {0xb4, 0x0e},
    {0xb5, 0x0e},
    {0xd0, 0x09},
    {0xd1, 0x00},
    {0xd2, 0x00},
    {0xd6, 0x08},
    {0xd7, 0x00},
    {0xd8, 0x00},
    {0xd9, 0x00},
    {0xda, 0x00},
    {0xdb, 0x00},
    {0xd3, 0x0a},
    {0xd4, 0x00},
    {0xd5, 0x00},
    {0xa4, 0x00},
    {0xa5, 0x00},
    {0xa6, 0x77},
    {0xa7, 0x77},
    {0xa8, 0x77},
    {0xa9, 0x77},
    {0xa1, 0x80},
    {0xa2, 0x80},

    {0xfe, 0x01},
    {0xdf, 0x0d},
    {0xdc, 0x25},
    {0xdd, 0x30},
    {0xe0, 0x77},
    {0xe1, 0x80},
    {0xe2, 0x77},
    {0xe3, 0x90},
    {0xe6, 0x90},
    {0xe7, 0xa0},
    {0xe8, 0x90},
    {0xe9, 0xa0},
    {0xfe, 0x00},
    ///////////////////////////////////////////////
    /////////// AWB////////////////////////
    ///////////////////////////////////////////////
    {0xfe, 0x01},
    {0x4f, 0x00},
    {0x4f, 0x00},
    {0x4b, 0x01},
    {0x4f, 0x00},

    {0x4c, 0x01}, // D75
    {0x4d, 0x71},
    {0x4e, 0x01},
    {0x4c, 0x01},
    {0x4d, 0x91},
    {0x4e, 0x01},
    {0x4c, 0x01},
    {0x4d, 0x70},
    {0x4e, 0x01},
    {0x4c, 0x01}, // D65
    {0x4d, 0x90},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xb0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x8f},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x6f},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xaf},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xd0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xf0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xcf},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xef},
    {0x4e, 0x02},
    {0x4c, 0x01}, //D50
    {0x4d, 0x6e},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8e},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xae},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xce},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xad},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcd},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xac},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcc},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcb},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xab},
    {0x4e, 0x03},
    {0x4c, 0x01}, //CWF
    {0x4d, 0x8a},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xaa},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xca},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xca},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xc9},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0x8a},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0x89},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xa9},
    {0x4e, 0x04},
    {0x4c, 0x02}, //tl84
    {0x4d, 0x0b},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x0a},
    {0x4e, 0x05},
    {0x4c, 0x01},
    {0x4d, 0xeb},
    {0x4e, 0x05},
    {0x4c, 0x01},
    {0x4d, 0xea},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x09},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x29},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x2a},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x4a},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x8a},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x49},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x69},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x89},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0xa9},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x48},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x68},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x69},
    {0x4e, 0x06},
    {0x4c, 0x02}, //H
    {0x4d, 0xca},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc9},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe9},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x09},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc8},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe8},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xa7},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc7},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe7},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x07},
    {0x4e, 0x07},

    {0x4f, 0x01},
    {0x50, 0x80},
    {0x51, 0xa8},
    {0x52, 0x47},
    {0x53, 0x38},
    {0x54, 0xc7},
    {0x56, 0x0e},
    {0x58, 0x08},
    {0x5b, 0x00},
    {0x5c, 0x74},
    {0x5d, 0x8b},
    {0x61, 0xdb},
    {0x62, 0xb8},
    {0x63, 0x86},
    {0x64, 0xc0},
    {0x65, 0x04},
    {0x67, 0xa8},
    {0x68, 0xb0},
    {0x69, 0x00},
    {0x6a, 0xa8},
    {0x6b, 0xb0},
    {0x6c, 0xaf},
    {0x6d, 0x8b},
    {0x6e, 0x50},
    {0x6f, 0x18},
    {0x73, 0xf0},
    {0x70, 0x0d},
    {0x71, 0x60},
    {0x72, 0x80},
    {0x74, 0x01},
    {0x75, 0x01},
    {0x7f, 0x0c},
    {0x76, 0x70},
    {0x77, 0x58},
    {0x78, 0xa0},
    {0x79, 0x5e},
    {0x7a, 0x54},
    {0x7b, 0x58},
    {0xfe, 0x00},
    //////////////////////////////////////////
    ///////////CC////////////////////////
    //////////////////////////////////////////
    {0xfe, 0x02},
    {0xc0, 0x01},
    {0xc1, 0x44},
    {0xc2, 0xfd},
    {0xc3, 0x04},
    {0xc4, 0xF0},
    {0xc5, 0x48},
    {0xc6, 0xfd},
    {0xc7, 0x46},
    {0xc8, 0xfd},
    {0xc9, 0x02},
    {0xca, 0xe0},
    {0xcb, 0x45},
    {0xcc, 0xec},
    {0xcd, 0x48},
    {0xce, 0xf0},
    {0xcf, 0xf0},
    {0xe3, 0x0c},
    {0xe4, 0x4b},
    {0xe5, 0xe0},
    //////////////////////////////////////////
    ///////////ABS ////////////////////
    //////////////////////////////////////////
    {0xfe, 0x01},
    {0x9f, 0x40},
    {0xfe, 0x00},

    //////////////////////////////////////
    ///////////  OUTPUT   ////////////////
    //////////////////////////////////////
    {0xfe, 0x00},
    {0xf2, 0x0f},

    ///////////////dark sun////////////////////
    {0xfe, 0x02},
    {0x40, 0xbf},
    {0x46, 0xcf},
    {0xfe, 0x00},

    //////////////frame rate control/////////
    {0xfe, 0x00},
    {0x05, 0x01},   // HBLANK
    {0x06, 0x1C},
    {0x07, 0x00},   // VBLANK
    {0x08, 0x32},
    {0x11, 0x00},   // SH Delay
    {0x12, 0x1D},
    {0x13, 0x00},   // St
    {0x14, 0x00},   // Et

    {0xfe, 0x01},
    {0x3c, 0x00},
    {0x3d, 0x04},
    {0xfe, 0x00},
    {0x00, 0x00},
};



//#define REG_DEBUG_ON

static int read_reg(uint8_t slv_addr, const uint16_t reg)
{
    int ret = SCCB_Read(slv_addr, reg);
#ifdef REG_DEBUG_ON
    if (ret < 0) {
        ESP_LOGE(TAG, "READ REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int write_reg(uint8_t slv_addr, const uint16_t reg, uint8_t value)
{
    int ret = -1;

    /*
     * B0462/GC2145 on ESP32-S3 sometimes NACKs one transfer while the
     * sensor is switching pages or starting its ISP. Do not fail on the
     * first error. Retry with a slightly longer delay.
     */
    for (int i = 0; i < 10; i++) {
        ret = SCCB_Write(slv_addr, reg, value);

        if (ret == 0) {
            vTaskDelay(pdMS_TO_TICKS(3));
            return 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGE(TAG,
             "GC2145 write failed after retries: reg=0x%04x value=0x%02x ret=%d",
             reg, value, ret);

    return ret;
}
static int check_reg_mask(uint8_t slv_addr, uint16_t reg, uint8_t mask)
{
    return (read_reg(slv_addr, reg) & mask) == mask;
}

static int set_reg_bits(uint8_t slv_addr, uint16_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;
    ret = read_reg(slv_addr, reg);
    if (ret < 0) {
        return ret;
    }
    c_value = ret;
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = write_reg(slv_addr, reg, new_value);
    return ret;
}

static int write_regs(uint8_t slv_addr, const uint16_t (*regs)[2])
{
    int i = 0, ret = 0;
    while (!ret && regs[i][0] != REGLIST_TAIL) {
        if (regs[i][0] == REG_DLY) {
            vTaskDelay(regs[i][1] / portTICK_PERIOD_MS);
        } else {
            ret = write_reg(slv_addr, regs[i][0], regs[i][1]);
        }
        i++;
    }
    return ret;
}

static void print_regs(uint8_t slv_addr)
{
#ifdef DEBUG_PRINT_REG
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "REG list look ======================");
    for (size_t i = 0xf0; i <= 0xfe; i++) {
        ESP_LOGI(TAG, "reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
    ESP_LOGI(TAG, "\npage 0 ===");
    write_reg(slv_addr, 0xfe, 0x00); // page 0
    for (size_t i = 0x03; i <= 0x24; i++) {
        ESP_LOGI(TAG, "p0 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
    for (size_t i = 0x80; i <= 0xa2; i++) {
        ESP_LOGI(TAG, "p0 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
    ESP_LOGI(TAG, "\npage 3 ===");
    write_reg(slv_addr, 0xfe, 0x03); // page 3
    for (size_t i = 0x01; i <= 0x43; i++) {
        ESP_LOGI(TAG, "p3 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
#endif
}
static int b0462_apply_default_regs(sensor_t *sensor)
{
    int ret = 0;

    ESP_LOGI(TAG, "B0462: applying Arduino GC2145 default_regs");

    for (int i = 0; b0462_default_regs[i][0] != 0x00; i++) {
        ret = write_reg(sensor->slv_addr,
                        b0462_default_regs[i][0],
                        b0462_default_regs[i][1]);

        if (ret != 0) {
            ESP_LOGE(TAG,
                     "B0462 default reg failed: reg=0x%02x val=0x%02x ret=%d",
                     b0462_default_regs[i][0],
                     b0462_default_regs[i][1],
                     ret);
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGI(TAG, "B0462: Arduino default_regs applied OK");
    return 0;
}

static int reset(sensor_t *sensor)
{
    int ret = 0;

    ret = b0462_apply_default_regs(sensor);
    if (ret != 0) {
        ESP_LOGE(TAG, "B0462 Arduino default register sequence failed");
        return ret;
    }

    /* Keep page0 selected and explicitly enable the DVP/RGB565 output path. */
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    write_reg(sensor->slv_addr, 0x84, 0x06);   // RGB565
    write_reg(sensor->slv_addr, 0x90, 0x01);   // crop/output window enable
    write_reg(sensor->slv_addr, 0xf2, 0x0f);   // output enable from Arduino table

    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "B0462 reset/register load complete");

    return 0;
}
static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    /*
     * Do not use read-modify-write here. SCCB reads are the unstable part
     * on this B0462 + ESP32-S3 setup. The Arduino/B0462 register table
     * already selects RGB565 using page0 register 0x84 = 0x06.
     */
    switch (pixformat) {
    case PIXFORMAT_RGB565:
        sensor->pixformat = pixformat;
        write_reg(sensor->slv_addr, 0xfe, 0x00);
        write_reg(sensor->slv_addr, P0_OUTPUT_FORMAT, 0x06);
        ESP_LOGI(TAG, "B0462 pixformat RGB565 applied direct");
        return 0;

    case PIXFORMAT_YUV422:
        sensor->pixformat = pixformat;
        write_reg(sensor->slv_addr, 0xfe, 0x00);
        write_reg(sensor->slv_addr, P0_OUTPUT_FORMAT, 0x02);
        ESP_LOGI(TAG, "B0462 pixformat YUV422 applied direct");
        return 0;

    default:
        ESP_LOGW(TAG, "B0462 unsupported format %d; use RGB565 first", pixformat);
        return -1;
    }
}
static int b0462_set_window(sensor_t *sensor,
                            uint16_t reg,
                            uint16_t x,
                            uint16_t y,
                            uint16_t w,
                            uint16_t h)
{
    int ret = 0;
    uint8_t slv_addr = sensor->slv_addr;

    /*
     * Arduino GC2145 style setWindow:
     * page 0, then Y, X, H, W
     */
    ret |= write_reg(slv_addr, 0xFE, 0x00);

    ret |= write_reg(slv_addr, reg++, (y >> 8) & 0xFF);
    ret |= write_reg(slv_addr, reg++, y & 0xFF);

    ret |= write_reg(slv_addr, reg++, (x >> 8) & 0xFF);
    ret |= write_reg(slv_addr, reg++, x & 0xFF);

    ret |= write_reg(slv_addr, reg++, (h >> 8) & 0xFF);
    ret |= write_reg(slv_addr, reg++, h & 0xFF);

    ret |= write_reg(slv_addr, reg++, (w >> 8) & 0xFF);
    ret |= write_reg(slv_addr, reg++, w & 0xFF);

    return ret;
}

static int b0462_arduino_style_set_framesize(sensor_t *sensor, framesize_t framesize)
{
    uint16_t w = 0;
    uint16_t h = 0;
    uint16_t win_w = 0;
    uint16_t win_h = 0;

    switch (framesize) {
        case FRAMESIZE_QQVGA:   // 160x120
            w = 160;
            h = 120;
            win_w = 640;
            win_h = 480;
            break;

        case FRAMESIZE_QVGA:    // 320x240
            w = 320;
            h = 240;
            win_w = 960;
            win_h = 720;
            break;

        case FRAMESIZE_VGA:     // 640x480
            w = 640;
            h = 480;
            win_w = 1280;
            win_h = 960;
            break;

        default:
            ESP_LOGE(TAG, "B0462 patch: unsupported framesize %d", framesize);
            return -1;
    }

    uint16_t win_x = (1600 - win_w) / 2;
    uint16_t win_y = (1200 - win_h) / 2;

    uint16_t c_ratio = win_w / w;
    uint16_t r_ratio = win_h / h;

    int ret = 0;
    uint8_t slv_addr = sensor->slv_addr;

    ESP_LOGI(TAG,
             "B0462 Arduino-style frame: out=%dx%d win=%dx%d ratio=%d:%d",
             w, h, win_w, win_h, c_ratio, r_ratio);

    sensor->status.framesize = framesize;

    /*
     * Arduino-style sequence:
     * 1. Sensor readout window at 0x09
     * 2. Crop/output window at 0x91
     * 3. Enable crop using 0x90
     * 4. Subsample using 0x99 and 0x9A
     */
    ret |= b0462_set_window(sensor, 0x09, win_x, win_y, win_w + 16, win_h + 8);
    ret |= b0462_set_window(sensor, 0x91, 0, 0, w, h);

    ret |= write_reg(slv_addr, 0x90, 0x01);
    ret |= write_reg(slv_addr, 0x99, ((r_ratio << 4) | c_ratio));
    ret |= write_reg(slv_addr, 0x9A, 0x0E);

    if (ret != 0) {
        ESP_LOGE(TAG, "B0462 Arduino-style framesize failed ret=%d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "B0462 Arduino-style framesize applied OK");
    return 0;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    /*
     * The default B0462 table configures SVGA 800x600 RGB565.
     * For first bring-up keep SVGA as-is. For smaller test modes use
     * the Arduino-style window/subsample sequence.
     */
    if (framesize == FRAMESIZE_SVGA) {
        sensor->status.framesize = framesize;
        ESP_LOGI(TAG, "B0462 framesize SVGA: using default register table output");
        return 0;
    }

    if (framesize == FRAMESIZE_QQVGA ||
        framesize == FRAMESIZE_QVGA  ||
        framesize == FRAMESIZE_VGA) {
        return b0462_arduino_style_set_framesize(sensor, framesize);
    }

    ESP_LOGW(TAG, "B0462 unsupported framesize %d; falling back to SVGA default", framesize);
    sensor->status.framesize = FRAMESIZE_SVGA;
    return 0;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.hmirror = enable;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, P0_ANALOG_MODE1, 0, 0x01, enable != 0);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set h-mirror to: %d", enable);
    }
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.vflip = enable;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, P0_ANALOG_MODE1, 1, 0x01, enable != 0);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set v-flip to: %d", enable);
    }
    return ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;

    sensor->status.colorbar = enable;

    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);

    if (enable) {
        ret |= set_reg_bits(sensor->slv_addr, P0_DEBUG_MODE3, 3, 0x01, 1);
        ESP_LOGI(TAG, "GC2145 colorbar enabled");
    } else {
        ret |= set_reg_bits(sensor->slv_addr, P0_DEBUG_MODE3, 3, 0x01, 0);
        ESP_LOGI(TAG, "GC2145 colorbar disabled");
    }

    return ret;
}

static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = 0;
    if (mask > 0xFF) {
        ESP_LOGE(TAG, "mask should not more than 0xff");
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if (ret > 0) {
        ret &= mask;
    }
    return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    int ret = 0;
    if (mask > 0xFF) {
        ESP_LOGE(TAG, "mask should not more than 0xff");
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if (ret < 0) {
        return ret;
    }
    value = (ret & ~mask) | (value & mask);

    if (mask > 0xFF) {

    } else {
        ret = write_reg(sensor->slv_addr, reg, value);
    }
    return ret;
}

static int init_status(sensor_t *sensor)
{
    /*
     * Avoid SCCB read-back during first bring-up. The log already shows
     * SCCB reads can fail after init, and these status values are not
     * required for frame capture.
     */
    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    sensor->status.sharpness = 0;
    sensor->status.denoise = 0;
    sensor->status.ae_level = 0;
    sensor->status.gainceiling = 0;
    sensor->status.awb = 0;
    sensor->status.dcw = 0;
    sensor->status.agc = 0;
    sensor->status.aec = 0;
    sensor->status.hmirror = 0;
    sensor->status.vflip = 0;
    sensor->status.colorbar = 0;
    sensor->status.bpc = 0;
    sensor->status.wpc = 0;
    sensor->status.raw_gma = 0;
    sensor->status.lenc = 0;
    sensor->status.quality = 0;
    sensor->status.special_effect = 0;
    sensor->status.wb_mode = 0;
    sensor->status.awb_gain = 0;
    sensor->status.agc_gain = 0;
    sensor->status.aec_value = 0;
    sensor->status.aec2 = 0;

    ESP_LOGI(TAG, "B0462 init_status done without SCCB read-back");
    return 0;
}

static int set_dummy(sensor_t *sensor, int val)
{
    ESP_LOGW(TAG, "Unsupported");
    return -1;
}
static int set_gainceiling_dummy(sensor_t *sensor, gainceiling_t val)
{
    ESP_LOGW(TAG, "Unsupported");
    return -1;
}

int esp32_camera_gc2145_detect(int slv_addr, sensor_id_t *id)
{
    if (GC2145_SCCB_ADDR != slv_addr) {
        return 0;
    }

    int midl = SCCB_Read(slv_addr, CHIP_ID_LOW);
    int midh = SCCB_Read(slv_addr, CHIP_ID_HIGH);

    if (midl < 0 || midh < 0) {
        ESP_LOGE(TAG, "GC2145 chip-id SCCB read failed: MIDH=%d MIDL=%d", midh, midl);
        return 0;
    }

    uint16_t pid = ((uint16_t)(midh & 0xff) << 8) | (uint8_t)(midl & 0xff);

    if (pid == GC2145_PID) {
        id->PID = pid;
        id->MIDH = midh & 0xff;
        id->MIDL = midl & 0xff;
        ESP_LOGI(TAG, "GC2145 detected: PID=0x%04x MIDH=0x%02x MIDL=0x%02x",
                 pid, id->MIDH, id->MIDL);
        return pid;
    }

    ESP_LOGI(TAG, "GC2145 PID mismatch: PID=0x%04x MIDH=0x%02x MIDL=0x%02x",
             pid, midh & 0xff, midl & 0xff);
    return 0;
}

int esp32_camera_gc2145_init(sensor_t *sensor)
{
    sensor->init_status = init_status;
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast = set_dummy;
    sensor->set_brightness = set_dummy;
    sensor->set_saturation = set_dummy;
    sensor->set_sharpness = set_dummy;
    sensor->set_denoise = set_dummy;
    sensor->set_gainceiling = set_gainceiling_dummy;
    sensor->set_quality = set_dummy;
    sensor->set_colorbar = set_colorbar;
    sensor->set_whitebal = set_dummy;
    sensor->set_gain_ctrl = set_dummy;
    sensor->set_exposure_ctrl = set_dummy;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    sensor->set_aec2 = set_dummy;
    sensor->set_awb_gain = set_dummy;
    sensor->set_agc_gain = set_dummy;
    sensor->set_aec_value = set_dummy;

    sensor->set_special_effect = set_dummy;
    sensor->set_wb_mode = set_dummy;
    sensor->set_ae_level = set_dummy;

    sensor->set_dcw = set_dummy;
    sensor->set_bpc = set_dummy;
    sensor->set_wpc = set_dummy;

    sensor->set_raw_gma = set_dummy;
    sensor->set_lenc = set_dummy;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_res_raw = NULL;
    sensor->set_pll = NULL;
    sensor->set_xclk = NULL;

    // No autofocus support
    sensor->af_is_supported = NULL;
    sensor->af_init = NULL;
    sensor->af_set_mode = NULL;
    sensor->af_trigger = NULL;
    sensor->af_get_status = NULL;
    sensor->af_set_manual_position = NULL;

    ESP_LOGD(TAG, "GC2145 Attached");
    return 0;
}
