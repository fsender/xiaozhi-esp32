#include "wifi_board.h"
#include "display/lcd_display.h"
#include "codecs/box_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "power_save_timer.h"
#include "assets.h"
#include "qmi8658.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <esp_lcd_st77916.h>
#include <esp_lcd_touch_cst816s.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <esp_timer.h>
#include <cmath>
#include <esp_heap_caps.h>
#include <jpeg_decoder.h>
#define TAG "waveshare_lcd_1_85b"

#define LCD_OPCODE_WRITE_CMD        (0x02ULL)
#define LCD_OPCODE_READ_CMD         (0x0BULL)
#define LCD_OPCODE_WRITE_COLOR      (0x32ULL)

static const st77916_lcd_init_cmd_t vendor_specific_init_version_1[] = {
    {0xF0, (uint8_t []){0x28}, 1, 0},
    {0xF2, (uint8_t []){0x28}, 1, 0},
    {0x7C, (uint8_t []){0xD1}, 1, 0},
    {0x83, (uint8_t []){0xE0}, 1, 0},
    {0x84, (uint8_t []){0x61}, 1, 0},
    {0xF2, (uint8_t []){0x82}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x01}, 1, 0},
    {0xF1, (uint8_t []){0x01}, 1, 0},
    {0xB0, (uint8_t []){0x49}, 1, 0},
    {0xB1, (uint8_t []){0x4A}, 1, 0},
    {0xB2, (uint8_t []){0x1F}, 1, 0},
    {0xB4, (uint8_t []){0x46}, 1, 0},
    {0xB5, (uint8_t []){0x34}, 1, 0},
    {0xB6, (uint8_t []){0xD5}, 1, 0},
    {0xB7, (uint8_t []){0x30}, 1, 0},
    {0xB8, (uint8_t []){0x04}, 1, 0},
    {0xBA, (uint8_t []){0x00}, 1, 0},
    {0xBB, (uint8_t []){0x08}, 1, 0},
    {0xBC, (uint8_t []){0x08}, 1, 0},
    {0xBD, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x80}, 1, 0},
    {0xC1, (uint8_t []){0x10}, 1, 0},
    {0xC2, (uint8_t []){0x37}, 1, 0},
    {0xC3, (uint8_t []){0x80}, 1, 0},
    {0xC4, (uint8_t []){0x10}, 1, 0},
    {0xC5, (uint8_t []){0x37}, 1, 0},
    {0xC6, (uint8_t []){0xA9}, 1, 0},
    {0xC7, (uint8_t []){0x41}, 1, 0},
    {0xC8, (uint8_t []){0x01}, 1, 0},
    {0xC9, (uint8_t []){0xA9}, 1, 0},
    {0xCA, (uint8_t []){0x41}, 1, 0},
    {0xCB, (uint8_t []){0x01}, 1, 0},
    {0xD0, (uint8_t []){0x91}, 1, 0},
    {0xD1, (uint8_t []){0x68}, 1, 0},
    {0xD2, (uint8_t []){0x68}, 1, 0},
    {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
    // {0xDD, (uint8_t []){0x35}, 1, 0},
    // {0xDE, (uint8_t []){0x35}, 1, 0},
    // {0xDD, (uint8_t []){0x3F}, 1, 0},
    // {0xDE, (uint8_t []){0x3F}, 1, 0},
    {0xF1, (uint8_t []){0x10}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x02}, 1, 0},
    {0xE0, (uint8_t []){0x70, 0x09, 0x12, 0x0C, 0x0B, 0x27, 0x38, 0x54, 0x4E, 0x19, 0x15, 0x15, 0x2C, 0x2F}, 14, 0},
    {0xE1, (uint8_t []){0x70, 0x08, 0x11, 0x0C, 0x0B, 0x27, 0x38, 0x43, 0x4C, 0x18, 0x14, 0x14, 0x2B, 0x2D}, 14, 0},
    // {0xE0, (uint8_t []){0xF0, 0x0E, 0x15, 0x0B, 0x0B, 0x07, 0x3C, 0x44, 0x51, 0x38, 0x15, 0x15, 0x32, 0x36}, 14, 0},
    // {0xE1, (uint8_t []){0xF0, 0x0D, 0x15, 0x0A, 0x0A, 0x26, 0x3B, 0x43, 0x50, 0x37, 0x14, 0x15, 0x31, 0x36}, 14, 0},
    {0xF0, (uint8_t []){0x10}, 1, 0},
    {0xF3, (uint8_t []){0x10}, 1, 0},
    {0xE0, (uint8_t []){0x08}, 1, 0},
    {0xE1, (uint8_t []){0x00}, 1, 0},
    {0xE2, (uint8_t []){0x0B}, 1, 0},
    {0xE3, (uint8_t []){0x00}, 1, 0},
    {0xE4, (uint8_t []){0xE0}, 1, 0},
    {0xE5, (uint8_t []){0x06}, 1, 0},
    {0xE6, (uint8_t []){0x21}, 1, 0},
    {0xE7, (uint8_t []){0x00}, 1, 0},
    {0xE8, (uint8_t []){0x05}, 1, 0},
    {0xE9, (uint8_t []){0x82}, 1, 0},
    {0xEA, (uint8_t []){0xDF}, 1, 0},
    {0xEB, (uint8_t []){0x89}, 1, 0},
    {0xEC, (uint8_t []){0x20}, 1, 0},
    {0xED, (uint8_t []){0x14}, 1, 0},
    {0xEE, (uint8_t []){0xFF}, 1, 0},
    {0xEF, (uint8_t []){0x00}, 1, 0},
    {0xF8, (uint8_t []){0xFF}, 1, 0},
    {0xF9, (uint8_t []){0x00}, 1, 0},
    {0xFA, (uint8_t []){0x00}, 1, 0},
    {0xFB, (uint8_t []){0x30}, 1, 0},
    {0xFC, (uint8_t []){0x00}, 1, 0},
    {0xFD, (uint8_t []){0x00}, 1, 0},
    {0xFE, (uint8_t []){0x00}, 1, 0},
    {0xFF, (uint8_t []){0x00}, 1, 0},
    {0x60, (uint8_t []){0x42}, 1, 0},
    {0x61, (uint8_t []){0xE0}, 1, 0},
    {0x62, (uint8_t []){0x40}, 1, 0},
    {0x63, (uint8_t []){0x40}, 1, 0},
    {0x64, (uint8_t []){0x02}, 1, 0},
    {0x65, (uint8_t []){0x00}, 1, 0},
    {0x66, (uint8_t []){0x40}, 1, 0},
    {0x67, (uint8_t []){0x03}, 1, 0},
    {0x68, (uint8_t []){0x00}, 1, 0},
    {0x69, (uint8_t []){0x00}, 1, 0},
    {0x6A, (uint8_t []){0x00}, 1, 0},
    {0x6B, (uint8_t []){0x00}, 1, 0},
    {0x70, (uint8_t []){0x42}, 1, 0},
    {0x71, (uint8_t []){0xE0}, 1, 0},
    {0x72, (uint8_t []){0x40}, 1, 0},
    {0x73, (uint8_t []){0x40}, 1, 0},
    {0x74, (uint8_t []){0x02}, 1, 0},
    {0x75, (uint8_t []){0x00}, 1, 0},
    {0x76, (uint8_t []){0x40}, 1, 0},
    {0x77, (uint8_t []){0x03}, 1, 0},
    {0x78, (uint8_t []){0x00}, 1, 0},
    {0x79, (uint8_t []){0x00}, 1, 0},
    {0x7A, (uint8_t []){0x00}, 1, 0},
    {0x7B, (uint8_t []){0x00}, 1, 0},
    // {0x80, (uint8_t []){0x38}, 1, 0},
    {0x80, (uint8_t []){0x38}, 1, 0},
    {0x81, (uint8_t []){0x00}, 1, 0},
    // {0x82, (uint8_t []){0x04}, 1, 0},
    {0x82, (uint8_t []){0x04}, 1, 0},
    {0x83, (uint8_t []){0x02}, 1, 0},
    // {0x84, (uint8_t []){0xDC}, 1, 0},
    {0x84, (uint8_t []){0xDC}, 1, 0},
    {0x85, (uint8_t []){0x00}, 1, 0},
    {0x86, (uint8_t []){0x00}, 1, 0},
    {0x87, (uint8_t []){0x00}, 1, 0},
    // {0x88, (uint8_t []){0x38}, 1, 0},
    {0x88, (uint8_t []){0x38}, 1, 0},
    {0x89, (uint8_t []){0x00}, 1, 0},
    // {0x8A, (uint8_t []){0x06}, 1, 0},
    {0x8A, (uint8_t []){0x06}, 1, 0},
    {0x8B, (uint8_t []){0x02}, 1, 0},
    // {0x8C, (uint8_t []){0xDE}, 1, 0},
    {0x8C, (uint8_t []){0xDE}, 1, 0},
    {0x8D, (uint8_t []){0x00}, 1, 0},
    {0x8E, (uint8_t []){0x00}, 1, 0},
    {0x8F, (uint8_t []){0x00}, 1, 0},
    // {0x90, (uint8_t []){0x38}, 1, 0},
    {0x90, (uint8_t []){0x38}, 1, 0},
    {0x91, (uint8_t []){0x00}, 1, 0},
    // {0x92, (uint8_t []){0x08}, 1, 0},
    {0x92, (uint8_t []){0x08}, 1, 0},
    {0x93, (uint8_t []){0x02}, 1, 0},
    // {0x94, (uint8_t []){0xE0}, 1, 0},
    {0x94, (uint8_t []){0xE0}, 1, 0},
    {0x95, (uint8_t []){0x00}, 1, 0},
    {0x96, (uint8_t []){0x00}, 1, 0},
    {0x97, (uint8_t []){0x00}, 1, 0},
    // {0x98, (uint8_t []){0x38}, 1, 0},
    {0x98, (uint8_t []){0x38}, 1, 0},
    {0x99, (uint8_t []){0x00}, 1, 0},
    // {0x9A, (uint8_t []){0x0A}, 1, 0},
    {0x9A, (uint8_t []){0x0A}, 1, 0},
    {0x9B, (uint8_t []){0x02}, 1, 0},
    // {0x9C, (uint8_t []){0xE2}, 1, 0},
    {0x9C, (uint8_t []){0xE2}, 1, 0},
    {0x9D, (uint8_t []){0x00}, 1, 0},
    {0x9E, (uint8_t []){0x00}, 1, 0},
    {0x9F, (uint8_t []){0x00}, 1, 0},
    // {0xA0, (uint8_t []){0x38}, 1, 0},
    {0xA0, (uint8_t []){0x38}, 1, 0},
    {0xA1, (uint8_t []){0x00}, 1, 0},
    // {0xA2, (uint8_t []){0x03}, 1, 0},
    {0xA2, (uint8_t []){0x03}, 1, 0},
    {0xA3, (uint8_t []){0x02}, 1, 0},
    // {0xA4, (uint8_t []){0xDB}, 1, 0},
    {0xA4, (uint8_t []){0xDB}, 1, 0},
    {0xA5, (uint8_t []){0x00}, 1, 0},
    {0xA6, (uint8_t []){0x00}, 1, 0},
    {0xA7, (uint8_t []){0x00}, 1, 0},
    // {0xA8, (uint8_t []){0x38}, 1, 0},
    {0xA8, (uint8_t []){0x38}, 1, 0},
    {0xA9, (uint8_t []){0x00}, 1, 0},
    // {0xAA, (uint8_t []){0x05}, 1, 0},
    {0xAA, (uint8_t []){0x05}, 1, 0},
    {0xAB, (uint8_t []){0x02}, 1, 0},
    // {0xAC, (uint8_t []){0xDD}, 1, 0},
    {0xAC, (uint8_t []){0xDD}, 1, 0},
    {0xAD, (uint8_t []){0x00}, 1, 0},
    {0xAE, (uint8_t []){0x00}, 1, 0},
    {0xAF, (uint8_t []){0x00}, 1, 0},
    // {0xB0, (uint8_t []){0x38}, 1, 0},
    {0xB0, (uint8_t []){0x38}, 1, 0},
    {0xB1, (uint8_t []){0x00}, 1, 0},
    // {0xB2, (uint8_t []){0x07}, 1, 0},
    {0xB2, (uint8_t []){0x07}, 1, 0},
    {0xB3, (uint8_t []){0x02}, 1, 0},
    // {0xB4, (uint8_t []){0xDF}, 1, 0},
    {0xB4, (uint8_t []){0xDF}, 1, 0},
    {0xB5, (uint8_t []){0x00}, 1, 0},
    {0xB6, (uint8_t []){0x00}, 1, 0},
    {0xB7, (uint8_t []){0x00}, 1, 0},
    // {0xB8, (uint8_t []){0x38}, 1, 0},
    {0xB8, (uint8_t []){0x38}, 1, 0},
    {0xB9, (uint8_t []){0x00}, 1, 0},
    // {0xBA, (uint8_t []){0x09}, 1, 0},
    {0xBA, (uint8_t []){0x09}, 1, 0},
    {0xBB, (uint8_t []){0x02}, 1, 0},
    // {0xBC, (uint8_t []){0xE1}, 1, 0},
    {0xBC, (uint8_t []){0xE1}, 1, 0},
    {0xBD, (uint8_t []){0x00}, 1, 0},
    {0xBE, (uint8_t []){0x00}, 1, 0},
    {0xBF, (uint8_t []){0x00}, 1, 0},
    // {0xC0, (uint8_t []){0x22}, 1, 0},
    {0xC0, (uint8_t []){0x22}, 1, 0},
    {0xC1, (uint8_t []){0xAA}, 1, 0},
    {0xC2, (uint8_t []){0x65}, 1, 0},
    {0xC3, (uint8_t []){0x74}, 1, 0},
    {0xC4, (uint8_t []){0x47}, 1, 0},
    {0xC5, (uint8_t []){0x56}, 1, 0},
    {0xC6, (uint8_t []){0x00}, 1, 0},
    {0xC7, (uint8_t []){0x88}, 1, 0},
    {0xC8, (uint8_t []){0x99}, 1, 0},
    {0xC9, (uint8_t []){0x33}, 1, 0},
    // {0xD0, (uint8_t []){0x11}, 1, 0},
    {0xD0, (uint8_t []){0x11}, 1, 0},
    {0xD1, (uint8_t []){0xAA}, 1, 0},
    {0xD2, (uint8_t []){0x65}, 1, 0},
    {0xD3, (uint8_t []){0x74}, 1, 0},
    {0xD4, (uint8_t []){0x47}, 1, 0},
    {0xD5, (uint8_t []){0x56}, 1, 0},
    {0xD6, (uint8_t []){0x00}, 1, 0},
    {0xD7, (uint8_t []){0x88}, 1, 0},
    {0xD8, (uint8_t []){0x99}, 1, 0},
    {0xD9, (uint8_t []){0x33}, 1, 0},
    {0xF3, (uint8_t []){0x01}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    // {0x3A, (uint8_t []){0x55}, 1, 0},
    {0x21, (uint8_t []){0x00}, 0, 0},
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0x29, (uint8_t []){0x00}, 0, 0},
};
static const st77916_lcd_init_cmd_t vendor_specific_init_version_2[] = {
  {0xF0, (uint8_t []){0x28}, 1, 0},
  {0xF2, (uint8_t []){0x28}, 1, 0},
  {0x73, (uint8_t []){0xF0}, 1, 0},
  {0x7C, (uint8_t []){0xD1}, 1, 0},
  {0x83, (uint8_t []){0xE0}, 1, 0},
  {0x84, (uint8_t []){0x61}, 1, 0},
  {0xF2, (uint8_t []){0x82}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0xF0, (uint8_t []){0x01}, 1, 0},
  {0xF1, (uint8_t []){0x01}, 1, 0},
  {0xB0, (uint8_t []){0x56}, 1, 0},
  {0xB1, (uint8_t []){0x4D}, 1, 0},
  {0xB2, (uint8_t []){0x24}, 1, 0},
  {0xB4, (uint8_t []){0x87}, 1, 0},
  {0xB5, (uint8_t []){0x44}, 1, 0},
  {0xB6, (uint8_t []){0x8B}, 1, 0},
  {0xB7, (uint8_t []){0x40}, 1, 0},
  {0xB8, (uint8_t []){0x86}, 1, 0},
  {0xBA, (uint8_t []){0x00}, 1, 0},
  {0xBB, (uint8_t []){0x08}, 1, 0},
  {0xBC, (uint8_t []){0x08}, 1, 0},
  {0xBD, (uint8_t []){0x00}, 1, 0},
  {0xC0, (uint8_t []){0x80}, 1, 0},
  {0xC1, (uint8_t []){0x10}, 1, 0},
  {0xC2, (uint8_t []){0x37}, 1, 0},
  {0xC3, (uint8_t []){0x80}, 1, 0},
  {0xC4, (uint8_t []){0x10}, 1, 0},
  {0xC5, (uint8_t []){0x37}, 1, 0},
  {0xC6, (uint8_t []){0xA9}, 1, 0},
  {0xC7, (uint8_t []){0x41}, 1, 0},
  {0xC8, (uint8_t []){0x01}, 1, 0},
  {0xC9, (uint8_t []){0xA9}, 1, 0},
  {0xCA, (uint8_t []){0x41}, 1, 0},
  {0xCB, (uint8_t []){0x01}, 1, 0},
  {0xD0, (uint8_t []){0x91}, 1, 0},
  {0xD1, (uint8_t []){0x68}, 1, 0},
  {0xD2, (uint8_t []){0x68}, 1, 0},
  {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
  {0xDD, (uint8_t []){0x4F}, 1, 0},
  {0xDE, (uint8_t []){0x4F}, 1, 0},
  {0xF1, (uint8_t []){0x10}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0xF0, (uint8_t []){0x02}, 1, 0},
  {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
  {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
  {0xF0, (uint8_t []){0x10}, 1, 0},
  {0xF3, (uint8_t []){0x10}, 1, 0},
  {0xE0, (uint8_t []){0x07}, 1, 0},
  {0xE1, (uint8_t []){0x00}, 1, 0},
  {0xE2, (uint8_t []){0x00}, 1, 0},
  {0xE3, (uint8_t []){0x00}, 1, 0},
  {0xE4, (uint8_t []){0xE0}, 1, 0},
  {0xE5, (uint8_t []){0x06}, 1, 0},
  {0xE6, (uint8_t []){0x21}, 1, 0},
  {0xE7, (uint8_t []){0x01}, 1, 0},
  {0xE8, (uint8_t []){0x05}, 1, 0},
  {0xE9, (uint8_t []){0x02}, 1, 0},
  {0xEA, (uint8_t []){0xDA}, 1, 0},
  {0xEB, (uint8_t []){0x00}, 1, 0},
  {0xEC, (uint8_t []){0x00}, 1, 0},
  {0xED, (uint8_t []){0x0F}, 1, 0},
  {0xEE, (uint8_t []){0x00}, 1, 0},
  {0xEF, (uint8_t []){0x00}, 1, 0},
  {0xF8, (uint8_t []){0x00}, 1, 0},
  {0xF9, (uint8_t []){0x00}, 1, 0},
  {0xFA, (uint8_t []){0x00}, 1, 0},
  {0xFB, (uint8_t []){0x00}, 1, 0},
  {0xFC, (uint8_t []){0x00}, 1, 0},
  {0xFD, (uint8_t []){0x00}, 1, 0},
  {0xFE, (uint8_t []){0x00}, 1, 0},
  {0xFF, (uint8_t []){0x00}, 1, 0},
  {0x60, (uint8_t []){0x40}, 1, 0},
  {0x61, (uint8_t []){0x04}, 1, 0},
  {0x62, (uint8_t []){0x00}, 1, 0},
  {0x63, (uint8_t []){0x42}, 1, 0},
  {0x64, (uint8_t []){0xD9}, 1, 0},
  {0x65, (uint8_t []){0x00}, 1, 0},
  {0x66, (uint8_t []){0x00}, 1, 0},
  {0x67, (uint8_t []){0x00}, 1, 0},
  {0x68, (uint8_t []){0x00}, 1, 0},
  {0x69, (uint8_t []){0x00}, 1, 0},
  {0x6A, (uint8_t []){0x00}, 1, 0},
  {0x6B, (uint8_t []){0x00}, 1, 0},
  {0x70, (uint8_t []){0x40}, 1, 0},
  {0x71, (uint8_t []){0x03}, 1, 0},
  {0x72, (uint8_t []){0x00}, 1, 0},
  {0x73, (uint8_t []){0x42}, 1, 0},
  {0x74, (uint8_t []){0xD8}, 1, 0},
  {0x75, (uint8_t []){0x00}, 1, 0},
  {0x76, (uint8_t []){0x00}, 1, 0},
  {0x77, (uint8_t []){0x00}, 1, 0},
  {0x78, (uint8_t []){0x00}, 1, 0},
  {0x79, (uint8_t []){0x00}, 1, 0},
  {0x7A, (uint8_t []){0x00}, 1, 0},
  {0x7B, (uint8_t []){0x00}, 1, 0},
  {0x80, (uint8_t []){0x48}, 1, 0},
  {0x81, (uint8_t []){0x00}, 1, 0},
  {0x82, (uint8_t []){0x06}, 1, 0},
  {0x83, (uint8_t []){0x02}, 1, 0},
  {0x84, (uint8_t []){0xD6}, 1, 0},
  {0x85, (uint8_t []){0x04}, 1, 0},
  {0x86, (uint8_t []){0x00}, 1, 0},
  {0x87, (uint8_t []){0x00}, 1, 0},
  {0x88, (uint8_t []){0x48}, 1, 0},
  {0x89, (uint8_t []){0x00}, 1, 0},
  {0x8A, (uint8_t []){0x08}, 1, 0},
  {0x8B, (uint8_t []){0x02}, 1, 0},
  {0x8C, (uint8_t []){0xD8}, 1, 0},
  {0x8D, (uint8_t []){0x04}, 1, 0},
  {0x8E, (uint8_t []){0x00}, 1, 0},
  {0x8F, (uint8_t []){0x00}, 1, 0},
  {0x90, (uint8_t []){0x48}, 1, 0},
  {0x91, (uint8_t []){0x00}, 1, 0},
  {0x92, (uint8_t []){0x0A}, 1, 0},
  {0x93, (uint8_t []){0x02}, 1, 0},
  {0x94, (uint8_t []){0xDA}, 1, 0},
  {0x95, (uint8_t []){0x04}, 1, 0},
  {0x96, (uint8_t []){0x00}, 1, 0},
  {0x97, (uint8_t []){0x00}, 1, 0},
  {0x98, (uint8_t []){0x48}, 1, 0},
  {0x99, (uint8_t []){0x00}, 1, 0},
  {0x9A, (uint8_t []){0x0C}, 1, 0},
  {0x9B, (uint8_t []){0x02}, 1, 0},
  {0x9C, (uint8_t []){0xDC}, 1, 0},
  {0x9D, (uint8_t []){0x04}, 1, 0},
  {0x9E, (uint8_t []){0x00}, 1, 0},
  {0x9F, (uint8_t []){0x00}, 1, 0},
  {0xA0, (uint8_t []){0x48}, 1, 0},
  {0xA1, (uint8_t []){0x00}, 1, 0},
  {0xA2, (uint8_t []){0x05}, 1, 0},
  {0xA3, (uint8_t []){0x02}, 1, 0},
  {0xA4, (uint8_t []){0xD5}, 1, 0},
  {0xA5, (uint8_t []){0x04}, 1, 0},
  {0xA6, (uint8_t []){0x00}, 1, 0},
  {0xA7, (uint8_t []){0x00}, 1, 0},
  {0xA8, (uint8_t []){0x48}, 1, 0},
  {0xA9, (uint8_t []){0x00}, 1, 0},
  {0xAA, (uint8_t []){0x07}, 1, 0},
  {0xAB, (uint8_t []){0x02}, 1, 0},
  {0xAC, (uint8_t []){0xD7}, 1, 0},
  {0xAD, (uint8_t []){0x04}, 1, 0},
  {0xAE, (uint8_t []){0x00}, 1, 0},
  {0xAF, (uint8_t []){0x00}, 1, 0},
  {0xB0, (uint8_t []){0x48}, 1, 0},
  {0xB1, (uint8_t []){0x00}, 1, 0},
  {0xB2, (uint8_t []){0x09}, 1, 0},
  {0xB3, (uint8_t []){0x02}, 1, 0},
  {0xB4, (uint8_t []){0xD9}, 1, 0},
  {0xB5, (uint8_t []){0x04}, 1, 0},
  {0xB6, (uint8_t []){0x00}, 1, 0},
  {0xB7, (uint8_t []){0x00}, 1, 0},
  
  {0xB8, (uint8_t []){0x48}, 1, 0},
  {0xB9, (uint8_t []){0x00}, 1, 0},
  {0xBA, (uint8_t []){0x0B}, 1, 0},
  {0xBB, (uint8_t []){0x02}, 1, 0},
  {0xBC, (uint8_t []){0xDB}, 1, 0},
  {0xBD, (uint8_t []){0x04}, 1, 0},
  {0xBE, (uint8_t []){0x00}, 1, 0},
  {0xBF, (uint8_t []){0x00}, 1, 0},
  {0xC0, (uint8_t []){0x10}, 1, 0},
  {0xC1, (uint8_t []){0x47}, 1, 0},
  {0xC2, (uint8_t []){0x56}, 1, 0},
  {0xC3, (uint8_t []){0x65}, 1, 0},
  {0xC4, (uint8_t []){0x74}, 1, 0},
  {0xC5, (uint8_t []){0x88}, 1, 0},
  {0xC6, (uint8_t []){0x99}, 1, 0},
  {0xC7, (uint8_t []){0x01}, 1, 0},
  {0xC8, (uint8_t []){0xBB}, 1, 0},
  {0xC9, (uint8_t []){0xAA}, 1, 0},
  {0xD0, (uint8_t []){0x10}, 1, 0},
  {0xD1, (uint8_t []){0x47}, 1, 0},
  {0xD2, (uint8_t []){0x56}, 1, 0},
  {0xD3, (uint8_t []){0x65}, 1, 0},
  {0xD4, (uint8_t []){0x74}, 1, 0},
  {0xD5, (uint8_t []){0x88}, 1, 0},
  {0xD6, (uint8_t []){0x99}, 1, 0},
  {0xD7, (uint8_t []){0x01}, 1, 0},
  {0xD8, (uint8_t []){0xBB}, 1, 0},
  {0xD9, (uint8_t []){0xAA}, 1, 0},
  {0xF3, (uint8_t []){0x01}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0x21, (uint8_t []){0x00}, 1, 0},
  {0x11, (uint8_t []){0x00}, 1, 120},
  {0x29, (uint8_t []){0x00}, 1, 0},  
};

class CustomLcdDisplay : public SpiLcdDisplay {
public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                    esp_lcd_panel_handle_t panel_handle,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy)
        : SpiLcdDisplay(io_handle, panel_handle,
                    width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
    }

    lv_obj_t* bg_img_ = nullptr;
    uint8_t* bg_pixels_ = nullptr;

    // GIF queue system
    struct GifStep { std::string name; bool loop; };
    std::vector<GifStep> gif_queue_;
    bool gif_busy_ = false;
    lv_timer_t* gif_watchdog_timer_ = nullptr;
    std::string current_state_ = "idle";
    bool gesture_active_ = false;  // Prevent tap during gesture
    enum class WakeSource { NONE, SHAKE_IMU };
    volatile WakeSource pending_wake_source_ = WakeSource::NONE;

    virtual void SetupUI() override {
        SpiLcdDisplay::SetupUI();
        DisplayLockGuard lock(this);

        auto screen = lv_screen_active();

        // Make container transparent, non-scrollable, and enable event bubbling
        if (container_) {
            lv_obj_set_style_bg_opa(container_, LV_OPA_TRANSP, 0);
            lv_obj_remove_flag(container_, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_scrollbar_mode(container_, LV_SCROLLBAR_MODE_OFF);
            lv_obj_add_flag(container_, LV_OBJ_FLAG_EVENT_BUBBLE);
        }

        // Load and decode bg.jpg using ESP ROM JPEG decoder
        void* bg_ptr = nullptr;
        size_t bg_size = 0;
        if (Assets::GetInstance().GetAssetData("bg.jpg", bg_ptr, bg_size)) {
            ESP_LOGW("CustomDisplay", "bg.jpg found, size=%u bytes, decoding...", (unsigned)bg_size);

            // Allocate output buffer for RGB565 decoded image
            bg_pixels_ = (uint8_t*)heap_caps_malloc(DISPLAY_WIDTH * DISPLAY_HEIGHT * 2, MALLOC_CAP_SPIRAM);
            if (bg_pixels_) {
                esp_jpeg_image_cfg_t cfg = {};
                cfg.indata = static_cast<uint8_t*>(bg_ptr);
                cfg.indata_size = bg_size;
                cfg.outbuf = bg_pixels_;
                cfg.outbuf_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;
                cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
                cfg.out_scale = JPEG_IMAGE_SCALE_0;
                cfg.flags.swap_color_bytes = 0;

                esp_jpeg_image_output_t img_info = {};
                esp_err_t ret = esp_jpeg_decode(&cfg, &img_info);
                if (ret == ESP_OK) {
                    ESP_LOGW("CustomDisplay", "JPEG decoded: %dx%d, %u bytes", img_info.width, img_info.height, (unsigned)img_info.output_len);

                    static lv_img_dsc_t bg_dsc = {};
                    bg_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
                    bg_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
                    bg_dsc.header.w = img_info.width;
                    bg_dsc.header.h = img_info.height;
                    bg_dsc.header.stride = img_info.width * 2;
                    bg_dsc.data = bg_pixels_;
                    bg_dsc.data_size = img_info.output_len;

                    bg_img_ = lv_image_create(screen);
                    lv_image_set_src(bg_img_, &bg_dsc);
                    lv_obj_set_size(bg_img_, LV_HOR_RES, LV_VER_RES);
                    lv_obj_align(bg_img_, LV_ALIGN_CENTER, 0, 0);
                    lv_obj_move_to_index(bg_img_, 0);
                    ESP_LOGW("CustomDisplay", "Background image displayed successfully!");
                } else {
                    ESP_LOGE("CustomDisplay", "JPEG decode failed: %d", ret);
                    lv_obj_set_style_bg_color(screen, lv_color_hex(0x1A3A5C), 0);
                    heap_caps_free(bg_pixels_);
                    bg_pixels_ = nullptr;
                }
            } else {
                ESP_LOGE("CustomDisplay", "Failed to allocate PSRAM for bg image");
                lv_obj_set_style_bg_color(screen, lv_color_hex(0x1A3A5C), 0);
            }
        } else {
            ESP_LOGE("CustomDisplay", "bg.jpg NOT found in assets partition!");
            lv_obj_set_style_bg_color(screen, lv_color_hex(0x1A3A5C), 0);
        }

        ApplyWhiteTheme();

        // Initialize GIF watchdog timer for queue system
        InitGifWatchdog();

        // Register touch gesture handlers on the GIF image area
        if (emoji_image_) {
            lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(emoji_image_, OnEmojiClicked, LV_EVENT_CLICKED, NULL);
            ESP_LOGW("CustomDisplay", "Tap handler on emoji_image_");
        }
        if (emoji_label_) {
            lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(emoji_label_, OnEmojiClicked, LV_EVENT_CLICKED, NULL);
        }
        // Swipe gesture on screen
        lv_obj_add_event_cb(screen, OnGesture, LV_EVENT_GESTURE, NULL);
        ESP_LOGW("CustomDisplay", "All gesture handlers registered");
    }

    static void OnGesture(lv_event_t* e) {
        static uint32_t last_gesture_time = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_gesture_time < 2000) return;
        last_gesture_time = now;

        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();

        // Only respond to swipe in idle state
        if (state != kDeviceStateIdle) {
            ESP_LOGW("Gesture", "Swipe ignored: state=%d (not idle)", state);
            return;
        }

        lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
        auto* display = dynamic_cast<CustomLcdDisplay*>(Board::GetInstance().GetDisplay());
        if (!display) return;

        display->gesture_active_ = true;

        switch (dir) {
            case LV_DIR_LEFT:
            case LV_DIR_RIGHT:
                ESP_LOGW("Gesture", ">>> SWIPE HORIZONTAL -> shake2 -> listen -> listening");
                display->WakeWithTrigger("swipe_horizontal");
                break;
            case LV_DIR_TOP:
            case LV_DIR_BOTTOM:
                ESP_LOGW("Gesture", ">>> SWIPE VERTICAL -> shake1 -> listen -> listening");
                display->WakeWithTrigger("swipe_vertical");
                break;
            default:
                break;
        }
        app.StartListening();
    }

    static void OnEmojiClicked(lv_event_t* e) {
        auto* display = dynamic_cast<CustomLcdDisplay*>(Board::GetInstance().GetDisplay());
        // Skip tap if gesture just fired (within 500ms)
        if (display && display->gesture_active_) {
            ESP_LOGW("Gesture", "TAP skipped: gesture active");
            display->gesture_active_ = false;
            return;
        }

        static uint32_t last_tap_time = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_tap_time < 2000) return;
        last_tap_time = now;

        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();

        if (state == kDeviceStateIdle) {
            // Idle: tap to wake with GIF chain
            if (display) {
                ESP_LOGW("Gesture", ">>> TAP (idle) -> tap.gif -> listen_start -> listening");
                display->WakeWithTrigger("tap");
            }
            app.ToggleChatState();
        } else if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
            // Listening/Speaking: tap to interrupt (SetEmotion handles the GIF transition)
            ESP_LOGW("Gesture", ">>> TAP (listening/speaking) -> interrupt -> idle");
            app.ToggleChatState();
        } else {
            ESP_LOGW("Gesture", "TAP ignored: state=%d", state);
        }
    }

    // --- GIF Queue System ---
    void InitGifWatchdog() {
        // Use LVGL timer - runs in LVGL task context, safe for lv_image_set_src
        gif_watchdog_timer_ = lv_timer_create([](lv_timer_t* timer) {
            auto* self = static_cast<CustomLcdDisplay*>(lv_timer_get_user_data(timer));
            if (self->gif_controller_ && !self->gif_controller_->IsPlaying() && !self->gif_queue_.empty()) {
                self->gif_queue_.erase(self->gif_queue_.begin());
                self->PlayNextInQueue();
            }
        }, 200, this);
        lv_timer_pause(gif_watchdog_timer_);  // Start paused
    }

    void PlayNextInQueue() {
        if (gif_queue_.empty()) {
            gif_busy_ = false;
            if (gif_watchdog_timer_) lv_timer_pause(gif_watchdog_timer_);
            ESP_LOGW("GifDisplay", "Queue empty, gif_busy_=false");
            // Apply current state GIF (in case it was skipped during busy)
            ApplyCurrentStateGif();
            return;
        }
        auto& step = gif_queue_.front();
        ESP_LOGW("GifDisplay", "Queue next: %s (%s)", step.name.c_str(), step.loop ? "loop" : "once");

        // If this is the final looping step, clear gif_busy_
        if (step.loop && gif_queue_.size() == 1) {
            gif_busy_ = false;
            ESP_LOGW("GifDisplay", "Final loop step, gif_busy_=false");
            // Don't apply current state here - let the looping GIF play
        }

        LoadAndPlayGif(step.name.c_str(), step.loop);

        if (!step.loop && gif_watchdog_timer_) {
            lv_timer_resume(gif_watchdog_timer_);
            lv_timer_reset(gif_watchdog_timer_);
        } else if (step.loop && gif_watchdog_timer_) {
            lv_timer_pause(gif_watchdog_timer_);
        }
    }

    void ApplyCurrentStateGif() {
        const char* target_gif = GetStateTargetGif();
        std::string target_state;
        if (strstr(target_gif, "idle")) target_state = "idle";
        else if (strstr(target_gif, "listening")) target_state = "listening";
        else if (strstr(target_gif, "speaking")) target_state = "speaking";
        else if (strstr(target_gif, "thinking")) target_state = "thinking";
        else target_state = "idle";

        if (current_state_ != target_state) {
            ESP_LOGW("GifDisplay", "State mismatch after busy: %s -> %s, applying transition",
                     current_state_.c_str(), target_state.c_str());
            const char* trans_gif = GetTransitionGif(current_state_.c_str(), target_state.c_str());
            if (trans_gif) {
                EnqueueGifChain({{trans_gif, false}, {target_gif, true}}, false);
            } else {
                EnqueueGifChain({{target_gif, true}}, false);
            }
            current_state_ = target_state;
        } else if (gif_queue_.empty()) {
            // Same state but queue is empty - just play the target GIF
            ESP_LOGW("GifDisplay", "Applying state GIF: %s", target_gif);
            LoadAndPlayGif(target_gif, true);
        }
    }

    void EnqueueGifChain(std::vector<GifStep> steps, bool high_priority) {
        if (gif_busy_ && !high_priority) {
            ESP_LOGW("GifDisplay", "Busy, ignoring normal priority chain");
            return;
        }
        if (high_priority) {
            if (gif_controller_) {
                gif_controller_->Stop();
                gif_controller_.reset();
            }
            if (gif_watchdog_timer_) lv_timer_pause(gif_watchdog_timer_);
        }
        // ALL chains set gif_busy_ to prevent interruption during playback
        gif_busy_ = true;
        gif_queue_ = steps;
        ESP_LOGW("GifDisplay", "Enqueued %d steps, priority=%s, gif_busy_=true",
                 (int)steps.size(), high_priority ? "HIGH" : "normal");
        PlayNextInQueue();
    }

    // Wake-up trigger: high priority chain
    void WakeWithTrigger(const char* trigger) {
        std::string initial_gif;
        if (strcmp(trigger, "tap") == 0 || strcmp(trigger, "button") == 0) {
            initial_gif = "tap.gif";
        } else if (strcmp(trigger, "swipe_vertical") == 0) {
            initial_gif = "shake1.gif";
        } else {
            initial_gif = "shake2.gif";
        }
        ESP_LOGW("GifDisplay", "WakeWithTrigger('%s') -> %s", trigger, initial_gif.c_str());
        EnqueueGifChain({
            {initial_gif, false},
            {"listen_start.gif", false},
            {"listening.gif", true},
        }, true);
        current_state_ = "listening";
    }

    // Interrupt the current animation chain and transition to idle.
    // gif_queue_[0] is always the currently-playing GIF; keep it so the
    // watchdog advances correctly, and rebuild the tail based on what is
    // playing now.
    void InterruptToIdle() {
        if (!gif_busy_) {
            // Currently looping (listening/speaking) - normal SetEmotion
            // transition (listening -> idle) handles listen_end -> idle.
            return;
        }
        std::string current = gif_queue_.empty() ? "" : gif_queue_.front().name;
        gif_queue_.clear();
        if (!current.empty()) {
            gif_queue_.push_back({current, false});
            if (current == "listen_start.gif") {
                // Wait for listen_start to finish, then a full listen_end.
                gif_queue_.push_back({"listen_end.gif", false});
            }
        }
        gif_queue_.push_back({"idle.gif", true});
        current_state_ = "idle";
        ESP_LOGW("GifDisplay", "InterruptToIdle: current=%s", current.c_str());
    }

    // --- End GIF Queue System ---

    virtual void SetTheme(Theme* theme) override {
        SpiLcdDisplay::SetTheme(theme);
        // Reapply customizations after parent SetTheme overwrites them
        if (container_) {
            lv_obj_set_style_bg_opa(container_, LV_OPA_TRANSP, 0);
        }
        if (bg_img_) {
            lv_obj_move_to_index(bg_img_, 0);  // Ensure bg stays at back
        }
        ApplyWhiteTheme();
    }

    void ApplyWhiteTheme() {
        if (!setup_ui_called_) return;
        DisplayLockGuard lock(this);

        lv_color_t white = lv_color_hex(0xFFFFFF);
        lv_color_t chat_text = lv_color_hex(0x0F6DB7);

        // Hide top bar (WiFi, battery, mute icons)
        if (top_bar_) lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);

        // Status bar: white text for time, no background
        if (status_label_) {
            lv_obj_set_style_text_color(status_label_, white, 0);
            lv_obj_set_style_bg_opa(status_bar_, LV_OPA_TRANSP, 0);
        }
        if (notification_label_) lv_obj_set_style_text_color(notification_label_, white, 0);

        // Content area: white rounded rectangle, 85% opacity, blue text
        if (content_) {
            lv_obj_set_style_bg_color(content_, white, 0);
            lv_obj_set_style_bg_opa(content_, 216, 0);  // 85% opacity
            lv_obj_set_style_radius(content_, 16, 0);
            lv_obj_set_style_border_width(content_, 0, 0);
            lv_obj_set_style_text_color(content_, chat_text, 0);
        }
        if (chat_message_label_) {
            lv_obj_set_style_text_color(chat_message_label_, chat_text, 0);
        }
    }

    bool LoadAndPlayGif(const char* gif_name, bool loop) {
        void* ptr = nullptr;
        size_t size = 0;
        if (!Assets::GetInstance().GetAssetData(gif_name, ptr, size)) {
            ESP_LOGE("GifDisplay", "Asset not found: %s", gif_name);
            return false;
        }

        if (gif_controller_) {
            gif_controller_->Stop();
            gif_controller_.reset();
        }

        lv_img_dsc_t tmp_dsc = {};
        tmp_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
        tmp_dsc.header.cf = LV_COLOR_FORMAT_ARGB8888;
        tmp_dsc.data = static_cast<const uint8_t*>(ptr);
        tmp_dsc.data_size = size;
        gif_controller_ = std::make_unique<LvglGif>(&tmp_dsc);

        if (!gif_controller_ || !gif_controller_->IsLoaded()) {
            ESP_LOGE("GifDisplay", "GIF decode failed: %s", gif_name);
            gif_controller_.reset();
            return false;
        }

        if (!loop) {
            gif_controller_->SetLoopCount(1);
        }

        gif_controller_->SetFrameCallback(
            [this]() { lv_image_set_src(emoji_image_, gif_controller_->image_dsc()); });

        // Don't call lv_image_set_src here - let the GIF timer handle all frame updates
        // to avoid blocking the main task with LVGL rendering
        gif_controller_->Start();

        if (emoji_label_) lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);

        ESP_LOGW("GifDisplay", ">>> PLAYING: %s (%s, %dx%d)", gif_name,
                 loop ? "loop" : "once", gif_controller_->width(), gif_controller_->height());
        return true;
    }

    // Get state-based target GIF
    const char* GetStateTargetGif() {
        auto state = Application::GetInstance().GetDeviceState();
        switch (state) {
            case kDeviceStateConnecting:  return "listening.gif";
            case kDeviceStateListening:   return "listening.gif";
            case kDeviceStateSpeaking:    return "speaking.gif";
            default:                      return "idle.gif";
        }
    }

    // Get transition GIF for state change (only during conversation)
    const char* GetTransitionGif(const char* from_state, const char* to_state) {
        if (!from_state || !to_state) return nullptr;
        std::string from(from_state), to(to_state);

        if (from == "idle" && to == "listening")       return "listen_start.gif";
        if (from == "listening" && to == "speaking")   return "listen_to_speak.gif";
        if (from == "speaking" && to == "listening")   return "listen_start.gif";
        if (from == "listening" && to == "thinking")   return "listen_to_think.gif";
        if (from == "listening" && to == "idle")        return "listen_end.gif";
        if (from == "speaking" && to == "idle")         return "speak_end.gif";
        if (from == "thinking" && to == "idle")         return "listen_end.gif";
        return nullptr;
    }

    virtual void SetEmotion(const char* emotion) override {
        if (!emotion) emotion = "neutral";
        if (!emoji_image_) {
            SpiLcdDisplay::SetEmotion(emotion);
            return;
        }

        const char* target_gif = GetStateTargetGif();
        std::string prev_state = current_state_;
        std::string target_state;
        if (strstr(target_gif, "idle")) target_state = "idle";
        else if (strstr(target_gif, "listening")) target_state = "listening";
        else if (strstr(target_gif, "speaking")) target_state = "speaking";
        else if (strstr(target_gif, "thinking")) target_state = "thinking";
        else target_state = "idle";

        // Always update state tracking
        current_state_ = target_state;

        // All GIF/LVGL work below must hold the LVGL lock (SetEmotion runs on
        // the main task, not the LVGL task).
        DisplayLockGuard lock(this);

        // Interrupt: transitioning to speaking while listen_start.gif is still
        // playing (fast server reply after wake) -> go straight to speaking.gif.
        if (target_state == "speaking" && gif_busy_ && !gif_queue_.empty() &&
            gif_queue_.front().name == "listen_start.gif") {
            ESP_LOGW("GifDisplay", "Speaking while listen_start playing -> interrupt to speaking.gif");
            EnqueueGifChain({{"speaking.gif", true}}, true);
            return;
        }

        // Interrupt: transitioning to idle while a wake chain is still playing.
        // Rebuild the queue to finish at idle instead of continuing to listen.
        if (target_state == "idle" && prev_state != "idle" && gif_busy_) {
            InterruptToIdle();
            return;
        }

        // Skip GIF changes if a high-priority chain is still active
        if (gif_busy_) {
            return;
        }

        // Check if IMU shake triggered this wake
        WakeSource wake_src = pending_wake_source_;
        pending_wake_source_ = WakeSource::NONE;

        if (wake_src == WakeSource::SHAKE_IMU && prev_state == "idle") {
            // IMU shake: play random shake GIF only (no wake)
            const char* shake_gif = (esp_random() & 1) ? "shake1.gif" : "shake2.gif";
            EnqueueGifChain({
                {shake_gif, false},
                {"idle.gif", true},
            }, true);
            return;
        }

        // Normal state transition
        const char* trans_gif = nullptr;
        if (prev_state != target_state) {
            trans_gif = GetTransitionGif(prev_state.c_str(), target_state.c_str());
        }

        if (trans_gif) {
            EnqueueGifChain({
                {trans_gif, false},
                {target_gif, true},
            }, false);
        } else {
            EnqueueGifChain({{target_gif, true}}, false);
        }
    }
};

class WaveshareEsp32s3TouchLcd1_85B : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    CustomLcdDisplay* display_;
    PowerSaveTimer* power_save_timer_;
    // QMI8658 IMU
    Qmi8658* imu_ = nullptr;
    TaskHandle_t imu_task_handle_ = nullptr;
    volatile bool shake_pending_ = false;

    static void st77916_reset(void)
    {
        gpio_config_t ioconf = {
            .pin_bit_mask = 1ULL << QSPI_PIN_NUM_LCD_RST,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&ioconf);

        gpio_set_level(QSPI_PIN_NUM_LCD_RST,0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(QSPI_PIN_NUM_LCD_RST,1);
        vTaskDelay(pdMS_TO_TICKS(10));

    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(20); });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness(); });
        // power_save_timer_->OnShutdownRequest([this](){ 
        //     pmic_->PowerOff(); });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeCodecI2c() {
        ESP_LOGI(TAG, "Initialize I2C0: SDA=%d, SCL=%d",
                 AUDIO_CODEC_I2C_SDA_PIN, AUDIO_CODEC_I2C_SCL_PIN);
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }


    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize QSPI bus");

        const spi_bus_config_t bus_config = TAIJIPI_ST77916_PANEL_BUS_QSPI_CONFIG(QSPI_PIN_NUM_LCD_PCLK,
                                                                        QSPI_PIN_NUM_LCD_DATA0,
                                                                        QSPI_PIN_NUM_LCD_DATA1,
                                                                        QSPI_PIN_NUM_LCD_DATA2,
                                                                        QSPI_PIN_NUM_LCD_DATA3,
                                                                        QSPI_LCD_H_RES * 80 * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void Initializest77916Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install panel IO");

        esp_lcd_panel_io_spi_config_t io_config = {
            .cs_gpio_num = QSPI_PIN_NUM_LCD_CS,               
            .dc_gpio_num = GPIO_NUM_NC,
            .spi_mode = 0,                     
            .pclk_hz = 3 * 1000 * 1000,      
            .trans_queue_depth = 10,            
            .on_color_trans_done = NULL,                            
            .user_ctx = NULL,                   
            .lcd_cmd_bits = 32,                 
            .lcd_param_bits = 8,                
            .flags = {                          
            .dc_low_on_data = 0,            
            .octal_mode = 0,                
            .quad_mode = 1,                 
            .sio_mode = 0,                  
            .lsb_first = 0,                 
            .cs_high_active = 0,            
            },                                  
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "Install ST77916 panel driver");
        
        st77916_vendor_config_t vendor_config = {
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        
        printf("-------------------------------------- Version selection -------------------------------------- \r\n");
        esp_err_t ret;
        int lcd_cmd = 0x04;
        uint8_t register_data[4] = {};
        size_t param_size = sizeof(register_data);
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_READ_CMD << 24;  // Use the read opcode instead of write
        ret = esp_lcd_panel_io_rx_param(panel_io, lcd_cmd, register_data, param_size); 
        if (ret == ESP_OK) {
            printf("Register 0x04 data: %02x %02x %02x %02x\n", register_data[0], register_data[1], register_data[2], register_data[3]);
        } else {
            printf("Failed to read register 0x04, error code: %d\n", ret);
        } 
        ESP_ERROR_CHECK(esp_lcd_panel_io_del(panel_io));
        panel_io = nullptr;
        io_config.pclk_hz = 80 * 1000 * 1000;
        if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io) != ESP_OK) {
            printf("Failed to set LCD communication parameters -- SPI\r\n");
            return ;
        }
        printf("LCD communication parameters are set successfully -- SPI\r\n");
        
        // Check register values and configure accordingly
        if (register_data[0] == 0x00 && register_data[1] == 0x7F && register_data[2] == 0x7F && register_data[3] == 0x7F) {
            vendor_config.init_cmds = vendor_specific_init_version_1;
            vendor_config.init_cmds_size = sizeof(vendor_specific_init_version_1) / sizeof(st77916_lcd_init_cmd_t);
            printf("Vendor-specific initialization for case 1.\n");
        }
        else if (register_data[0] == 0x00 && register_data[1] == 0x02 && register_data[2] == 0x7F && register_data[3] == 0x7F) {
            vendor_config.init_cmds = vendor_specific_init_version_2;
            vendor_config.init_cmds_size = sizeof(vendor_specific_init_version_2) / sizeof(st77916_lcd_init_cmd_t);
            printf("Vendor-specific initialization for case 2.\n");
        }
        printf("------------------------------------- End of version selection------------------------------------- \r\n");
 
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL;
        panel_config.reset_gpio_num = QSPI_PIN_NUM_LCD_RST;
        panel_config.vendor_config = &vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new CustomLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t touch_handle;
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;

        esp_lcd_panel_io_i2c_config_t tp_io_config = {};
        tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS;
        tp_io_config.scl_speed_hz = 100000;
        tp_io_config.control_phase_bytes = 1;
        tp_io_config.dc_bit_offset = 0;
        tp_io_config.lcd_cmd_bits = 8;
        tp_io_config.lcd_param_bits = 0;
        tp_io_config.flags.disable_control_phase = 1;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));

        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = TP_PIN_NUM_RST,
            .int_gpio_num = TP_PIN_NUM_INT,
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };
        ESP_LOGI(TAG, "Initialize touch controller CST816");
        if (esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle) == ESP_OK) {
            const lvgl_port_touch_cfg_t touch_cfg = {
                .disp = lv_display_get_default(),
                .handle = touch_handle,
            };
            lvgl_port_add_touch(&touch_cfg);
            ESP_LOGI(TAG, "Touch panel initialized successfully");
        } else {
            ESP_LOGW(TAG, "Touch panel initialization failed");
        }
    }

    void InitializeQmi8658() {
        ESP_LOGI(TAG, "Initializing QMI8658 IMU on shared I2C bus");
        imu_ = new Qmi8658(i2c_bus_, 0x6B);
        if (imu_->Init()) {
            ESP_LOGI(TAG, "QMI8658 IMU initialized at address 0x6B");
            StartImuTask();
            return;
        }
        delete imu_;
        imu_ = nullptr;

        imu_ = new Qmi8658(i2c_bus_, 0x6A);
        if (imu_->Init()) {
            ESP_LOGI(TAG, "QMI8658 IMU initialized at address 0x6A");
            StartImuTask();
            return;
        }
        delete imu_;
        imu_ = nullptr;

        ESP_LOGW(TAG, "QMI8658 init failed at both addresses (0x6A and 0x6B)");
    }

    void StartImuTask() {
        xTaskCreatePinnedToCore(ImuTask, "imu_task", 4 * 1024, this, 5, &imu_task_handle_, 1);
        ESP_LOGI(TAG, "IMU motion detection task started");
    }

    static void ImuTask(void* arg) {
        auto* self = static_cast<WaveshareEsp32s3TouchLcd1_85B*>(arg);
        if (!self || !self->imu_) {
            vTaskDelete(NULL);
            return;
        }

        constexpr float kShakeAccelThreshold = 2.5f;
        constexpr int64_t kShakeCooldownMs = 2000;
        constexpr float kTiltThreshold = 0.4f;
        constexpr int64_t kTiltCooldownMs = 1500;

        float prev_ax = 0, prev_ay = 0, prev_az = 0;
        bool has_prev = false;
        int64_t last_shake_ms = 0;
        int64_t last_tilt_ms = 0;

        while (true) {
            float ax, ay, az;
            if (self->imu_->ReadAccel(ax, ay, az)) {
                int64_t now_ms = esp_timer_get_time() / 1000;

                if (has_prev) {
                    float dx = fabsf(ax - prev_ax);
                    float dy = fabsf(ay - prev_ay);
                    float dz = fabsf(az - prev_az);
                    float shake_magnitude = dx + dy + dz;

                    if (shake_magnitude > kShakeAccelThreshold &&
                        (now_ms - last_shake_ms) > kShakeCooldownMs) {
                        last_shake_ms = now_ms;
                        self->OnShakeDetected();
                    }

                    float tilt_x = fabsf(ax);
                    float tilt_y = fabsf(ay);

                    if ((tilt_x > kTiltThreshold || tilt_y > kTiltThreshold) &&
                        (now_ms - last_tilt_ms) > kTiltCooldownMs) {
                        last_tilt_ms = now_ms;
                        self->OnTiltDetected(ax, ay);
                    }
                }
                prev_ax = ax;
                prev_ay = ay;
                prev_az = az;
                has_prev = true;
            }
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }

    void OnShakeDetected() {
        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();

        if (state != kDeviceStateIdle || shake_pending_) {
            return;
        }
        shake_pending_ = true;

        // Schedule on main thread: reset sleep state, restore brightness,
        // then play the shake GIF (without waking into listening).
        app.Schedule([this]() {
            shake_pending_ = false;
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() != kDeviceStateIdle) {
                return;
            }
            // Reset sleep countdown and exit sleep mode if sleeping (restores brightness)
            power_save_timer_->WakeUp();
            // Play shake GIF
            auto* display = dynamic_cast<CustomLcdDisplay*>(Board::GetInstance().GetDisplay());
            if (display) {
                display->pending_wake_source_ = CustomLcdDisplay::WakeSource::SHAKE_IMU;
                display->SetEmotion("neutral");
            }
        });
    }

    void OnTiltDetected(float ax, float ay) {
        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();

        if (state != kDeviceStateIdle) {
            return;
        }
        // Could trigger different animations based on tilt direction
        // For now, no action - implement specific animations as needed
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }


public:
    WaveshareEsp32s3TouchLcd1_85B() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializePowerSaveTimer();
        InitializeCodecI2c();
        st77916_reset();
        InitializeSpi();
        Initializest77916Display();
        InitializeTouch();
        InitializeQmi8658();
        InitializeButtons();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(WaveshareEsp32s3TouchLcd1_85B);
