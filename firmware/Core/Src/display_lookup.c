/*
 * display_lookup.c
 *
 *  Created on: Dec 11, 2023
 *      Author: Guim
 *
 *  With the aid of https://github.com/alexeychurchill/SSD1306Fnt
 */


#include <display_lookup.h>


static const uint8_t logo[] = { // 83 x 64
		0x53, 0x40,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x08, 0x08, 0x0c, 0x0c, 0x0c, 0x0c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c,
		0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
		0x0c, 0x0c, 0x0c, 0x0c, 0x08, 0x08, 0x08, 0x18, 0x18, 0x18, 0x18, 0x10, 0x10, 0x10, 0x30, 0x30,
		0x30, 0x20, 0x20, 0x60, 0x60, 0x40, 0xc0, 0xc0, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x06,
		0x7c, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0x40, 0x40, 0x40, 0x40,
		0x40, 0x40, 0xc0, 0xc0, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x40, 0x40, 0x20, 0x20, 0x00, 0x90, 0x90, 0x48, 0x48,
		0x04, 0x26, 0x13, 0x08, 0xc3, 0xf2, 0x3c, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x87, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03, 0x02, 0x02,
		0x02, 0x06, 0x1e, 0x16, 0x16, 0x14, 0x24, 0x64, 0xc4, 0x84, 0x04, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x08, 0x08, 0x08, 0x1c, 0x06, 0x02, 0x02, 0x02, 0x0a, 0x0a,
		0x12, 0x13, 0x11, 0x11, 0x11, 0x19, 0x1c, 0x16, 0x92, 0x42, 0x32, 0x02, 0x02, 0x01, 0x01, 0x80,
		0x00, 0x00, 0xc0, 0x40, 0x60, 0x18, 0xfe, 0x7f, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
		0x78, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08,
		0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x40, 0x00, 0x80, 0x80, 0x80, 0x80, 0x40, 0x40, 0x40, 0x40,
		0x20, 0x20, 0x20, 0x10, 0x10, 0x18, 0x08, 0x08, 0x04, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x80,
		0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3f, 0x7f, 0x7c, 0xe0, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x01, 0x3f, 0xce, 0x88, 0x10, 0x30, 0x60, 0xc0, 0x80, 0x80, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xc0, 0xc0, 0x60, 0x20, 0x30, 0x30, 0x18,
		0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x20, 0x44, 0x04, 0x04, 0x04, 0x80, 0x82, 0x82, 0x82, 0x83,
		0x83, 0x05, 0x05, 0x04, 0x04, 0x04, 0x04, 0x08, 0x88, 0xc8, 0x68, 0x38, 0xf0, 0x3f, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x02, 0x04, 0x0c, 0x08,
		0x19, 0x19, 0x1b, 0x12, 0x26, 0x26, 0x24, 0x24, 0x6c, 0x4c, 0x4c, 0x48, 0x48, 0xc8, 0x88, 0x88,
		0x88, 0x88, 0x88, 0x8c, 0x8c, 0x8c, 0x84, 0x86, 0x82, 0x83, 0x81, 0x81, 0x80, 0x80, 0x80, 0x80,
		0x80, 0xc0, 0xc0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x60, 0x60, 0x60, 0x20, 0x20,
		0xb0, 0xb0, 0x90, 0xd8, 0xd8, 0xc8, 0x6c, 0x66, 0x26, 0x33, 0x31, 0x19, 0x1c, 0x0e, 0x07, 0x03,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x11, 0x10, 0x10, 0x10, 0x00, 0x08, 0x08,
		0x08, 0x08, 0x08, 0x0c, 0x0c, 0x0c, 0x0c, 0x04, 0x04, 0x04, 0x06, 0x06, 0x06, 0x02, 0x02, 0x03,
		0x03, 0x03, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t ssd1306_roboto_glyph_data_deg[] = {
		0x08, 0x20,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x7e, 0xe7, 0xe7, 0xe7, 0xe7, 0x7e, 0x3c,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t ssd1306_c_glyph_data_67[] = {
	0x12u, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u, 0xC0u, 0xF0u,
	0xF0u, 0x38u, 0x18u, 0x1Cu, 0x1Cu, 0x0Cu, 0x0Cu, 0x1Cu,
	0x1Cu, 0x1Cu, 0x38u, 0xF8u, 0xF0u, 0xE0u, 0xFEu, 0xFFu,
	0xFFu, 0x03u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u, 0x01u,
	0x00u, 0x07u, 0x0Fu, 0x1Fu, 0x3Cu, 0x70u, 0x60u, 0xE0u,
	0xE0u, 0xC0u, 0xC0u, 0xE0u, 0xE0u, 0xE0u, 0x70u, 0x7Eu,
	0x3Eu, 0x1Eu
};

static const uint8_t ssd1306_digits_glyph_data_48[] = {
	0x0Eu, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0xC0u, 0xE0u, 0xF0u, 0x78u, 0x1Cu, 0x1Cu, 0x1Cu, 0x0Cu,
	0x1Cu, 0x1Cu, 0x38u, 0xF8u, 0xF0u, 0xC0u, 0xFFu, 0xFFu,
	0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0xFFu, 0xFFu, 0xFFu, 0x0Fu, 0x1Fu, 0x3Fu, 0x78u,
	0xE0u, 0xE0u, 0xE0u, 0xC0u, 0xE0u, 0xE0u, 0x70u, 0x7Fu,
	0x3Fu, 0x0Fu
};

static const uint8_t ssd1306_digits_glyph_data_49[] = {
	0x0Au, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x18u, 0x18u, 0x18u, 0xFCu,
	0xFCu, 0xFCu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u,
	0xC0u, 0xC0u, 0xC0u, 0xFFu, 0xFFu, 0xFFu, 0xC0u, 0xC0u,
	0xC0u, 0xC0u
};

static const uint8_t ssd1306_digits_glyph_data_50[] = {
	0x0Fu, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0xC0u, 0xE0u, 0xF0u, 0x78u, 0x1Cu, 0x1Cu, 0x1Cu,
	0x0Cu, 0x1Cu, 0x1Cu, 0x3Cu, 0xF8u, 0xF0u, 0xE0u, 0x00u,
	0x01u, 0x01u, 0x01u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,
	0xE0u, 0xF0u, 0x78u, 0x3Fu, 0x1Fu, 0x07u, 0x00u, 0x00u,
	0xE0u, 0xF0u, 0xF8u, 0xDCu, 0xCEu, 0xC7u, 0xC3u, 0xC1u,
	0xC0u, 0xC0u, 0xC0u, 0xF0u, 0xF8u, 0xF8u
};

static const uint8_t ssd1306_digits_glyph_data_51[] = {
	0x0Fu, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0xC0u, 0xF0u, 0xF8u, 0x78u, 0x1Cu, 0x1Cu, 0x0Cu,
	0x0Cu, 0x1Cu, 0x1Cu, 0x38u, 0xF8u, 0xF0u, 0xE0u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x18u, 0x18u, 0x18u,
	0x38u, 0x38u, 0x3Cu, 0xFFu, 0xE7u, 0xC3u, 0x00u, 0x0Cu,
	0x3Cu, 0x7Cu, 0x78u, 0xE0u, 0xE0u, 0xE0u, 0xC0u, 0xE0u,
	0xE0u, 0x70u, 0x7Fu, 0x3Fu, 0x1Fu, 0x02u
};

static const uint8_t ssd1306_digits_glyph_data_52[] = {
	0x10u, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x80u, 0xC0u, 0xF0u, 0xF8u, 0xFCu, 0xFCu, 0xFCu, 0x00u,
	0x00u, 0x00u, 0x00u, 0x80u, 0xE0u, 0xF0u, 0x7Cu, 0x1Eu,
	0x0Fu, 0x03u, 0x01u, 0x00u, 0xFFu, 0xFFu, 0xFFu, 0x00u,
	0x00u, 0x00u, 0x07u, 0x07u, 0x07u, 0x06u, 0x06u, 0x06u,
	0x06u, 0x06u, 0xC6u, 0xC6u, 0xFFu, 0xFFu, 0xFFu, 0xC6u,
	0xC6u, 0xC6u
};

static const uint8_t ssd1306_digits_glyph_data_53[] = {
	0x0Eu, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x80u, 0xFCu, 0xFCu, 0x3Cu, 0x0Cu, 0x0Cu, 0x0Cu,
	0x0Cu, 0x0Cu, 0x0Cu, 0x0Cu, 0x3Cu, 0x3Cu, 0x00u, 0x1Fu,
	0x3Fu, 0x1Fu, 0x0Cu, 0x0Eu, 0x06u, 0x06u, 0x0Eu, 0x0Eu,
	0x1Cu, 0xFCu, 0xF8u, 0xE0u, 0x04u, 0x3Cu, 0x7Cu, 0x78u,
	0xE0u, 0xE0u, 0xE0u, 0xC0u, 0xE0u, 0xE0u, 0x70u, 0x7Fu,
	0x3Fu, 0x1Fu
};

static const uint8_t ssd1306_digits_glyph_data_54[] = {
	0x0Fu, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0xC0u, 0xF0u, 0xF0u, 0x38u, 0x1Cu, 0x1Cu,
	0x1Cu, 0x0Cu, 0x0Cu, 0x1Cu, 0x1Cu, 0x08u, 0x00u, 0x00u,
	0xFFu, 0xFFu, 0xFFu, 0x1Cu, 0x0Eu, 0x06u, 0x06u, 0x06u,
	0x06u, 0x0Eu, 0x1Eu, 0x7Cu, 0xFCu, 0xF0u, 0xC0u, 0x07u,
	0x1Fu, 0x3Fu, 0x78u, 0x70u, 0xE0u, 0xE0u, 0xC0u, 0xE0u,
	0xE0u, 0x70u, 0x7Eu, 0x3Fu, 0x1Fu, 0x03u
};

static const uint8_t ssd1306_digits_glyph_data_55[] = {
	0x10u, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x7Cu, 0x7Cu, 0x7Cu, 0x0Cu, 0x0Cu, 0x0Cu,
	0x0Cu, 0x0Cu, 0x0Cu, 0x0Cu, 0x8Cu, 0xCCu, 0xFCu, 0x7Cu,
	0x3Cu, 0x0Cu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0xC0u, 0xF0u, 0xFCu, 0x3Fu, 0x0Fu, 0x03u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFCu,
	0xFFu, 0xFFu, 0x03u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u
};

static const uint8_t ssd1306_digits_glyph_data_56[] = {
	0x0Fu, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0xE0u, 0xF0u, 0xF8u, 0x38u, 0x1Cu, 0x1Cu,
	0x0Cu, 0x1Cu, 0x1Cu, 0x1Cu, 0x78u, 0xF8u, 0xF0u, 0xC0u,
	0x00u, 0xC3u, 0xE7u, 0xEFu, 0x7Eu, 0x3Cu, 0x38u, 0x18u,
	0x38u, 0x38u, 0x3Cu, 0x7Fu, 0xE7u, 0xC7u, 0x81u, 0x06u,
	0x1Fu, 0x3Fu, 0x7Fu, 0xF0u, 0xE0u, 0xE0u, 0xC0u, 0xC0u,
	0xE0u, 0xE0u, 0x70u, 0x7Fu, 0x3Fu, 0x1Fu
};

static const uint8_t ssd1306_digits_glyph_data_57[] = {
	0x0Fu, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x80u, 0xE0u, 0xF0u, 0xF8u, 0x38u, 0x1Cu, 0x1Cu,
	0x0Cu, 0x1Cu, 0x1Cu, 0x18u, 0x78u, 0xF0u, 0xE0u, 0x80u,
	0x07u, 0x3Fu, 0x7Fu, 0x78u, 0xE0u, 0xC0u, 0xC0u, 0xC0u,
	0xC0u, 0xC0u, 0xE0u, 0x70u, 0xFFu, 0xFFu, 0xFFu, 0x00u,
	0x00u, 0x40u, 0xE0u, 0xE0u, 0xE0u, 0xC0u, 0xE0u, 0xE0u,
	0xE0u, 0x70u, 0x7Eu, 0x3Fu, 0x1Fu, 0x03u
};

static const uint8_t ssd1306_outputs_glyph_data_37[] = {
	0x15u, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0xF0u,
	0xFCu, 0x9Cu, 0x06u, 0x06u, 0x06u, 0x06u, 0xFCu, 0xFCu,
	0xE0u, 0x00u, 0x00u, 0x80u, 0xC0u, 0xF0u, 0x38u, 0x10u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x03u, 0x07u, 0x06u,
	0x0Cu, 0x06u, 0x06u, 0x87u, 0xC3u, 0xF0u, 0x38u, 0x1Eu,
	0x87u, 0xC3u, 0xC0u, 0xC0u, 0xC0u, 0xC0u, 0xC0u, 0x80u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x30u, 0x38u, 0x1Eu,
	0x07u, 0x03u, 0x00u, 0x00u, 0x1Eu, 0x7Fu, 0xFFu, 0xC0u,
	0xC0u, 0xC0u, 0xC0u, 0xFFu, 0x7Fu, 0x1Eu
};

static const uint8_t ssd1306_dot_glyph_data_46[] = {
	0x03u, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0xE0u, 0xE0u, 0xE0u
};

static const uint8_t white[] = {
	0x00u, 0x00, 0x00u
};

static const uint8_t* const ssd1306_digits_glyph_table[] = {
	ssd1306_dot_glyph_data_46,
	ssd1306_digits_glyph_data_48,
	ssd1306_digits_glyph_data_49,
	ssd1306_digits_glyph_data_50,
	ssd1306_digits_glyph_data_51,
	ssd1306_digits_glyph_data_52,
	ssd1306_digits_glyph_data_53,
	ssd1306_digits_glyph_data_54,
	ssd1306_digits_glyph_data_55,
	ssd1306_digits_glyph_data_56,
	ssd1306_digits_glyph_data_57,
	ssd1306_c_glyph_data_67,
	ssd1306_roboto_glyph_data_deg,
	ssd1306_outputs_glyph_data_37,
	logo,
	white
};

static uint8_t ssd1306_get_glyph_index(uint8_t utf8_code) {
	if (utf8_code == '.'){
		return 0;
	}
	else if (48 <= utf8_code && utf8_code <= 57){
		return utf8_code - 47;
	}
	else if (utf8_code == 'C') {
		return 11;
	}
	else if (utf8_code == 'd') {
		return 12;
	}
	else if (utf8_code == 'p') {
		return 13;
	}
	else if (utf8_code == 'l'){
		return 14;
	}
	else if (utf8_code == ' '){
		return 15;
	}
	return 14;
}

const uint8_t* ssd1306_get_glyph(uint8_t utf8_code) {
	uint8_t glyph_index = ssd1306_get_glyph_index(utf8_code);
	return ssd1306_digits_glyph_table[glyph_index];
}

