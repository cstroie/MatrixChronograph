/**
  fntsqr.h - Rectangular 5x5 font

  Copyright (c) 2017-2018 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of LedMatrixClock.

  https://xantorohara.github.io/led-matrix-editor/#00f8888888f80000|0070202020300000|00f808f880f80000|00f880f080f80000|008080f888880000|00f880f808f80000|00f888f808f80000|0080808088f80000|00f888f888f80000|00f880f888f80000
*/

#ifndef FNTSQR_H
#define FNTSQR_H

const uint64_t IMAGES[] PROGMEM = {
  0x00f8888888f80000,
  0x0070202020300000,
  0x00f808f880f80000,
  0x00f880f080f80000,
  0x008080f888880000,
  0x00f880f808f80000,
  0x00f888f808f80000,
  0x0080808088f80000,
  0x00f888f888f80000,
  0x00f880f888f80000
};
const int IMAGES_LEN = sizeof(IMAGES) / 8;

#endif /* FNTSQR_H */
