/***************************************************
  This is an Arduino Library for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <stdint.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif

#if defined(__AVR__)
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include "ILI9340.h"
#include "Adafruit_ILI9340.h"

/* ---------------------------------------------------------------------------------------
 * Construction
 */

Adafruit_ILI9340::Adafruit_ILI9340(uint8_t cs, uint8_t dc, uint8_t rst):
  Adafruit_GFX(ILI9340_TFTWIDTH, ILI9340_TFTHEIGHT), _cs(cs), _dc(dc), _rst(rst)
{
}

/* ---------------------------------------------------------------------------------------
 * Public Methods 
 */

void Adafruit_ILI9340::begin(void) 
{
  pinMode(_rst, OUTPUT);
  digitalWrite(_rst, LOW);
  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);

  SPI.begin();
#if defined (__AVR__)
  SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
#endif
#if defined (ESP8266)
  SPI.setFrequency(40000000);
#endif
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  // toggle RST low to reset
  digitalWrite(_rst, HIGH);
  delay(5);
  digitalWrite(_rst, LOW);
  delay(20);
  digitalWrite(_rst, HIGH);
  delay(150);

  Initialize();
}

void Adafruit_ILI9340::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1)
{
  writecommand(ILI9340_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART 
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9340_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ILI9340_RAMWR); // write to RAM
}

void Adafruit_ILI9340::pushColor(uint16_t color)
{
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  SPI.transfer(color >> 8);
  SPI.transfer(color);

  digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  digitalWrite(_cs, LOW);

  digitalWrite(_dc, LOW);
  SPI.transfer(ILI9340_CASET); // Column addr set

  digitalWrite(_dc, HIGH);
  SPI.transfer(x >> 8);
  SPI.transfer(x); 
  SPI.transfer((x+1) >> 8);
  SPI.transfer(x+1); 

  digitalWrite(_dc, LOW);
  SPI.transfer(ILI9340_PASET); // Row addr set

  digitalWrite(_dc, HIGH);
  SPI.transfer(y >> 8);
  SPI.transfer(y); 
  SPI.transfer((y+1) >> 8);
  SPI.transfer(y+1); 

  digitalWrite(_dc, LOW);
  SPI.transfer(ILI9340_RAMWR); // write to RAM 

  digitalWrite(_dc, HIGH);
  SPI.transfer(color >> 8);
  SPI.transfer(color);

  digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color)
{
  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
    h = _height-y;

  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  while (h--) {
    SPI.transfer(hi);
    SPI.transfer(lo);
  }
  digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color)
{
  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
  while (w--) {
    SPI.transfer(hi);
    SPI.transfer(lo);
  }
  digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9340::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color)
{
  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      SPI.transfer(hi);
      SPI.transfer(lo);
    }
  }
  digitalWrite(_cs, HIGH);
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9340::Color565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void Adafruit_ILI9340::setRotation(uint8_t m)
{
  writecommand(ILI9340_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
     break;
   case 1:
     writedata(ILI9340_MADCTL_MV | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  case 2:
    writedata(ILI9340_MADCTL_MY | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
    break;
   case 3:
     writedata(ILI9340_MADCTL_MV | ILI9340_MADCTL_MY | ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  }
}

void Adafruit_ILI9340::invertDisplay(boolean i)
{
  writecommand(i ? ILI9340_INVON : ILI9340_INVOFF);
}

/* ---------------------------------------------------------------------------------------
 * Private Methods
 */

typedef struct
{
  bool isCommand;
  uint8_t value;
} tInitEntry;

static tInitEntry initTable[] =
{
  { true, 0xEF },
  { false, 0x03 },
  { false, 0x80 },
  { false, 0x02 },

  { true, 0xCF },
  { false, 0x00 },
  { false, 0xC1 },
  { false, 0x30 },

  { true, 0xED },
  { false, 0x64 },
  { false, 0x03 },
  { false, 0x12 },
  { false, 0x81 },
 
  { true, 0xE8 },
  { false, 0x85 },
  { false, 0x00 },
  { false, 0x78 },

  { true, 0xCB },
  { false, 0x39 },
  { false, 0x2C },
  { false, 0x00 },
  { false, 0x34 },
  { false, 0x02 },
 
  { true, 0xF7 },
  { false, 0x20 },

  { true, 0xEA },
  { false, 0x00 },
  { false, 0x00 },
 
  { true, ILI9340_PWCTR1 }, // Power control
  { false, 0x23 },          // VRH[5:0]
 
  { true, ILI9340_PWCTR2 }, // Power control
  { false, 0x10 },          // SAP[2:0];BT[3:0]
 
  { true, ILI9340_VMCTR1 }, // VCM control
  { false, 0x3e },
  { false, 0x28 },

  { true, ILI9340_VMCTR2 }, // VCM control2
  { false, 0x86 },          // --
 
  { true, ILI9340_MADCTL }, // Memory Access Control
  { false, ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR },

  { true, ILI9340_PIXFMT },
  { false, 0x55 },

  { true, ILI9340_FRMCTR1 },
  { false, 0x00 },
  { false, 0x18 },
 
  { true, ILI9340_DFUNCTR },  // Display Function Control
  { false, 0x08 },
  { false, 0x82 },
  { false, 0x27 },
 
  { true, 0xF2 },             // 3Gamma Function Disable
  { false, 0x00 },
 
  { true, ILI9340_GAMMASET }, // Gamma curve selected
  { false, 0x01 },

  { true, ILI9340_GMCTRP1 },  // Set Gamma
  { false, 0x0F },
  { false, 0x31 },
  { false, 0x2B },
  { false, 0x0C },
  { false, 0x0E },
  { false, 0x08 },
  { false, 0x4E },
  { false, 0xF1 },
  { false, 0x37 },
  { false, 0x07 },
  { false, 0x10 },
  { false, 0x03 },
  { false, 0x0E },
  { false, 0x09 },
  { false, 0x00 },

  { true, ILI9340_GMCTRN1 },  // Set Gamma
  { false, 0x00 },
  { false, 0x0E },
  { false, 0x14 },
  { false, 0x03 },
  { false, 0x11 },
  { false, 0x07 },
  { false, 0x31 },
  { false, 0xC1 },
  { false, 0x48 },
  { false, 0x08 },
  { false, 0x0F },
  { false, 0x0C },
  { false, 0x31 },
  { false, 0x36 },
  { false, 0x0F },
};
#define INIT_TABLE_ENTRIES (sizeof(initTable)/sizeof(initTable[0]))

void Adafruit_ILI9340::Initialize(void)
{
  uint32_t idx;
  for(idx = 0; idx < INIT_TABLE_ENTRIES; ++idx)
  {
    tInitEntry *pEntry = &initTable[idx];
    if(pEntry->isCommand)
      writecommand(pEntry->value);
    else
      writedata(pEntry->value);
  }

  writecommand(ILI9340_SLPOUT);    // Exit Sleep
  delay(120);
  writecommand(ILI9340_DISPON);    // Display on
}

void Adafruit_ILI9340::writecommand(uint8_t c)
{
  digitalWrite(_dc, LOW);
  digitalWrite(_cs, LOW);
  SPI.transfer(c);
  digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340::writedata(uint8_t c)
{
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
  SPI.transfer(c);
  digitalWrite(_cs, HIGH);
}

