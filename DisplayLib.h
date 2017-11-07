// DisplayLib.h
// interfacing 128x64 OLED display with I2C (pin A4 and A5 UNO)
// note: ensure time between each display is at least 40ms

#ifndef _DISPLAYLIB_h
#define _DISPLAYLIB_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include<Wire.h>

//=============================================================================
//defines
//=============================================================================
// Some defines for the SSD1306 controller driving a 128x64 resolution OLED display
// PART     - http://www.simplelabs.co.in/content/96-blue-i2c-oled-module
// DATASHEET  - https://www.adafruit.com/datasheets/SSD1306.pdf


// The Slave Address (SLA) of the OLED controller - SSD1306 - is 0x3C 
// The LSB is supposed to be the mode. Since we are only going to WRITE to the OLED, the LSB is going to be 0
// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define OLED_I2C_ADDRESS   0x3C

// The SSD1306 datasheet (pg.20) says that a control byte has to be sent before sending a command
// Control byte consists of 
// bit 7    : Co   : Continuation bit - If 0, then it assumes all the next bytes are data (no more control bytes).
//        :    You can send a stream of data, ie: gRAM dump - if Co=0
//        :        For Command, you'd prolly wanna set this - one at a time. Hence, Co=1 for commands
//        :    For Data stream, Co=0 :)
// bit 6      : D/C# : Data/Command Selection bit, Data=1/Command=0
// bit [5-0]  : lower 6 bits have to be 0
#define OLED_CONTROL_BYTE_CMD_SINGLE  0x80
#define OLED_CONTROL_BYTE_CMD_STREAM  0x00
#define OLED_CONTROL_BYTE_DATA_STREAM 0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST     0x81  // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM      0xA4
#define OLED_CMD_DISPLAY_ALLON      0xA5
#define OLED_CMD_DISPLAY_NORMAL     0xA6
#define OLED_CMD_DISPLAY_INVERTED     0xA7
#define OLED_CMD_DISPLAY_OFF      0xAE
#define OLED_CMD_DISPLAY_ON       0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE 0x20  // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE   0x21  // can be used only in HORZ/VERT mode - follow with 0x00 + 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE     0x22  // can be used only in HORZ/VERT mode - follow with 0x00 + 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP    0xA0//0xA1  
#define OLED_CMD_SET_MUX_RATIO      0xA8  // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE    0xC0//0xC8  
#define OLED_CMD_SET_DISPLAY_OFFSET   0xD3  // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP    0xDA  // follow with 0x12
#define OLED_CMD_NOP          0xE3  // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV  0xD5  // follow with 0x80
#define OLED_CMD_SET_PRECHARGE      0xD9  // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT    0xDB  // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP    0x8D  // follow with 0x14



//=============================================================================
//variables
//=============================================================================

// 128x64 = 8192 bits = 1024 bytes in videoMemory, 1 bit for each pixel
static unsigned char videoMemory[1024];

//=============================================================================
//functions display comm
//=============================================================================

void OLED_init();// Init the I2C interface (pins A4 and A5 on the Arduino Uno board) in Master Mode.

void display();//updates the videomemory of the display

void resetMemSetting();//used by display() after each update of display memory to reset row and col count

void resetDisplay();//blanks the display

//=============================================================================
//functions raster graphics
//=============================================================================

uint16_t videoMemoryOffset(uint8_t x, uint8_t y);//given  the pixel coordinates of the screen: returns the byte address (index) in memory (videoMemory array) where the pixel bit is located 

uint8_t bitmask(uint8_t y);//given the y-coordinate of the desired pixel: returns the bitmask so the bit inside the byte in memory can be pinpointed

void setPixel(uint8_t x, uint8_t y);//given the pixel coordinates: sets the bit in memory coresponding to that pixel

void resetPixel(uint8_t x, uint8_t y);//given the pixel coordinates: resets the bit in memory coresponding to that pixel to 0 regardless of previous state

bool pixelIsSet(uint8_t x, uint8_t y);//returns true if the given pixel is set


//=============================================================================
//functions geometry
//=============================================================================

void makeLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);//draws a line in videoMemory using Bresenham’s straight line algorithm, does NOT display

void makeCircle(uint8_t x_c, uint8_t y_c, uint8_t radius);//draws a circle in videoMemory, does NOT display

enum spiralDirection
{
	IN,
	OUT,
	NONE
};

//draws a polygon in videoMemory, does NOT display
void makePolygonSpiral(uint8_t x_c, uint8_t y_c, uint8_t x_s, uint8_t y_s, uint8_t nrOfCorners, spiralDirection dir, uint8_t nrOfTurns);

//=============================================================================
//functions math operator
//=============================================================================
int8_t sign(int8_t val);//returns the sign of val

void complexProduct(uint8_t (&points)[2][2]);//takes a 4x4 matrix with 1 2D point in each row and leftshifts the columns adding the complex product as the rightmost column



#endif

