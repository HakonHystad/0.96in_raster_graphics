// 
// 
// 

#include "DisplayLib.h"


//=============================================================================
//functions display comm
//=============================================================================

void OLED_init()
{
	// Init the I2C interface (pins A4 and A5 on the Arduino Uno board) in Master Mode.
	Wire.begin();

	// Set the I2C to HS mode - 400KHz!
	// TWBR = (CPU_CLK / I2C_CLK) -16 /2
	// TWBR = ((16,000,000 / 400,000) - 16) / 2 = 12
	TWBR = 12;

	// keywords:
	// SEG = COL = segment = column byte data on a page
	// Page = 8 pixel tall row. Has 128 SEGs and 8 COMs
	// COM = row

	// Begin the I2C comm with SSD1306's address (SLA+Write)
	Wire.beginTransmission(OLED_I2C_ADDRESS);

	// Tell the SSD1306 that a command stream is incoming
	Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);

	// Follow instructions on pg.64 of the dataSheet for software configuration of the SSD1306
	// Turn the Display OFF
	Wire.write(OLED_CMD_DISPLAY_OFF);
	// Set mux ration tp select max number of rows - 64
	Wire.write(OLED_CMD_SET_MUX_RATIO);
	Wire.write(0x3F);
	// Set the display offset to 0
	Wire.write(OLED_CMD_SET_DISPLAY_OFFSET);
	Wire.write(0x00);
	// Display start line to 0
	Wire.write(OLED_CMD_SET_DISPLAY_START_LINE);

	// Mirror the x-axis. In case you set it up such that the pins are north.
	// Wire.write(0xA0); - in case pins are south - default
	Wire.write(OLED_CMD_SET_SEGMENT_REMAP);

	// Mirror the y-axis. In case you set it up such that the pins are north.
	// Wire.write(0xC0); - in case pins are south - default
	Wire.write(OLED_CMD_SET_COM_SCAN_MODE);

	// Default - alternate COM pin map
	Wire.write(OLED_CMD_SET_COM_PIN_MAP);
	Wire.write(0x12);
	// set contrast
	Wire.write(OLED_CMD_SET_CONTRAST);
	Wire.write(0x7F);
	// Set display to enable rendering from GDDRAM (Graphic Display Data RAM)
	Wire.write(OLED_CMD_DISPLAY_RAM);
	// Normal mode!
	Wire.write(OLED_CMD_DISPLAY_NORMAL);
	// Default oscillator clock
	Wire.write(OLED_CMD_SET_DISPLAY_CLK_DIV);
	Wire.write(0x80);
	// Enable the charge pump
	Wire.write(OLED_CMD_SET_CHARGE_PUMP);
	Wire.write(0x14);
	// Set precharge cycles to high cap type
	Wire.write(OLED_CMD_SET_PRECHARGE);
	Wire.write(0x22);
	// Set the V_COMH deselect volatage to max
	Wire.write(OLED_CMD_SET_VCOMH_DESELCT);
	Wire.write(0x30);
	// Horizonatal addressing mode - same as the KS108 GLCD
	Wire.write(OLED_CMD_SET_MEMORY_ADDR_MODE);
	Wire.write(0x00);
	// Turn the Display ON
	Wire.write(OLED_CMD_DISPLAY_ON);

	// End the I2C comm with the SSD1306
	Wire.endTransmission();
}

void display()
{
	int index = 0;
	for (uint8_t row = 0; row<64; row++)//there are 64 lines on the display
	{
		Wire.beginTransmission(OLED_I2C_ADDRESS);//make communication with slave
		Wire.write(OLED_CONTROL_BYTE_DATA_STREAM);//follwing byte is data

		for (uint8_t col = 0; col<16; col++)//128 bits <=> 16 bytes in 1 line, send 1 line at a time
		{
			Wire.write(videoMemory[index]);  //the graphics ram address of the display increases by one after each byte received
			index++;
		}
		Wire.endTransmission();
	}
	resetMemSetting();//reset the row and col count of the display
}

void resetMemSetting()
{
	// Set the GDDRAM to (Row0, Col0), ie: top-left and establish range as the whole screen - 128x64
	Wire.beginTransmission(OLED_I2C_ADDRESS);
	Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);//following is a commands
											 // column 0 to 127
	Wire.write(OLED_CMD_SET_COLUMN_RANGE);
	Wire.write(0x00);
	Wire.write(0x7F);
	// page 0 to 7
	Wire.write(OLED_CMD_SET_PAGE_RANGE);
	Wire.write(0x00);
	Wire.write(0x07);
	Wire.endTransmission();
}

void resetDisplay()
{
	for (int i = 0; i<1024; i++)
	{
		videoMemory[i] = 0;
	}
	display();
}

//=============================================================================
//functions raster graphics
//=============================================================================

uint16_t videoMemoryOffset(uint8_t x, uint8_t y)
{
	// each byte written writes 8 lines down in 1 column 
	if (x<128 && y<64)
	{
		//offset = column + rowOffset*128 <- 128 bits in 1 row
		uint16_t rowOffset = (y / 8) * 128;//1 byte covers 8 rows
		uint8_t colOffset = x;

		return (rowOffset + colOffset);
	}
	return 0;
}

uint8_t bitmask(uint8_t y)
{
	if (y<64)
	{
		uint8_t posOfBit = y % 8;

		uint8_t initMask = 1;//0000 0001

		uint8_t mask = initMask << posOfBit;//leftshift the set bit to the position of the bit e.g: 0000 0001<<3 -> 0000 1000

		return mask;
	}
	return 0;
}

void setPixel(uint8_t x, uint8_t y)
{
	if (x<128 && y<64)
	{
		uint16_t index = videoMemoryOffset(x, y);

		uint8_t newByte = videoMemory[index] | bitmask(y);//OR together the byte in memory and the bitmask setting the desired bit and leaving the others untouched
		videoMemory[index] = newByte;//place the byte with the set bit in memory
	}
}

void resetPixel(uint8_t x, uint8_t y)
{
	if (x<128 && y<64)
	{
		uint16_t index = videoMemoryOffset(x, y);
		uint8_t newByte = videoMemory[index] & (~bitmask(y));//AND together the byte in memory and the inverted bitmask setting the desired bit to 0 and leaving the others untouched
		videoMemory[index] = newByte;//place the byte with the set bit in memory
	}
}

bool pixelIsSet(uint8_t x, uint8_t y)
{
	if (x<128 && y<64)
	{
		uint16_t index = videoMemoryOffset(x, y);
		uint8_t bitInMemory = videoMemory[index] & bitmask(y);//AND together the byte in memory and the bitmask returning only that bit in it's position of the byte

		return bitInMemory != 0;//true if the bit is set in any position of the memory byte
	}
	return false;
}


//=============================================================================
//functions geometry
//=============================================================================

void makeLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
	int8_t dx = x2 - x1; //diff in x direction
	int8_t dy = y2 - y1; //diff in y direction

	int8_t dx_sign = sign(dx);//the x direction 1 or -1
	int8_t dy_sign = sign(dy);//the y direction 1 or -1

	uint8_t dx_abs = abs(dx);//length in x direction
	uint8_t dy_abs = abs(dy);//length in y direction

	uint8_t largestSpan = max(dx_abs, dy_abs);//the largest diff of the 2 directions

	uint8_t currentX = x1;//current x-coordinate, initialized with start of line
	uint8_t currentY = y1;//current y-coordinate, initialized with start of line

	setPixel(currentX, currentY);//draw the starting point
	bool addPixel;

	int testX = 0, testY = 0;//conditions to see if it is outside of the (largest)span, see loop

	for (int i = 0; i <= largestSpan; i++)//loop over entire span
	{
		testX += dx_abs;//add 1 length in x-direction
		testY += dy_abs;//add 1 length in y-direction

		addPixel = false;

		if (testX>largestSpan)//if the added x-length puts us outside the span, we know the line moves horizontally
		{
			currentX += dx_sign;//move 1 pixel horizontally
			addPixel = true;
			testX -= largestSpan;//reset test condition
		}

		if (testY>largestSpan)//if the added y-length puts us outside the span,  we know the line moves vertically
		{
			currentY += dy_sign;//move 1 pixel vertically
			addPixel = true;
			testY -= largestSpan;//reset test condition
		}
		/*If added length in x/y does not put us outside the largest span it is not as steep in that direction and
		* currentPoint is not updated to plot entirely vertical/horizontal */

		if (addPixel)
		{
			setPixel(currentX, currentY);
		}

	}
}

void makeCircle(uint8_t x_c, uint8_t y_c, uint8_t radius)
{
	uint8_t x = radius;
	uint8_t y = 0;
	uint8_t error = 0;

	while (x>y)//only do 45 degrees
	{
		//and mirror all octants at the same time
		setPixel(x_c + x, y_c + y);
		setPixel(x_c + y, y_c + x);
		setPixel(x_c - y, y_c + x);
		setPixel(x_c - x, y_c + y);
		setPixel(x_c - x, y_c - y);
		setPixel(x_c - y, y_c - x);
		setPixel(x_c + y, y_c - x);
		setPixel(x_c + x, y_c - y);

		/*given integer math define the error E(x,y) = x^2+y^2-r^2*/
		y += 1;
		error += 1 + 2 * y; //if only y is inc the change in error becomes y_change = 2y+1 (from (y+1)^2 - y^2)

		/*since we start horizontal out and inc y the only option for x is to dec or stay*/

		if ((2 * (error - x) + 1)>0)//test to see if dec x -change in x becomes 1-2x (from (x-1)^2-x^2)- holds for 2(E+y_change)+x_change>0 (	from E(x-1,y+1)<E(x,y+1)	)
		{
			x -= 1;//if the error becomes smaller with a dec in x, do it
			error += 1 - 2 * x;//update error
		}

	}
}

void makePolygonSpiral(uint8_t x_c, uint8_t y_c, uint8_t x_s, uint8_t y_s, uint8_t nrOfCorners, spiralDirection dir, uint8_t nrOfTurns)
{
	double angle = (double)(360 / nrOfCorners);
	uint8_t startLength = sqrt((x_s - x_c) ^ 2 + (y_s - y_c) ^ 2);

	uint8_t points[2][2];

	//point 1 = startpoint
	points[0][0] = x_s;
	points[0][1] = y_s;

	//point 2 = 


}

//=============================================================================
//functions math operator
//=============================================================================

int8_t sign(int8_t val)
{
	return (val > 0) - (val < 0);
}

void complexProduct(uint8_t (&points)[2][2])
//takes a 4x4 matrix with 1 2D point in each row and leftshifts the columns adding the complex product as the rightmost column
{
	double p1 = points[0][0] * points[0][1] - points[1][0] * points[1][1];//x1x2-y1y2
	double p2 = points[0][0] * points[1][1] + points[0][1] * points[1][0];//x1y2+x2y1

	points[0][0] = points[0][1];//leftshift row 1
	points[1][0] = points[1][1];//leftshift row 2

	points[0][1] = p1;//add complex product
	points[1][1] = p2;
}

