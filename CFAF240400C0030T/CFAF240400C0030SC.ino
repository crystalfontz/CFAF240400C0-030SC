//==============================================================================
//  CRYSTALFONTZ 
//==============================================================================
//
//  This code drives the CFAF240400C0-030SC display
//    https://www.crystalfontz.com/product/cfaf240400c0030sc
//
//  The controller is a Ilitek ILI9327
//    https://www.crystalfontz.com/controllers/Ilitek/ILI9327/
//
//  The touch controller is a FocalTech FT6336G
//    https://www.crystalfontz.com/controllers/FocalTech/FT6336G/
//
//  Seeeduino v4.2, an open-source 3.3v capable Arduino clone.
//    https://www.seeedstudio.com/Seeeduino-V4.2-p-2517.html
//    https://github.com/SeeedDocument/SeeeduinoV4/raw/master/resources/Seeeduino_v4.2_sch.pdf
//
//  Code can also be found on GitHub.
//    https://github.com/crystalfontz/CFAF240400c0-030sc
//
//==============================================================================
//
//  2018-11-05 Trevin Jorgenson
//
//==============================================================================
//This is free and unencumbered software released into the public domain.
//
//Anyone is free to copy, modify, publish, use, compile, sell, or
//distribute this software, either in source code form or as a compiled
//binary, for any purpose, commercial or non-commercial, and by any
//means.
//
//In jurisdictions that recognize copyright laws, the author or authors
//of this software dedicate any and all copyright interest in the
//software to the public domain. We make this dedication for the benefit
//of the public at large and to the detriment of our heirs and
//successors. We intend this dedication to be an overt act of
//relinquishment in perpetuity of all present and future rights to this
//software under copyright law.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
//OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//OTHER DEALINGS IN THE SOFTWARE.
//
//For more information, please refer to <http://unlicense.org/>
//==============================================================================

#include <Wire.h>
#include <stdarg.h>
#include <SD.h>
#include <SoftwareSerial.h>

#define RD          14
#define WR          15
#define RS          16
#define CS          17
#define RESET       8
#define TCH_RES     
#define INTERRUPT   9
#define I2C_SDA     18
#define I2C_SCL     19

#define OPTOMIZED_COMMANDS

#ifdef OPTOMIZED_COMMANDS

#define SET_RD      PORTC |=  0x01
#define CLR_RD      PORTC &= ~0x01
#define SET_WR      PORTC |=  0x02
#define CLR_WR      PORTC &= ~0x02
#define SET_RS      PORTC |=  0x04
#define CLR_RS      PORTC &= ~0x04
#define SET_CS      PORTB |=  0x01
#define CLR_CS      PORTB &= ~0x01
#define SET_RESET   PORTC |=  0x08
#define CLR_RESET   PORTC &= ~0x08

#else
#define SET_CS      digitalWrite(CS,HIGH)
#define CLR_CS      digitalWrite(CS,LOW)
#define SET_RS      digitalWrite(RS,HIGH)
#define CLR_RS      digitalWrite(RS,LOW)
#define SET_RD      digitalWrite(RD,HIGH)
#define CLR_RD      digitalWrite(RD,LOW)
#define SET_WR      digitalWrite(WR,HIGH)
#define CLR_WR      digitalWrite(WR,LOW)
#define SET_RESET   digitalWrite(RESET,HIGH)
#define CLR_RESET   digitalWrite(RESET,LOW)
#define SET_BL      digitalWrite(BL,HIGH)
#define CLR_BL      digitalWrite(BL,LOW)
#endif

SoftwareSerial mySerial(18,19);

const uint16_t BLACK = 0x0000;
const uint16_t BLUE = 0x001F;
const uint16_t RED = 0xF800;
const uint16_t GREEN = 0x07E0;
const uint16_t CYAN = 0x07FF;
const uint16_t MAGENTA = 0xF81F;
const uint16_t YELLOW = 0xFFE0;
const uint16_t WHITE = 0xFFFF;

#define LENGTH      400
#define WIDTH       240

//==============================================================================
// LCD & USD control lines
//   ARD      | Port  | CFAF240400C0    |  Function - Parallel        
//------------+-------+-----------------+-----------------------------
//  3.3V      |       | #1-2,33-34,42   |  POWER 3.3V                 
//  GND	      |       | #3,5-8,28,35-41 |  GROUND                     
// -----------+-------+-----------------+-----------------------------
//  A0        | PORTC | #29             |  Read                  (RD) 
//  A1        | PORTC | #30             |  Write                 (WR) 
//  A2        | PORTC | #31             |  Register Select       (RS) 
//  A3        | PORTC | #4              |  Reset              (RESET) 
//  D8        | PORTC | #32             |  Chip Enable Signal    (CS) 
//            | PORTC | #29             |  Back Light            (BL) 
// -----------+-------+-----------------+-----------------------------
//  D0        | PORTD | #26             |  LCD_D10              (DB0)              
//  D1        | PORTD | #25             |  LCD_D11              (DB1)              
//  D2        | PORTD | #24             |  LCD_D12              (DB2)              
//  D3        | PORTD | #23             |  LCD_D13              (DB3)              
//  D4        | PORTD | #22             |  LCD_D14              (DB4)              
//  D5        | PORTD | #21             |  LCD_D15              (DB5)              
//  D6        | PORTD | #20             |  LCD_D16              (DB6)              
//  D7        | PORTD | #19             |  LCD_D17              (DB7)              
// -----------+-------+-----------------+-----------------------------
//==============================================================================
// Cap Touch & USD control lines
//   ARD      | Port  | CFAF240400C0    |  Function - Parallel        
//------------+-------+-----------------+-----------------------------
//  3.3V      |       | #2,3            |  POWER 3.3V                 
//  GND	      |       | #1,8            |  GROUND                     
// -----------+-------+-----------------+-----------------------------
//  D9        | PORTC | #6              |  Interrupt      (INTERRUPT)
//  3.3V      | PORTC | #7              |  Reset          (TCH_RESET) 
// -----------+-------+-----------------+-----------------------------
//  A4/SDA    | PORTC | #5              |  I2C SDA           (I2C_SDA)
//  A5/SCL    | PORTD | #4              |  I2C SCL           (I2C_SCL)              
// -----------+-------+-----------------+-----------------------------
//==============================================================================
//  Note: For I2C protocol to work, pullup resistors need to be attached to SDA and SCL
//        refer to I2C protocol
//  Note: Due to a lack of pins on the Arduino, the Reset pin was set to 3.3v but
//        this should be tied to an available GPIO
//==============================================================================
//To use SD:
#define SD_CS 10
//  ARD       | SD  
// -----------+-----
//  SD_CS     | CS    
//  D11	      | MOSI     
//  D12       | MISO     
//  D13       | CLK     
//==============================================================================
//  This code is demonstrated using 8080 Parallel
//==============================================================================
//  Display Controller Commands Used
#define ILI9327_ENTER_SLEEP               (0x10)
#define ILI9327_EXIT_SLEEP                (0x11)
#define ILI9327_DISPLAY_OFF               (0x28)
#define ILI9327_DISPLAY_ON                (0x29)
#define ILI9327_SET_COL_ADDR              (0x2A)
#define ILI9327_SET_PAGE_ADDR             (0x2B)
#define ILI9327_WR_MEM_START              (0x2C)
#define ILI9327_SET_ADDR_MODE             (0x36)
#define ili9327_SET_PIXEL_FORMAT          (0x3A)
#define ILI9327_PANEL_DRIVE_SETTING       (0xC0)
#define ILI9327_DISPLAY_TIMING            (0xC1)
#define ILI9327_DISPLAY_TIMING_IDLE       (0xC3)
#define ILI9327_FRAME_RATE_CONTROL        (0xC5)
#define ILI9327_GAMMA_SETTING             (0xC8)
#define ILI9327_POWER_SETTING             (0xD0)
#define ILI9327_VCOM_CONTROL              (0xD1)
#define ILI9327_POWER_SETTING_NORMAL      (0xD2)
#define ILI9327_3_GAMMA_FUNCTION_CONTROL  (0xEA)
//============================================================================
//  Touch Screen Controller Commands Used
#define FT_DEVIDE_MODE 			              (0x00)
#define FT_REG_NUM_FINGER                 (0x02)	
#define FT_TP1_REG                        (0X03)	  
#define FT_TP2_REG                        (0X09)		
#define	FT_ID_G_LIB_VERSION               (0xA1)		
#define FT_ID_G_MODE                      (0xA4)   		
#define FT_ID_G_THGROUP	                  (0x80)   		
#define FT_ID_G_PERIODACTIVE	            (0x88)   	
#define ID_G_SPEC_GESTURE_ENABLE	        (0xD0)   	

//============================================================================
//  TOUCH STUFF
//===========================================================================
void sendCommand(uint8_t reg, uint8_t data)
{
  //Start trasmission by sending address 0x38
  Wire.beginTransmission(0x38);
  //Start reading at register 0x02
  Wire.write(reg);
  Wire.write(data);
  //Send the data over
  Wire.endTransmission();
}

void tchSetup()
{
  Wire.begin();
  Wire.setClock(400000);

  //Use this if TCH_RES is attached to a pin)
  //digitalWrite(TCH_RES, LOW);
  //delay(200);
  //digitalWrite(TCH_RES, HIGH);
  //delay(200);

  //Send Device Mode
  sendCommand(FT_DEVIDE_MODE, 0x00);

  //ID_G_MODE set interrupt
  sendCommand(FT_ID_G_MODE, 0x00);
  //  0x00 - Polling Mode
  //  0x01 - Trigger Mode

  //Touch Threshold
  sendCommand(FT_ID_G_THGROUP, 22);

  //Reporting Rate
  sendCommand(FT_ID_G_PERIODACTIVE, 12);
}

void touchDemo()
{
  //draw a little X box
  for (int i = 0; i < 20; i++)
  {
    Fast_Horizontal_Line(220, i, 239, RED);
  }
  drawLine(223, 2, 237, 18, WHITE);
  drawLine(223, 18, 237, 2, WHITE);

  //set the variables used in the demo
  uint8_t
    press1, release1, contact1, touchID1, w1, m1,
    press2, release2, contact2, touchID2, w2, m2;
  uint16_t
    x1, y1,
    x2, y2;

  //start touch demo
  while (1)
  {
    //Check to see if the interrupt is toggled. Since it's in polling mode, it will remain LOW 
    //whenever there is an active touch. If it is in trigger mode, it will pulse
    if (digitalRead(INTERRUPT) == 0)
    {
      //Start trasmission by sending address 0x38
      Wire.beginTransmission(0x38);
      //Start reading at register 0x02
      Wire.write(0x02);
      //Send the data over
      Wire.endTransmission();

      //Read 16 bytes from address 0x38
      Wire.requestFrom(0x38, 16);

      //Register 2 contains the number of points touched
      uint8_t
        pointsTouched;
      pointsTouched = Wire.read() & 0x0F;

      if (0 < pointsTouched)
      {

        //read the data
        ((uint8_t *)(&x1))[1] = Wire.read();
        ((uint8_t *)(&x1))[0] = Wire.read();
        ((uint8_t *)(&y1))[1] = Wire.read();
        ((uint8_t *)(&y1))[0] = Wire.read();
        w1 = Wire.read();
        m1 = Wire.read();

        //crunch the data
        press1 = ((x1 & 0xC000) == 0x0000) ? 1 : 0;
        release1 = ((x1 & 0xC000) == 0x4000) ? 1 : 0;
        contact1 = ((x1 & 0xC000) == 0x8000) ? 1 : 0;
        touchID1 = y1 & 0xE0000;
        x1 &= 0x0FFF;
        y1 &= 0x0FFF;

        //check for two points touched
        if (1 < pointsTouched)
        {
          ((uint8_t *)(&x2))[1] = Wire.read();
          ((uint8_t *)(&x2))[0] = Wire.read();
          ((uint8_t *)(&y2))[1] = Wire.read();
          ((uint8_t *)(&y2))[0] = Wire.read();
          w2 = Wire.read();
          m2 = Wire.read();


          press2 = ((x2 & 0xC000) == 0x0000) ? 1 : 0;
          release2 = ((x2 & 0xC000) == 0x4000) ? 1 : 0;
          contact2 = ((x2 & 0xC000) == 0x8000) ? 1 : 0;
          touchID2 = y2 & 0xE0000;
          x2 &= 0x0FFF;
          y2 &= 0x0FFF;
          drawPixel(240 - x2, y2, YELLOW);
        }
        drawPixel(240 - x1, y1, WHITE);

        //break out of the loop if the little X box is clicked
        if ((x1 < 20 && y1 < 20) || (x2 < 20 && y2 < 20))
          break;
      }

#ifdef TOUCH_DEBUG
      //print out the data
      SerPrintFF(F("G = 0x%02X P = %d, (%5u,%5u) p=%3u r=%3u c=%3u ID=%3u"),
        gesture,
        pointsTouched,
        x1, y1,
        press1,
        release1,
        contact1,
        touchID1);
      SerPrintFF(F(" (%5u,%5u) p=%3u r=%3u c=%3u ID=%3u \n"),
        x2, y2,
        press2,
        release2,
        contact2,
        touchID2);
#endif
    }
  }
}
    


//===========================================================================
//  DISPLAY STUFF
//===========================================================================
void writeCommand(uint8_t command)
{
  CLR_CS; //Assert chip select
  CLR_RS; //Select command register
  CLR_WR; //Assert write mode

  PORTD = command;

  SET_WR; //De-assert write mode indicating command is "ready" to be read
  SET_CS; //De-assert chip select
}

void writeData(uint8_t data)
{
  CLR_CS; //Assert chip select
  SET_RS; //Select data register
  CLR_WR; //Assert write mode

  PORTD = data;

  SET_WR; //De-assert write mode indicating data is "ready" to be read
  SET_CS; //De-assert chip select
}

uint8_t readData()
{
  PORTD = 0x00;
  DDRD = 0x00;
  delay(1);
  uint8_t data;
  CLR_CS; //Assert chip select
  SET_RS; //Select data register
  CLR_RD; //Assert write mode
  SET_RD; //De-assert write mode indicating data is "ready" to be read
  data = PIND;
  SET_CS;
  DDRD = 0xff;
  return data;
}

//==============================================================================
void init_cfaf40400a0(void)
{

  //Deassert the read pin
  SET_RD;

  //Reset LCD Driver
  SET_RESET;
  _delay_ms(1);
  CLR_RESET;
  _delay_ms(10);
  SET_RESET;
  _delay_ms(120);

  //LCD SETTING
  mySerial.println("Read Device ID:");
  deviceIDRead(); //Read the device ID

  writeCommand(ILI9327_EXIT_SLEEP);
  _delay_ms(120);


  writeCommand(ILI9327_VCOM_CONTROL);
  writeData(0x00); 
  writeData(0x58); 
  writeData(0x15); 

  writeCommand(ILI9327_POWER_SETTING);
  writeData(0x07);
  writeData(0x01);
  writeData(0x04);

  writeCommand(ILI9327_SET_ADDR_MODE);
  writeData(0x08); 

  writeCommand(ili9327_SET_PIXEL_FORMAT);
  writeData(0x55);

  writeCommand(ILI9327_DISPLAY_TIMING);
  writeData(0x10);
  writeData(0x10);
  writeData(0x02);
  writeData(0x02);

  writeCommand(ILI9327_PANEL_DRIVE_SETTING); //Set Default Gamma
  writeData(0x00);
  writeData(0x35);
  writeData(0x00);
  writeData(0x00);
  writeData(0x01);
  writeData(0x02);

  writeCommand(ILI9327_FRAME_RATE_CONTROL); //Set frame rate
  writeData(0x02);

  writeCommand(ILI9327_POWER_SETTING_NORMAL); //power setting
  writeData(0x01);
  writeData(0x33);

  writeCommand(ILI9327_GAMMA_SETTING); //Set Gamma
  writeData(0x02);
  writeData(0x77);
  writeData(0x47);
  writeData(0x09);
  writeData(0x09);
  writeData(0x00);
  writeData(0x03);
  writeData(0x00);
  writeData(0x57);
  writeData(0x50);
  writeData(0x00);
  writeData(0x10);
  writeData(0x08);
  writeData(0x80);
  writeData(0x00);

  writeCommand(ILI9327_DISPLAY_ON);//DISPLAY ON
  _delay_ms(25);

  writeCommand(ILI9327_3_GAMMA_FUNCTION_CONTROL); //Enable 3 Gamma
  writeData(0x00);

  //Invert colors
  writeCommand(0x21);

  displayHome();
}

//==============================================================================
uint16_t rgbTo16(uint8_t r, uint8_t g, uint8_t b)
{
  //grab the top 5 bits of red, then the top 6 bits of green, then the top six bits of blue
  uint16_t color;
  color = (r & 0xf8) << 8;
  color |= (g & 0xfc) << 3;
  color |= (b & 0xf8) >> 3;

  return color;
}

//==============================================================================
void SPI_send_pixels_666(int byteCount, uint8_t *pointer)
{
  uint16_t data;

  uint8_t red;
  uint8_t green;
  uint8_t blue;
  while (byteCount != 0)
  {
    data = 0;
    //the first byte received from a .BMP file is red, then green, then blue.
    blue = *pointer;
    pointer++;
    byteCount--;
    green = *pointer;
    pointer++;
    byteCount--;
    red = *pointer;
    pointer++;
    byteCount--;

    data = (red & 0xf8) << 8;
    data |= (green & 0xfc) << 3;
    data |= (blue & 0xf8) >> 3;

    writeColor(data);
  }
}

//==============================================================================
void displayHome(void)
{
  //This function is Hard coded 240x400

  //Set the display width
  writeCommand(ILI9327_SET_COL_ADDR); //0-239 == 0x0000-0x00EF
  writeData(0x00);  //SC8
  writeData(0x00);  //SC7-SC0
  writeData(0x00);  //EC8
  writeData(0xEF);  //EC7-EC0

  //Set the display length
  writeCommand(ILI9327_SET_PAGE_ADDR); //0-399 == 0x0000-0x018F
  writeData(0x00);  //SP8
  writeData(0x00);  //SP7-SP0
  writeData(0x01);  //EP8
  writeData(0x8F);  //EP7-EP0

  writeCommand(ILI9327_WR_MEM_START);
}

//==============================================================================
void writeColor(uint16_t data)
{

  CLR_CS; //Assert chip select
  SET_RS; //Select data register
  CLR_WR; //Assert write mode

  //write to the whole port for parallel
  PORTD = data >> 8;

  SET_WR; //De-assert write mode indicating data is "ready" to be read
  CLR_WR; //Assert write mode

  PORTD = data;

  SET_WR; //De-assert write mode indicating data is "ready" to be read
  SET_CS; //De-assert chip select
}

//==============================================================================
void setDisplayWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  if (x0 > x1)
  {
    uint16_t holder = x1;
    x1 = x0;
    x0 = holder;
  }if (y0 > y1)
  {
    uint16_t holder = y1;
    y1 = y0;
    y0 = holder;
  }
  writeCommand(ILI9327_SET_COL_ADDR); //0-239 == 0x0000-0x00EF
  writeData((x0 >> 8) & 0x01);  //SC8
  writeData(x0 & 0xff);  //SC7-SC0
  writeData((x1 >> 8) & 0x01);  //EC8
  writeData(x1 & 0xff);  //EC7-EC0

  //Set the display length
  writeCommand(ILI9327_SET_PAGE_ADDR); //0-399 == 0x0000-0x018F
  writeData((y0 >> 8) & 0x01);  //SP8
  writeData(y0 & 0xff);  //SP7-SP0
  writeData((y1 >> 8) & 0x01);  //EP8
  writeData(y1 & 0xff);  //EP7-EP0
}

//==============================================================================
void drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  setDisplayWindow(x, y, x, y);

  writeCommand(ILI9327_WR_MEM_START);
  writeColor(color);
}

//==============================================================================
void LCD_Line(uint16_t x0, uint16_t y0,
  uint16_t x1, uint16_t y1, uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t color;
  color = (r & 0xf8) << 8;
  color |= (g & 0xfc) << 3;
  color |= (b & 0xf8) >> 3;
  drawLine(x0, y0, x1, y1, color);

}

//==============================================================================
// Draws a line of a specified color from Point [x0, y0] to [x1, y1]
// From: http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#C
void drawLine(uint16_t x0, uint16_t y0,
  uint16_t x1, uint16_t y1, uint16_t color)
{
  //General case
  if (y0 != y1)
  {
    //find the deltas and slopes
    int16_t dx = abs((int16_t)x1 - (int16_t)x0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t dy = abs((int16_t)y1 - (int16_t)y0);
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = (dx > dy ? dx : -dy) / 2;

    for (;;)
    {
      drawPixel(x0, y0, color);
      if ((x0 == x1) && (y0 == y1))
      {
        break;
      }
      int16_t e2 = err;
      if (e2 > -dx)
      {
        err -= dy;
        x0 = (uint16_t)((int16_t)x0 + sx);
      }
      if (e2 < dy)
      {
        err += dx;
        y0 = (uint16_t)((int16_t)y0 + sy);
      }
    }
  }
  else
  {
    //Optimized for this display
    Fast_Horizontal_Line(x0, y0, x1, color);
  }
}

void Fast_Horizontal_Line(uint16_t x0, uint16_t y, uint16_t x1, uint16_t color)
{

  setDisplayWindow(x0, y, x1, y);

  if (x0 < x1)
  {
    for (int i = x0; i < x1; i++)
    {
      drawPixel(i, y, color);
    }
  }
  else
  {
    for (int i = x1; i < x0; i++)
    {
      drawPixel(i, y, color);
    }
  }
}

//==============================================================================
void fillDisplay(uint16_t shade)
{
  uint16_t i, j;

  //Always return home before writing to the screen
  displayHome();

  for (i = 0; i < LENGTH; i++)
  {
    for (j = 0; j < WIDTH; j++)
    {
      writeColor(shade);
    }
  }
}

void deviceIDRead()
{
  writeCommand(0xef);
  for (uint8_t i = 0; i < 7; i++)
  {
    mySerial.println(readData(), HEX);
  }
}

//==============================================================================
void displayColorBars(void)
{
  uint16_t i, j;

  //Always return home before writing to the screen
  displayHome();

  for (i = 0; i < WIDTH; i++)
  {
    for (j = 0; j < LENGTH; j++)
    {
      if (i>209)        writeColor(BLACK);
      else if (i>179)   writeColor(BLUE);
      else if (i>149)   writeColor(RED);
      else if (i>119)   writeColor(GREEN);
      else if (i>89)    writeColor(CYAN);
      else if (i>59)    writeColor(MAGENTA);
      else if (i>29)    writeColor(YELLOW);
      else              writeColor(WHITE);
    }
  }
}

//==============================================================================
// Draws a circle of a specified color centered at [x, y] with a specified radius
// From: http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
void LCD_Circle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color)
{
  uint8_t x = radius;
  uint8_t y = 0;
  int16_t radiusError = 1 - (int16_t)x;

  while (x >= y)
  {
    //11 O'Clock
    drawPixel(x0 - y, y0 + x, color);
    //1 O'Clock
    drawPixel(x0 + y, y0 + x, color);
    //10 O'Clock
    drawPixel(x0 - x, y0 + y, color);
    //2 O'Clock
    drawPixel(x0 + x, y0 + y, color);
    //8 O'Clock
    drawPixel(x0 - x, y0 - y, color);
    //4 O'Clock
    drawPixel(x0 + x, y0 - y, color);
    //7 O'Clock
    drawPixel(x0 - y, y0 - x, color);
    //5 O'Clock
    drawPixel(x0 + y, y0 - x, color);

    y++;
    if (radiusError < 0)
    {
      radiusError += (int16_t)(2 * y + 1);
    }
    else
    {
      x--;
      radiusError += 2 * (((int16_t)y - (int16_t)x) + 1);
    }
  }
}

//==============================================================================
void circleDemo(uint8_t width, uint8_t height)
{
  uint8_t w_section = width / 6;
  uint8_t h_section = height / 6;

  //Draw a cyan circle
  LCD_Circle(w_section * 3, h_section * 3, (w_section * 3) - 1, CYAN);
  //Draw a green circle
  LCD_Circle(w_section * 3, h_section * 3, w_section - 1, GREEN);
  //Draw a white circle
  LCD_Circle(w_section * 1, h_section * 3, w_section - 1, WHITE);
  //Draw a red circle
  LCD_Circle(w_section * 5, h_section * 3, w_section - 1, RED);
  //Draw a purple circle
  LCD_Circle(w_section * 3, h_section * 5, w_section - 1, MAGENTA);
  //Draw a orange circle
  LCD_Circle(w_section * 3, h_section * 1, w_section - 1, rgbTo16(0xff,0xa5,0x00));
  delay(1000);

  Serial.println("draw some circles");
  for (uint8_t i = 2; i < (w_section * 3); i += 2)
  {
    LCD_Circle(i + 2, (h_section * 3), i, rgbTo16( i << 2, 0xFF - (i << 2),0x00 ));
  }
}

//==============================================================================
void lines(void)
{
  //draw lines across the screen
  //lines start at the middle of the screen: half the width & half the length
  mySerial.println("cheesy lines");
  int x;
  int y;
  uint8_t r = 0xff;
  uint8_t g = 0x00;
  uint8_t b = 0x80;
  for (x = 0; x < WIDTH; x++)
  {
    //span the bottom section of the screen
    LCD_Line(WIDTH/2, LENGTH/2,
      x, 0,
      r++, g--, b += 2);
  }
  for (y = 0; y < LENGTH; y++)
  {
    //span the right side section of the screen
    LCD_Line(WIDTH/2, LENGTH/2,
      WIDTH-1, y,
      r++, g += 4, b += 2);
  }
  for (x = WIDTH-1; 0 != x; x--)
  {
    //span the top section of the screen
    LCD_Line(WIDTH/2, LENGTH/2,
      x, LENGTH-1,
      r -= 3, g -= 2, b -= 1);
  }
  for (y = LENGTH-1; 0 != y; y--)
  {
    //span the left side section of the screen
    LCD_Line(WIDTH/2, LENGTH/2,
      0, y,
      r + -3, g--, b++);
  }
  delay(1000);
}

//==============================================================================
void show_BMPs_in_root(void)
{
  File root_dir;
  root_dir = SD.open("/");
  if (0 == root_dir)
  {
    //error
  }
  else
  {
    File bmp_file;

    do
    {

      bmp_file = root_dir.openNextFile();
      if (0 == bmp_file)
      {
        // no more files, break out of while()
        // root_dir will be closed below.
        break;
      }
      //Skip directories (what about volume name?)
      else if (0 == bmp_file.isDirectory())
      {
        //The file name must include ".BMP"
        if (0 != strstr(bmp_file.name(), ".BMP"))
        {
          //The BMP must be exactly 172854 long
          //(this is correct for 240x400, 24-bit + ~54)
          if (288050 < bmp_file.size() < 288060)
          {
            //Jump over BMP header. BMP must be 240x240 24-bit
            bmp_file.seek(54);

            displayHome();

            //Since we are limited in memory, break the line up from
            // 240*3 = 720 bytes into three chunks of 240 pixels
            // each 240*3 = 720 bytes.
            //Making this static speeds it up slightly (10ms)
            //Reduces flash size uses less bytes.
            static uint8_t uSDLine[240];
            for (int line = 0; line < (400); line++)
            {
              for (uint8_t line_section = 0; line_section < 3; line_section++)
              {
                //Get a third of the row worth of pixels
                bmp_file.read(uSDLine, 240);
                //Now write this third to the TFT
                SPI_send_pixels_666(240, uSDLine);
              }
            }
          }
        }
      }
      //Release the BMP file handle
      bmp_file.close();
      delay(2000);
    } while (0 == bmp_file);
  }
  //Release the root directory file handle
  root_dir.close();
}

//============================================================================
//  ARDUINO STUFF
//============================================================================
// ref http://playground.arduino.cc/Main/Printf
//
// Example to dump a uint32_t in hex and decimal
// SerPrintFF(F("RAM_G_Unused_Start: 0x%08lX = %lu\n"),RAM_G_Unused_Start,RAM_G_Unused_Start);
// Example to dump a uint16_t in hex and decimal
// SerPrintFF(F("Initial Offest Read: 0x%04X = %u\n"),FT8xx_write_offset ,FT8xx_write_offset);
void SerPrintFF(const __FlashStringHelper *fmt, ...)
{
  char
    tmp[128]; // resulting string limited to 128 chars
  va_list
    args;
  va_start(args, fmt);
  vsnprintf_P(tmp, 128, (const char *)fmt, args);
  va_end(args);
  Serial.print(tmp);
}
//==============================================================================
void setup()
{
  //set pin direction - can also use pinmode();
  DDRC = 0xff;
  DDRB = 0xfd;
  DDRD = 0xff;
  //turn the backlight on - this can be used if the backlight pin is connected
  //SET_BL;

  mySerial.begin(9600);
  mySerial.println("serial initialized");
  //initialize the TFT
  //Serial.begin(115200);
  tchSetup();
  while (0)
  {
    touchDemo();
  }
  init_cfaf40400a0();

  if (!SD.begin(SD_CS))
  {
    //error
  }
   
}

//==============================================================================
// main loop
//
// set/clear defines to run the demos
// BmpDemo requires an SD card
//==============================================================================
#define ColorBarsDemo 1
#define TouchDemo 1
#define LinesDemo 1
#define CircleDemo 1
#define FillDemo 1
#define BmpDemo 0

void loop()
{
  mySerial.println("top of loop");
#if ColorBarsDemo
  displayColorBars();
  delay(1000);
#endif
#if TouchDemo
  //Touch the screen to see it work
  fillDisplay(BLACK);
  touchDemo();
#endif
#if LinesDemo
  lines();
  delay(1000);
#endif
#if CircleDemo
  fillDisplay(BLUE);
  circleDemo(240,240);
  delay(1000);
#endif
#if FillDemo
  fillDisplay(WHITE);
  delay(500);
  fillDisplay(BLACK);
  delay(500);
  fillDisplay(GREEN);
  delay(500);
  fillDisplay(MAGENTA);
  delay(500);
#endif
  //activate this demo if an SD card is connected and a 240x400 .bmp file is stored
#if BmpDemo
  show_BMPs_in_root();
  delay(2000);
#endif

}
