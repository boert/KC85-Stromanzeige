/*
Works on Arduino Uno (ATmega328) with parallel China TFT+touch shield and INA3221 module connected via I2C.

The following librarys are required:
- TouchScreen library
- TFT library
- Timer library
- INA3221 library

*/

// only for special porposes
//#define SIMULATE_VALUES
//#define ENABLE_SCREENSHOT


// TouchScreen  library
// https://github.com/adafruit/Touch-Screen-Library
#include <TouchScreen.h>

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

#define MINPRESSURE 300
#define MAXPRESSURE 1000

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);


// TFT library
// https://github.com/adafruit/TFTLCD-Library

#include <Adafruit_TFTLCD.h>

// TFT defines
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET -1
Adafruit_TFTLCD tft = Adafruit_TFTLCD( LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);


// Timer library
// https://github.com/sadr0b0t/arduino-timer-api
#include <timer-api.h>
#include <timer_setup.h>


// INA3221 library
// https://github.com/switchdoclabs/SDL_Arduino_INA3221
#include <SDL_Arduino_INA3221.h>
SDL_Arduino_INA3221 ina3221;

#define _12P_CHANNEL 3
#define _5P_CHANNEL 2
#define _COMMON_CHANNEL 1


// Color definitions
#define ILI9341_BLACK       0x0000  ///<   0,   0,   0                                                                                                                 
#define ILI9341_NAVY        0x000F  ///<   0,   0, 123                                                                                                                 
#define ILI9341_DARKGREEN   0x03E0  ///<   0, 125,   0                                                                                                                 
#define ILI9341_DARKCYAN    0x03EF  ///<   0, 125, 123                                                                                                                 
#define ILI9341_MAROON      0x7800  ///< 123,   0,   0                                                                                                                 
#define ILI9341_PURPLE      0x780F  ///< 123,   0, 123                                                                                                                 
#define ILI9341_OLIVE       0x7BE0  ///< 123, 125,   0                                                                                                                 
#define ILI9341_LIGHTGREY   0xC618  ///< 198, 195, 198                                                                                                                 
#define ILI9341_DARKGREY    0x7BEF  ///< 123, 125, 123                                                                                                                 
#define ILI9341_BLUE        0x001F  ///<   0,   0, 255                                                                                                                 
#define ILI9341_GREEN       0x07E0  ///<   0, 255,   0                                                                                                                 
#define ILI9341_CYAN        0x07FF  ///<   0, 255, 255                                                                                                                 
#define ILI9341_RED         0xF800  ///< 255,   0,   0                                                                                                                 
#define ILI9341_MAGENTA     0xF81F  ///< 255,   0, 255                                                                                                                 
#define ILI9341_YELLOW      0xFFE0  ///< 255, 255,   0                                                                                                                 
#define ILI9341_WHITE       0xFFFF  ///< 255, 255, 255                                                                                                                 
#define ILI9341_ORANGE      0xFD20  ///< 255, 165,   0                                                                                                                 
#define ILI9341_GREENYELLOW 0xAFE5  ///< 173, 255,  41                                                                                                                 
#define ILI9341_PINK        0xFC18  ///< 255, 130, 198  

#define BACKGROUND ILI9341_BLACK


// types
enum mode_t {
  numbers,
  graph,
  power
};


// global vars
mode_t mode;
float voltage_ch1;
float current_ch1;
float voltage_ch2;
float current_ch2;
float voltage_ch3;
float current_ch3;

bool graph_redraw;
bool first_point;
float x;
float ox1, oy1;
float ox2, oy2;
float ox3, oy3;
volatile bool timer_tick;


void setup() {
  tft.reset();
  delay( 5);
  tft.begin( tft.readID());
  tft.setRotation( 3);
  tft.setTextSize( 2);
  tft.fillScreen( BACKGROUND);

  tft.print( "init TFT: ");
  tft.println( tft.readID(), HEX);

  long serialspeed = 500000;
  tft.print( "init serial: "); tft.print( serialspeed); tft.println( ",8N1");
  Serial.begin( serialspeed);

  tft.println( "init timer: 200 ms");
  timer_init_ISR_5Hz( TIMER_DEFAULT);
  timer_tick = true;

  tft.println("init sensor: INA3221");
  ina3221.begin();

  tft.println( "start application");
  tft.println();
  tft.println( "KC85-Stromanzeige");
  tft.println( "05/2019 boert");

  for ( byte index = 0; index < 20; index++)
  {
    while ( !timer_tick);
    timer_tick = false;
  }

  mode = numbers;
  init_screen( mode, true);

#ifdef SIMULATE_VALUES
  voltage_ch1 = 12.5;
  current_ch1 = 0.055;

  voltage_ch2 = 4.9;
  current_ch2 = 1.34;

  voltage_ch3 = -5.1;
  current_ch3 = 0.009;
#endif
}


void loop() {

  int incoming;

  // check serial
  if (Serial.available() > 0) {
    // read the incoming byte:
    incoming = Serial.read();

#ifdef ENABLE_SCREENSHOT
    if ( incoming == 'S')
    {
      screenshot();
    }
#endif
  }

  // a point object holds x y and z coordinates
  TSPoint p = ts.getPoint();
  // Re-Set A2 A3 8 9 for ILI9341
  reinit_shild_pins();

  // change mode
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
    if ( mode == numbers)
    {
      mode = graph;
    }
    else if ( mode == graph)
    {
      mode = power;
    }
    else
    {
      mode = numbers;
    }
    init_screen( mode, false);
    graph_redraw = true;
    first_point = true;
    x = 0.0;
    timer_tick = true; // redraw
  }

  if (timer_tick)
  {
    timer_tick = false;

    // catch new values
    read_values();

    // update screen, depends on mode
    switch (mode)
    {
      case numbers:
        update_values( voltage_ch1, current_ch1, ILI9341_YELLOW, 0);
        update_values( voltage_ch2, current_ch2, ILI9341_RED, 1);
        update_values( voltage_ch3, current_ch3, ILI9341_BLUE, 2);
        break;

      case graph:
        if ( graph_redraw)
        {
          tft.setTextSize(1);
          tft.setTextColor( ILI9341_YELLOW, BACKGROUND);
          tft.setCursor( 50, 25);
          tft.println("+12V");
          tft.setTextColor( ILI9341_RED, BACKGROUND);
          tft.setCursor( 120, 25);
          tft.println("+5V");
          tft.setTextColor( ILI9341_BLUE, BACKGROUND);
          tft.setCursor( 190, 25);
          tft.println("-5V");
        }
#define XMIN 0.0
#define XMAX 32.0
#define XDIV 5.0
#define YMIN 0.0
#define YMAX 2.0
#define YDIV 0.25
        Graph(tft, x, current_ch1, ox1, oy1, 50, tft.height() - 35, tft.width() - 60,  tft.height() - 70, XMIN, XMAX, XDIV, YMIN, YMAX, YDIV, "Stromaufnahme", "", "", ILI9341_DARKGREY, ILI9341_LIGHTGREY, ILI9341_YELLOW, ILI9341_WHITE, BACKGROUND, graph_redraw, first_point);
        graph_redraw = false;
        Graph(tft, x, current_ch2, ox2, oy2, 50, tft.height() - 35, tft.width() - 60,  tft.height() - 70, XMIN, XMAX, XDIV, YMIN, YMAX, YDIV, "", "", "", ILI9341_DARKGREY, ILI9341_LIGHTGREY, ILI9341_RED,    ILI9341_WHITE, BACKGROUND, graph_redraw, first_point);
        Graph(tft, x, current_ch3, ox3, oy3, 50, tft.height() - 35, tft.width() - 60,  tft.height() - 70, XMIN, XMAX, XDIV, YMIN, YMAX, YDIV, "", "", "", ILI9341_DARKGREY, ILI9341_LIGHTGREY, ILI9341_BLUE,   ILI9341_WHITE, BACKGROUND, graph_redraw, first_point);
        first_point = false;

        if ( x <= XMAX)
        {
          x += 0.2;
        }
        else
        {
          first_point = true;
          graph_redraw = true;
          x = XMIN;
        }
        break;

      case power:
        float power_ch1 = voltage_ch1 * current_ch1;
        float power_ch2 = voltage_ch2 * current_ch2;
        float power_ch3 = voltage_ch3 * current_ch3;
        update_power( power_ch1 + power_ch2 + power_ch3);
        break;
    }
  }
}


void init_screen( mode_t mode, bool first) {
  if (first)
  {
    tft.fillScreen( BACKGROUND);
    // roter Rahmen
    for ( byte index = 0; index < 3; index++)
    {
      tft.drawRect( index, index, tft.width() - index - index, tft.height() - index - index, ILI9341_RED);
    }
  }
  else
  {
    byte index = 4;
    tft.fillRect( index, index, tft.width() - index - index, tft.height() - index - index, BACKGROUND);
  }
  switch ( mode)
  {
    case  numbers:
      print_valueset( "+12V", 0);
      print_valueset( "+5V", 1);
      print_valueset( "-5V", 2);
      break;
    case graph:
      break;
    case power:
      tft.setTextColor( ILI9341_LIGHTGREY);
      tft.setTextSize( 2);
      tft.setCursor( 10, 10);
      tft.println( "Leistungsaufnahme (Summe)");
      tft.setTextColor( ILI9341_WHITE);
      tft.setTextSize( 4);
      tft.setCursor( 282, 78);
      tft.println( "W");
      break;
  }
}


void print_valueset( String name, byte position) {
  const word spaltename = 10;
  const word spaltecurrent = 110;
  const word spaltepower = 210;
  const word zeile_offset = 16;
  const word zeile_abstand = 40;
  word zeile =  zeile_offset + 2 * position * zeile_abstand;

  tft.setTextColor( ILI9341_LIGHTGREY, BACKGROUND);
  tft.setTextSize( 2);

  tft.setCursor( spaltename, zeile);
  tft.print( alignString( name));

  tft.setCursor( spaltecurrent, zeile);
  tft.print( alignString( "A"));

  tft.setCursor( spaltepower, zeile);
  tft.print( alignString( "W"));
}


void update_values( float voltage, float current, word color, byte position) {
  const word spalte_voltage = 15;
  const word spalte_current = 117;
  const word spalte_power = 218;
  const word zeile_offset = 38;
  const word zeile_abstand = 40;
  word zeile = zeile_offset + 2 * position * zeile_abstand;

  tft.setTextColor( color, BACKGROUND);
  tft.setTextSize( 3);
  tft.setCursor( spalte_voltage, zeile);
  tft.println( formatNumber( voltage, 5, 2));

  tft.setCursor( spalte_current, zeile);
  tft.println( formatNumber( current, 5, 2));

  tft.setCursor( spalte_power, zeile);
  tft.println( formatNumber( abs( voltage) * current, 5, 2));
}


void update_power( float power) {
  tft.setTextColor( ILI9341_WHITE, BACKGROUND);
  tft.setTextSize( 8);
  tft.setCursor( 30, 50);
  tft.println( formatNumber( power, 5, 2));
}



String formatNumber(float input, byte columns, byte places) {
  char buffer[20];
  dtostrf(input, columns, places, buffer);
  return ( buffer);
}


String alignString(String input) {
  char buffer[12];
  sprintf(buffer, "%8s", input.c_str());
  return ( buffer);
}


// Re-Set A2 A3 8 9 for ILI9341
inline void  reinit_shild_pins( void) {
  // Pins 7-2 as output, no change for pins 1,0 (RX TX)
  DDRD = DDRD | B11111100;
  // Pins 8-9 as output
  DDRB = DDRB | B00000011;
  DDRC = DDRC | B00011111; // A0-A4 as outputs
}



/*
   Source code:
   Kris Kasprzak
   https://www.youtube.com/watch?v=YejRbIKe6e0
   https://drive.google.com/file/d/0ByEQKtsOckWBNy1GVS1MMm0zSkk/view

  function to draw a cartesian coordinate system and plot whatever data you want
  just pass x and y and the graph will be drawn

  huge arguement list
  &d name of your display object
  x = x data point
  y = y datapont
  gx = x graph location (lower left)
  gy = y graph location (lower left)
  w = width of graph
  h = height of graph
  xlo = lower bound of x axis
  xhi = upper bound of x asis
  xinc = division of x axis (distance not count)
  ylo = lower bound of y axis
  yhi = upper bound of y asis
  yinc = division of y axis (distance not count)
  title = title of graph
  xlabel = x asis label
  ylabel = y asis label
  gcolor = graph line colors
  acolor = axi ine colors
  pcolor = color of your plotted data
  tcolor = text color
  bcolor = background color
  &redraw = flag to redraw graph on fist call only
*/



void Graph(Adafruit_GFX &d, float x, float y, float &ox , float &oy, float gx, float gy, word w, word h, float xlo, float xhi, float xinc, float ylo, float yhi, float yinc, String title, String xlabel, String ylabel, word gcolor, word acolor, word pcolor, unsigned int tcolor, unsigned int bcolor, boolean redraw, bool reset) {

  float ydiv, xdiv;
  // initialize old x and old y in order to draw the first point of the graph
  // but save the transformed value
  // note my transform funcition is the same as the map function, except the map uses long and we need doubles
  //static double ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
  //static double oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  float i;
  float temp;
  int rot, newrot;

  if (reset) {
    ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  }
  if (redraw) {

    // clear diagram area
    d.fillRect( gx, gy - h + 1, w + 1, h + 1, bcolor);

    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        d.drawLine(gx, temp, gx + w, temp, acolor);
      }
      else {
        d.drawLine(gx, temp, gx + w, temp, gcolor);
      }

      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(gx - 40, temp);
      // precision is default Arduino--this could really use some format control
      d.println(formatNumber(i, 0, 2));
    }

    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform
      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        d.drawLine(temp, gy, temp, gy - h, acolor);
      }
      else {
        d.drawLine(temp, gy, temp, gy - h, gcolor);
      }

      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(temp, gy + 10);
      // precision is default Arduino--this could really use some format control
      d.println(formatNumber(i, 0, 0));
    }

    //now draw the labels
    d.setTextSize(2);
    d.setTextColor(tcolor, bcolor);
    d.setCursor(gx , gy - h - 30);
    d.println(title);

    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx , gy + 20);
    d.println(xlabel);

    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx - 30, gy - h - 10);
    d.println(ylabel);
  }

  //graph drawn now plot the data
  // the entire plotting code are these few lines...
  // recall that ox and oy are initialized as static above
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox, oy, x, y, pcolor);
  d.drawLine(ox, oy + 1, x, y + 1, pcolor);
  d.drawLine(ox, oy - 1, x, y - 1, pcolor);
  ox = x;
  oy = y;
}


/**
   Timer interrupt service routine, called with chosen period
   @param timer - timer id
*/
void timer_handle_interrupts(int timer) {
  timer_tick = true;
}


void read_values( void)
{
#ifndef SIMULATE_VALUES
  float busvoltage1 = ina3221.getBusVoltage_V( _12P_CHANNEL);
  float shuntvoltage1 = ina3221.getShuntVoltage_mV( _12P_CHANNEL);
  current_ch1 = ina3221.getCurrent_mA( _12P_CHANNEL) / 1000;
  voltage_ch1 = busvoltage1 + (shuntvoltage1 / 1000);

  float busvoltage2 = ina3221.getBusVoltage_V( _5P_CHANNEL);
  float shuntvoltage2 = ina3221.getShuntVoltage_mV( _5P_CHANNEL);
  current_ch2 = ina3221.getCurrent_mA( _5P_CHANNEL) / 1000 * 0.1 / 0.04;
  voltage_ch2 = busvoltage2 + (shuntvoltage2 / 1000);

  float busvoltage3 = ina3221.getBusVoltage_V( _COMMON_CHANNEL);
  float shuntvoltage3 = ina3221.getShuntVoltage_mV( _COMMON_CHANNEL);
  current_ch3 = ina3221.getCurrent_mA( _COMMON_CHANNEL) / 1000 * 0.1 / 0.04;
  voltage_ch3 = busvoltage3 + (shuntvoltage3 / 1000);

  // fix voltage for KC85
  voltage_ch3 = -voltage_ch3;
  voltage_ch1 = voltage_ch1 + voltage_ch3;
  voltage_ch2 = voltage_ch2 + voltage_ch3;

  // fix currect for KC85
  current_ch1 = - current_ch1;
  current_ch2 = - current_ch2;
  current_ch3 = current_ch3 - current_ch1 - current_ch2;
#endif
}


#ifdef ENABLE_SCREENSHOT
void screenshot( void)
{
  // read whole display an send as PPM (ASCII) via serial line
  // see: https://de.wikipedia.org/wiki/Portable_Anymap

  // header
  Serial.println( "P3");
  Serial.print( tft.width()); Serial.print( " "); Serial.println( tft.height());
  Serial.println( "255"); // range

  uint16_t x, y, color;
  uint8_t r, g, b, i;
  i = 0;
  for ( y = 0; y < tft.height(); y++)
  {
    for ( x = 0; x < tft.width(); x++)
    {
      color = tft.readPixel( x, y);
      color565toRGB( color, r, g, b);
      Serial.print( r); Serial.print( " ");
      Serial.print( g); Serial.print( " ");
      Serial.print( b); Serial.print( " ");
      // add linebreak
      if (i < 6) {
        i++;
      }
      else
      {
        i = 0;
        Serial.println();
      }
    }
  }
}


//Source: https://github.com/PaulStoffregen/ILI9341_t3/blob/master/ILI9341_t3.h
//color565toRGB   - converts 565 format 16 bit color to RGB
static void color565toRGB(uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = (color >> 8) & 0x00F8;
  g = (color >> 3) & 0x00FC;
  b = (color << 3) & 0x00F8;
}
#endif
