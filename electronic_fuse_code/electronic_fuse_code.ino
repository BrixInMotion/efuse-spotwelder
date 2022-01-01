/**
 * @file        ElectronicFuseSketch.ino
 * @brief       Code for an electronic fuse application with Infineon BTS7002 high side switch shield.
 * Specs:
 *  - Input voltage: 7-28V DC
 *  - Max. input current: 60A
 *  - Max. output current: 3 x 21A
 *  
 * @copyright   Copyright (c) 2021 Infineon Technologies AG   
 * @version     1.0.2
 * 
 * MIT License
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
 
#include <Arduino.h>
#include <hss-board-arduino.hpp>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// --------------- Pin definitions for Electronic Fuse application ------------------------------------------------------------------------------------

#define SWITCHPIN_HSS1                  4
#define SWITCHPIN_HSS2                  5
#define SWITCHPIN_HSS3                  12

#define ROT_ENCODER_A                   0
#define ROT_ENCODER_B                   A0
#define ROT_ENCODER_BTN                 2
//#define TOGGLE_TESTPIN                  digitalWrite(13, debugPin_Stat); debugPin_Stat = !debugPin_Stat;
#define TOGGLE_TESTPIN

// --------------- Defines / System settings ----------------------------------------------------------------------------------------------------------

#define AMOUNT_MEASUREMENTS             10      // how many measurements are stored in order to calculate a rolling average
#define TRIGGER_DELAY_INTERVAL          50      // [ms] how many milliseconds will td be increased when turning the knob
#define CURRENT_THRESHOLD_INTERVAL      0.2     // [A]  how many Amps will th be increased when turning the knob
#define CURRENT_SUM_THRESHOLD_INTERVAL  0.5     // [A]  how many Amps will the summmarized current threshold be increased when turning the knob
#define DISPLAY_UPDATE_INTERVAL         1000    // [ms] how often will Vbatt and the actual current values be updated on the LCD. Too many refreshes may affect usability of the rotary encoder.
#define MAX_CURRENT_THRESHOLD           21.0    // [A]  the top limit for the current threshold when turning the knob

// -----------------------------------------------------------------------------------------------------------------------------------------------------

#define EEPROM_ADDRESS_TRIGGER_DELAY    0
#define EEPROM_ADDRESS_CUR_THRES_SUM    (sizeof(int))
#define EEPROM_ADDRESS_CUR_THRESHOLD    (sizeof(int) + sizeof(float))  
// -----------------------------------------------------------------------------------------------------------------------------------------------------

enum _rotStatus{
  SCAN,
  WAIT_FOR_RISE
};
enum _cursorMode{
  CHANGE_VALUE,
  SELECT_VALUE
};
enum _EF_status{
  EF_OFF = 0,
  EF_ON,
  EF_OVERCURRENT,
  EF_BTS_ERROR,
  EF_UNDERVOLTAGE
};

hardwareconfig_t EFUSE_PINS
{
    .led1 = GPIOIno::unusedPin,     //LED 1
    .led2 = GPIOIno::unusedPin,     //LED 2
    .led3 = GPIOIno::unusedPin,     //LED 3
    .led4 = GPIOIno::unusedPin,     //LED 4

    .in1 = 9,                       //IN 1
    .in2 = 10,                      //IN 2
    .in3 = 11,                      //IN 3
    .in4 = 3,                       //IN 4

    .oloff = 7,                     //OLOFF

    .den1_den3 = 6,                 //DEN 1_3
    .den2_den4 = 8,                 //DEN 2_4

    .pushButtonDigital = 2,         //PUSHBUTTONDIGITAL

    .pushButtonAnalog = A0,         //PUSHBUTTONANALOG
    .vBat =             A1,         //VBAT
    .is1_is2 =          A2,         //IS 1_2
    .is3_is4 =          A3          //IS 3_4
};

// --------------- Global Variables --------------------------------------------------------------------------------------------------------------------
float         cur_threshold[4] = {0.0, 1.0, 1.0, 1.0};        // Stores the threshold values, at which the HSS switches off (configurable via the LCD)
float         cur_threshold_sum = 20.0;                       // [A] this should be the maximum current which can be delivered by your power supply
float         cur_measurements[4][AMOUNT_MEASUREMENTS];       // Stores the current Values for the last 10 measurements for each HSS
float         cur_measurements_avg[4] = {0.0, 0.0, 0.0, 0.0}; // Stores the average current Values for each HSS
uint8_t       hss_status[4] = {1,0,0,0};                      // Another status array, has nothing to do with the Diagnosis Status (first value is a dummy value)
uint8_t       cur_meas_index[4] = {0,0,0,0};                  // points to the actual position in the mean value measurement array
int           oldRotEncPos, RotEncPos = 0;                    // Position index of the rotary encoder
uint8_t       rotStatus = SCAN;                               // Status of the Rotary encoder (to switch on or off the interrupt routine)
int           trigger_delay = 100;                            // Time in milliseconds, after which the HSS switches off after an overcurrent event.
uint32_t      display_update_timer = 0;                       // counter variable to update the LCD information in constant time intervalls
uint32_t      button_timer = 0;                               // helps to estimate release time for the HID switches
uint32_t      current_measurement_timer = 0;                  // counter variable to scan for an overcurrent event with a specific delay
uint8_t       cursor_mode = SELECT_VALUE;                     // Cursor has to modes: it can either move in the rows to selct a value, or its position is fixed so it can change the value
uint8_t       cursor_row = 0;                                 // [0:3] in which row is the cursor
bool          debugPin_Stat = 0;                              // Status of debug pin
bool          current_exceeded_flag = 0;                      // Set to one as soon as current threshold was exceeded

// ---------------- String arrays (for display) ----------------------------------------------------------------------------------------------------------
const String  hss_status_TXT[4] = {" OFF ", " ON  ", " OVC "};
const String  EF_errorMessages[8] = {"NORMAL", "- OVERLOAD -      ", "- SHORT TO GND -  ", "- OVERTEMPERATURE-", "- SHORT TO VSS -  ", "- OPEN LOAD -     ", "- UNDER LOAD -    ", "- INVERSE CURRENT-"};
//------------------------------------------------------------------------------------------------------------------------------------------------------

HssBoardIno         HSS = HssBoardIno(&BTS7002, EFUSE_PINS);        // create an instance of a high-side switch board with custom pin configuration in order to disable the LEDs
LiquidCrystal_I2C   lcd(0x27,20,4);                                 // create an instance of a LCD display and set the LCD address to 0x27 for a 20 chars and 4 line display

void setup()
{
    Serial.begin(115200);
 
    HSS.init();                                   // Initialization of the High-Side-Switch-Board
    Serial.println("High-Side-Switch is initialized");
    pinMode(ROT_ENCODER_BTN, INPUT_PULLUP);
    pinMode(SWITCHPIN_HSS1, INPUT);
    pinMode(SWITCHPIN_HSS2, INPUT);
    pinMode(SWITCHPIN_HSS3, INPUT);
    pinMode(ROT_ENCODER_A, INPUT);
    pinMode(ROT_ENCODER_B, INPUT);
    pinMode(13, OUTPUT);

    //read thesholds from EEPROM
    uint16_t address = EEPROM_ADDRESS_TRIGGER_DELAY;
    EEPROM.get(address, trigger_delay);
    
    /*
     * When reading the EEPROM the first time, make sure there is no NAN (not a number) value in it
     */
     
    if (isnan(trigger_delay)) {
      for (int i = 0 ; i < EEPROM.length() ; i++) {
        EEPROM.write(i, 0);
      }
    }
    
    if((trigger_delay > 999) || (trigger_delay < 0))    // If EEprom reads garbage, reset to default value
    {
      trigger_delay = 100;
      EEPROM.put(address, trigger_delay);
    }
    address = EEPROM_ADDRESS_CUR_THRES_SUM;
    EEPROM.get(address, cur_threshold_sum);
    
    if (isnan(cur_threshold_sum)) {
      for (int i = 0 ; i < EEPROM.length() ; i++) {
        EEPROM.write(i, 0);
      }
    }
    
    if((cur_threshold_sum > 99.9) ||(cur_threshold_sum < 1.0))
    {
      cur_threshold_sum = 20.0;
      EEPROM.put(address, cur_threshold_sum);
    }
    
    for(uint8_t i=1; i<4; i++)
    { 
      address = EEPROM_ADDRESS_CUR_THRESHOLD + (i-1)*sizeof(float);
      EEPROM.get(address, cur_threshold[i]);
      if((cur_threshold[i] < 0.2) || (cur_threshold[i] > MAX_CURRENT_THRESHOLD))   // If EEprom reads garbage, reset to default value
      {
        cur_threshold[i] = 1.0;
        EEPROM.put(address, cur_threshold[i]);
      }
    }

    /**
     * The LCD shows a startup screen and the maximum current threshold for all three outputs together.
     * The user can either change the value by turning the knob or accept it by pressing the knob.
     */
    lcd.init();
    lcd.backlight();
    lcd.setCursor(3,0);
    lcd.print("Electronic Fuse");
    lcd.setCursor(2,1);
    lcd.print("Infineon BTS7002");
    lcd.setCursor(1,3);
    lcd.print("max current: ");
    if(cur_threshold_sum < 10.0) lcd.print(" ");
    lcd.print(cur_threshold_sum, 1);
    lcd.print("A");
    while(digitalRead(ROT_ENCODER_BTN))
    {
      Interrupt_routine();
      updateSumCurThrs();
    }
    address = EEPROM_ADDRESS_CUR_THRES_SUM;
    EEPROM.put(address, cur_threshold_sum);

    printMenu(4);                           // print menu for all 4 lines
    display_update_timer = millis();
    current_measurement_timer = millis();
}

void loop()
{
    checkTimer();                         // check if its time to update the LCD display

    checkRotSwitch();                     // check if button of rotary encoder was pressed
    Interrupt_routine();                  // check if rotary encoder was turned
    setCursorIndicator();                 // check if cursor position or blink mode needs to be updated on the LCD
    
    for(uint8_t hss = 1; hss <= 3; hss++) // measure the current in all 3 high-side switches
    {
      if(checkCurrent(hss))
      {
        if(!current_exceeded_flag)
        {
          current_measurement_timer = millis();       // Reset timer
          current_exceeded_flag = 1;
        }
      }
    }
    
    TOGGLE_TESTPIN;                     // only necessary for debugging, to see how fast this loop is
    
    if(((millis() - current_measurement_timer) > trigger_delay) && current_exceeded_flag)
    {
       
      // Check the single outputs for overcurrent
      for(uint8_t hss = 1; hss <= 3; hss++)
      {
        if(checkCurrent(hss)){
          HSS.switchHxOff(hss);             // If threshold was exceeded, switch HSS off
          hss_status[hss] = EF_OVERCURRENT;
          printHIDswitchStatonLCD(hss);     //LCD print OVC
          Serial.print("HSS ");
          Serial.print(hss);
          Serial.println(" switched OFF after overcurrent event");
        }
        //checkHSSDiag(hss);                  // See, if another unusual error occured and print error message
      }
      
      // check if summarized current exceeds limit
      float current_sum = cur_measurements_avg[1] + cur_measurements_avg[2] + cur_measurements_avg[3];
      if( current_sum > cur_threshold_sum)
      {
        
        for(uint8_t hss = 1; hss <= 3; hss++)
        {
          HSS.switchHxOff(hss);
          hss_status[hss] = EF_OVERCURRENT;
          printHIDswitchStatonLCD(hss);     //LCD print OVC
        }
      }
      current_exceeded_flag = 0;
    }
 
    checkHIDswitch(SWITCHPIN_HSS1, 1);    // check, if one of the three red buttons is pressed
    checkHIDswitch(SWITCHPIN_HSS2, 2);
    checkHIDswitch(SWITCHPIN_HSS3, 3);
 }

// --------------- Function definitions --------------------------------------------------------------------------------------------------------------------

/**
 * Called periodically.
 * If more than DISPLAY_UPDATE_INTERVAL seconds have passed since the last update, it will update the voltage and current information on the LCD again.
 */
void checkTimer(void)
{
  if(cursor_mode != CHANGE_VALUE)     // make sure, the user doesn't change a value at that moment
  {
    if((millis() - display_update_timer) > DISPLAY_UPDATE_INTERVAL)   // update voltage and current information every second
    {
      printBattVoltonLCD();
      for(uint8_t hss = 1; hss <= 3; hss++)
      {
        if(hss_status[hss] != EF_BTS_ERROR)
        {
          printCuronLCD(hss);
        }
      }
      lcd.setCursor(17, RotEncPos);
      lcd.cursor();
      display_update_timer = millis();
    }
  }
}

/**
 * Called periodically.
 * If the button of the rotary encoder was pressed, toggle the cursor status
 */
void checkRotSwitch(void)
{
  int val = 0;
  if((!digitalRead(ROT_ENCODER_BTN)) && rotStatus == SCAN && cursor_mode == SELECT_VALUE)
  {
    lcd.blink();
    cursor_row = RotEncPos;
    rotStatus = WAIT_FOR_RISE;
    cursor_mode = CHANGE_VALUE;
  }
  else if((!digitalRead(ROT_ENCODER_BTN)) && rotStatus == SCAN && cursor_mode == CHANGE_VALUE)
  {
    lcd.noBlink();
    RotEncPos = cursor_row;
    rotStatus = WAIT_FOR_RISE;
    cursor_mode = SELECT_VALUE;

    
    if(cursor_row == 0)
    {
      EEPROM.put(EEPROM_ADDRESS_TRIGGER_DELAY, trigger_delay);
    }
    else
    {
      uint8_t address = EEPROM_ADDRESS_CUR_THRESHOLD + (cursor_row-1) * sizeof(float);
      EEPROM.put(address, cur_threshold[cursor_row]);
    }
  }
}

/**
 * Called periodically.
 * Change the vertical cursor position or the value where the cursor actually is placed.
 */
void setCursorIndicator(void)
{
  if(oldRotEncPos != RotEncPos)
  {
      oldRotEncPos = RotEncPos;
      if(cursor_mode == SELECT_VALUE)       // Move cursor up and down
      {
        // Limit values
        if(RotEncPos < 0) RotEncPos = 0;
        else if(RotEncPos > 3) RotEncPos = 3;
        lcd.setCursor(17, RotEncPos);
        lcd.cursor();
      }
      else if(cursor_mode == CHANGE_VALUE)    // change the value where the cursor blinks
      {
        if(cursor_row == 0)                   // change time
        {
          trigger_delay += RotEncPos*TRIGGER_DELAY_INTERVAL;
          if(trigger_delay < 0) trigger_delay = 0;
          if(trigger_delay > 990) trigger_delay = 1000 - TRIGGER_DELAY_INTERVAL;
          RotEncPos = 0;
          printTimeonLCD();
        }
        else                                  //change thresholds
        {
          cur_threshold[cursor_row] += RotEncPos*CURRENT_THRESHOLD_INTERVAL;
          if(cur_threshold[cursor_row] < 0.1)  cur_threshold[cursor_row] = 0.2;
          if(cur_threshold[cursor_row] > MAX_CURRENT_THRESHOLD)  cur_threshold[cursor_row] = MAX_CURRENT_THRESHOLD;
          RotEncPos = 0;
          printCurThresholdonLCD(cursor_row);
        }
      }
       
  }
  if(rotStatus == WAIT_FOR_RISE)    // As long as one pin is still tied to low, prohibit the interrupt
  {
    if(digitalRead(ROT_ENCODER_A) && digitalRead(ROT_ENCODER_B) && digitalRead(ROT_ENCODER_BTN)) rotStatus = SCAN;
  }
}

/**
 * Called every trigger_delay - interval.
 * Measures the current in the High side switch and compares it to the threshold.
 */
bool checkCurrent(uint8_t hss)
{
  if(hss_status[hss] == EF_ON)
  {
    float readAmps = HSS.readIsx(hss);
    readAmps = meanValue(hss, readAmps);
    if(readAmps > cur_threshold[hss]) return 1;
  }
  return 0;
}

/*
 * Calculate a rolling mean value
 */
float meanValue(uint8_t hss, float new_cur)
{
  float sum=0.0;
  float mean=0.0;
  cur_measurements[hss][cur_meas_index[hss]] = new_cur;
  for(int k=0; k < AMOUNT_MEASUREMENTS; k++) sum += cur_measurements[hss][k];
  mean =(float) sum / AMOUNT_MEASUREMENTS; 
  cur_meas_index[hss] += 1;
  if(cur_meas_index[hss] >= AMOUNT_MEASUREMENTS) cur_meas_index[hss] = 0;
  cur_measurements_avg[hss] = mean;         // Store the value as well in the global array for later use
  return mean;
}

/**
 * Called periodically.
 * Check if a red button was pressed in order to turn on or off a HSS.
 */
void checkHIDswitch(uint8_t pin, uint8_t hss)
{
  uint32_t buttondelay = millis() - button_timer; 
  if(digitalRead(pin) && (buttondelay > 250)){
    switch (hss_status[hss]){
      case EF_OFF:
        hss_status[hss] = EF_ON;
        HSS.switchHxOn(hss);
        Serial.print("HSS ");
        Serial.print(hss);
        Serial.println(" switched ON");
        printHIDswitchStatonLCD(hss);   //LCD print on
        break;
      case EF_ON:
        hss_status[hss] = EF_OFF;
        HSS.switchHxOff(hss);
        Serial.print("HSS ");
        Serial.print(hss);
        Serial.println(" switched OFF");
        printHIDswitchStatonLCD(hss);     //LCD print off
        cur_measurements_avg[hss] = 0;
        for(uint8_t i = 0; i < AMOUNT_MEASUREMENTS; i++){
          cur_measurements[hss][i] = 0;
        }
        break;
      case EF_OVERCURRENT:
        hss_status[hss] = EF_OFF;
        HSS.switchHxOff(hss);
        printHIDswitchStatonLCD(hss);     //LCD print OVC
        cur_measurements_avg[hss] = 0;
        for(uint8_t i = 0; i < AMOUNT_MEASUREMENTS; i++){
          cur_measurements[hss][i] = 0;
        }
        break;
      default:
        break;
    }
    button_timer = millis();
  }
}

/**
     * The status of the switch can be determined with the following table:
     *      || Diagnosis Status || Description                                                      ||
     *      ------------------------------------------------------------------------------------------
     *      ||          0       || NORMAL = Everything is working correctly                         ||
     *      ||          1       || OVERLOAD = Exceeded the board's current limit                    ||
     *      ||          2       || SHORT_TO_GND = Short the ground of the board                     ||
     *      ||          3       || OVERTEMPERATURE = Board got to hot                               ||
     *      ||          4       || SHORT_TO_VSS = Short to the Battery pad of the board             ||
     *      ||          5       || OPEN_LOAD = No load is connected                                 ||
     *      ||          6       || UNDER_LOAD = Not enough voltage/current to turn on the switch    ||
     *      ||          7       || INVERSE_CURRENT = Inverse current flows into the board           ||
     * 
     * Please note: If you use the diagnosis function when the switch is off and no load is connected
     * the status will be SHORT_TO_GND, because this state is not clear because of the provided IS signal
     * of the board. Read more about this in the data sheet of the PROFET on page 40.
     */
void checkHSSDiag(uint8_t hss)
{
  uint8_t switchStatus = HSS.readDiagx(hss);
  if((switchStatus != 0) && (switchStatus != 5) && (switchStatus != 2))
  {
    lcd.noCursor();
    lcd.setCursor(2, hss);
    lcd.print(EF_errorMessages[switchStatus]);
    hss_status[hss] = EF_BTS_ERROR;
    Serial.print("Diagnosis status of HSS "); Serial.print(hss); Serial.print(": "); Serial.println(switchStatus); 
  }
  else if(hss_status[hss] == EF_BTS_ERROR)     // Status changed from BTS_ERROR to NORMAL again
  {
    hss_status[hss] = EF_OFF;
    printMenu(hss);
  }
}

/**
 * Will look like this:
 *  ____________________
 * |BATT 12.32V td 100ms|
 * |1 ON  18.3A th19.0A |
 * |2 OFF  0.0A th 1.0A |
 * |3 OVC  0.0A th 5.4A |
 *  ____________________
 *  
 *  print either a single line if value [0;3] or all lines [>3]
 */
void printMenu(int row)
{
  float val = 0.0;
  if(row == 0)
  {
    lcd.setCursor(0,0);
    lcd.print("Batt ");
    printBattVoltonLCD();
    lcd.print("V td ");
    printTimeonLCD();
    lcd.print("ms");
  }
  else if(row > 0 && row < 4)
  {
      lcd.setCursor(0,row);
      lcd.print(row);
      printHIDswitchStatonLCD(row);
      printCuronLCD(row);
      lcd.print("A Th");
      printCurThresholdonLCD(row);
      lcd.print("A");
  }
  else
  {
    lcd.clear();
    
    lcd.setCursor(0,0);
    lcd.print("Batt ");
    printBattVoltonLCD();
    lcd.print("V td ");
    printTimeonLCD();
    lcd.print("ms");

    for(int x=1; x<4; x++)
    {
      lcd.setCursor(0,x);
      lcd.print(x);
      printHIDswitchStatonLCD(x);
      printCuronLCD(x);
      lcd.print("A th");
      printCurThresholdonLCD(x);
      lcd.print("A");
    }
  }
}

void printTimeonLCD(void)
{
  lcd.noCursor();
  lcd.setCursor(15, 0);
  if(trigger_delay < 100) lcd.print(" ");
  if(trigger_delay < 10) lcd.print(" ");
  lcd.print(trigger_delay);
}

/**
 * hss [1:3]
 */
void printCurThresholdonLCD(uint8_t hss)
{
  lcd.noCursor();
  lcd.setCursor(14, hss);
  float val = cur_threshold[hss];
  if(val < 9.99) lcd.print(" ");
  lcd.print(val, 1);                              // print float value with one digit after decimal point
}

void printCuronLCD(uint8_t hss)
{
  lcd.noCursor();
  lcd.setCursor(6, hss);
  float val =cur_measurements_avg[hss];
  if(val < 10.0) lcd.print(" ");
  lcd.print(val, 1);                              // print float value with one digit after decimal point
}

void printHIDswitchStatonLCD(uint8_t hss)
{
  lcd.noCursor();
  lcd.setCursor(1, hss);
  lcd.print(hss_status_TXT[ hss_status[hss] ]);
}

void printBattVoltonLCD(void)
{
  lcd.noCursor();
  lcd.setCursor(5, 0);
  float val = HSS.readVss();
    if(val < 10.0) lcd.print(" ");                    // align values with decimal point
  lcd.print(val, 2);
}

/**
 * Only for the startup-screen.
 */
void updateSumCurThrs(void)
{
  if(oldRotEncPos != RotEncPos)
  {
      oldRotEncPos = RotEncPos;
      cur_threshold_sum += RotEncPos * CURRENT_SUM_THRESHOLD_INTERVAL;
      if(cur_threshold_sum < 1.0) cur_threshold_sum = 1.0;
      else if(cur_threshold_sum > 99.9) cur_threshold_sum = 99.9;
      lcd.setCursor(14, 3);
      if(cur_threshold_sum < 10.0) lcd.print(" ");
      lcd.print(cur_threshold_sum, 1);
      RotEncPos = 0;  
  }
  if(rotStatus == WAIT_FOR_RISE)    // As long as one pin is still tied to low, prohibit the interrupt
  {
    if(digitalRead(ROT_ENCODER_A) && digitalRead(ROT_ENCODER_B) && digitalRead(ROT_ENCODER_BTN)) rotStatus = SCAN;
  }
}

/**
 * Read the pins of the rotary encoder
 */
void Interrupt_routine(void)
{
  if(rotStatus == SCAN)          // Only read out, if both pins were high before
  {
    bool A = digitalRead(ROT_ENCODER_A);
    bool B = digitalRead(ROT_ENCODER_B);
    if( (A==1) && (B==0) ) RotEncPos -= 1;
    else if( (A== 0) && (B==1) ) RotEncPos += 1;
    rotStatus = WAIT_FOR_RISE;
  }
}
