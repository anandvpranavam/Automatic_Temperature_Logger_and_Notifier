// ETEC 222 LAB project - TEMPERATURE LOGGER //
// Group Members: 
//ANAND VENUGOPALAPANICKER and SENU GEORGE

#include "pic24_all.h"
#include <stdio.h>
#include<xc.h>
int processingUserinput=0;

//LCD code**********************************************************************
#if (HARDWARE_PLATFORM == EMBEDDED_C1)
//# define RS_HIGH()        (_LATC2 = 1)
//# define RS_LOW()         (_LATC2 = 0)
//# define CONFIG_RS()      CONFIG_RC2_AS_DIG_OUTPUT()
//
//# define RW_HIGH()        (_LATC1 = 1)
//# define RW_LOW()         (_LATC1 = 0)
//# define CONFIG_RW()      CONFIG_RC1_AS_DIG_OUTPUT()
//
// 
//# define E_HIGH()         (_LATC0 = 1)
//# define E_LOW()          (_LATC0 = 0)
//# define CONFIG_E()       CONFIG_RC0_AS_DIG_OUTPUT()
//
//# define LCD4O          (_LATC4)
//# define LCD5O          (_LATC5)
//# define LCD6O          (_LATC6)
//# define LCD7O          (_LATC7)
//# define LCD7I          (_RC7)
//
// 
//# define CONFIG_LCD4_AS_INPUT() CONFIG_RC4_AS_DIG_INPUT()
//# define CONFIG_LCD5_AS_INPUT() CONFIG_RC5_AS_DIG_INPUT()
//# define CONFIG_LCD6_AS_INPUT() CONFIG_RC6_AS_DIG_INPUT()
//# define CONFIG_LCD7_AS_INPUT() CONFIG_RC7_AS_DIG_INPUT()
//
//# define CONFIG_LCD4_AS_OUTPUT() CONFIG_RC4_AS_DIG_OUTPUT()
//# define CONFIG_LCD5_AS_OUTPUT() CONFIG_RC5_AS_DIG_OUTPUT()
//# define CONFIG_LCD6_AS_OUTPUT() CONFIG_RC6_AS_DIG_OUTPUT()
//# define CONFIG_LCD7_AS_OUTPUT() CONFIG_RC7_AS_DIG_OUTPUT()

#else

# define RS_HIGH()        (_LATB12 = 1)
# define RS_LOW()         (_LATB12 = 0)
# define CONFIG_RS()      CONFIG_RB12_AS_DIG_OUTPUT()

# define RW_HIGH()        (_LATB13 = 1)
# define RW_LOW()         (_LATB13 = 0)
# define CONFIG_RW()      CONFIG_RB13_AS_DIG_OUTPUT()

# define E_HIGH()         (_LATB14 = 1)
# define E_LOW()          (_LATB14 = 0)
# define CONFIG_E()       CONFIG_RB14_AS_DIG_OUTPUT()

# define LCD4O          (_LATB5)
# define LCD5O          (_LATB6)
# define LCD6O          (_LATB7)
# define LCD7O          (_LATB10)
# define LCD7I          (_RB10)

# define CONFIG_LCD4_AS_INPUT() CONFIG_RB5_AS_DIG_INPUT()
# define CONFIG_LCD5_AS_INPUT() CONFIG_RB6_AS_DIG_INPUT()
# define CONFIG_LCD6_AS_INPUT() CONFIG_RB7_AS_DIG_INPUT()
# define CONFIG_LCD7_AS_INPUT() CONFIG_RB10_AS_DIG_INPUT()

# define CONFIG_LCD4_AS_OUTPUT() CONFIG_RB5_AS_DIG_OUTPUT()
# define CONFIG_LCD5_AS_OUTPUT() CONFIG_RB6_AS_DIG_OUTPUT()
# define CONFIG_LCD6_AS_OUTPUT() CONFIG_RB7_AS_DIG_OUTPUT()
# define CONFIG_LCD7_AS_OUTPUT() CONFIG_RB10_AS_DIG_OUTPUT()
#endif

#define GET_BUSY_FLAG()  (LCD7I)

//Configure 4-bit data bus for output
void configBusAsOutLCD(void) {
  RW_LOW();                  //RW=0 to stop LCD from driving pins
  CONFIG_LCD4_AS_OUTPUT();   //D4
  CONFIG_LCD5_AS_OUTPUT();   //D5
  CONFIG_LCD6_AS_OUTPUT();   //D6
  CONFIG_LCD7_AS_OUTPUT();   //D7
}
//Configure 4-bit data bus for input

void configBusAsInLCD(void) {
  CONFIG_LCD4_AS_INPUT();   //D4
  CONFIG_LCD5_AS_INPUT();   //D5
  CONFIG_LCD6_AS_INPUT();   //D6
  CONFIG_LCD7_AS_INPUT();   //D7
  RW_HIGH();                   // R/W = 1, for read

}
//Output lower 4-bits of u8_c to LCD data lines

void outputToBusLCD(uint8_t u8_c) {
  LCD4O = u8_c & 0x01;          //D4
  LCD5O = (u8_c >> 1)& 0x01;    //D5
  LCD6O = (u8_c >> 2)& 0x01;    //D6
  LCD7O = (u8_c >> 3)& 0x01;    //D7

}

//Configure the control lines for the LCD
void configControlLCD(void) {
  CONFIG_RS();     //RS
  CONFIG_RW();     //RW
  CONFIG_E();      //E
  RW_LOW();
  E_LOW();
  RS_LOW();
}

//Pulse the E clock, 1 us delay around edges for
//setup/hold times
void pulseE(void) {
  DELAY_US(1);
  E_HIGH();
  DELAY_US(1);
  E_LOW();
  DELAY_US(1);
}

/* Write a byte (u8_Cmd) to the LCD.
u8_DataFlag is '1' if data byte, '0' if command byte
u8_CheckBusy is '1' if must poll busy bit before write, else simply delay before write
u8_Send8Bits is '1' if must send all 8 bits, else send only upper 4-bits
*/
void writeLCD(uint8_t u8_Cmd, uint8_t u8_DataFlag, uint8_t u8_CheckBusy, uint8_t u8_Send8Bits) {
  uint8_t u8_BusyFlag;
  uint8_t u8_wdtState;
  if (u8_CheckBusy) {
    RS_LOW();            //RS = 0 to check busy
    // check busy
    configBusAsInLCD();  //set data pins all inputs
    u8_wdtState = _SWDTEN;  //save WDT enable state
    CLRWDT();          //clear the WDT timer
    _SWDTEN = 1;            //enable WDT to escape infinite wait

    do {
      E_HIGH();
      DELAY_US(1);  // read upper 4 bits
      u8_BusyFlag = GET_BUSY_FLAG();
      E_LOW();
      DELAY_US(1);
      pulseE();              //pulse again for lower 4-bits
    } 
    
    while (u8_BusyFlag);
    _SWDTEN = u8_wdtState;   //restore WDT enable state
  } 
  else {
    DELAY_MS(10); // don't use busy, just delay
  }
  configBusAsOutLCD();
  if (u8_DataFlag) RS_HIGH();   // RS=1, data byte
  else    RS_LOW();             // RS=0, command byte
  outputToBusLCD(u8_Cmd >> 4);  // send upper 4 bits
  pulseE();
  if (u8_Send8Bits) {
    outputToBusLCD(u8_Cmd);     // send lower 4 bits
    pulseE();
  }
}
// Initialize the LCD, modify to suit your application and LCD
void initLCD() {
    DELAY_MS(50);          //wait for device to settle
    writeLCD(0x20,0,0,0); // 4 bit interface
    writeLCD(0x28,0,0,1); // 2 line display, 5x7 font
    writeLCD(0x28,0,0,1); // repeat
    writeLCD(0x06,0,0,1); // enable display
    writeLCD(0x0C,0,0,1); // turn display on; cursor, blink is off
    writeLCD(0x01,0,0,1); // clear display, move cursor to home
    DELAY_MS(3);
}
//Output a string to the LCD

void outStringLCD(char *psz_s) {
  while (*psz_s) {
    writeLCD(*psz_s, 1, 0,1);
    //writeLCD(*psz_s, 1, 1,1);
    psz_s++;
  }
}
//******************************************************************************
////////////////////////////////////////////////////////////////////////////////
//DS1631 code*******************************************************************
#define DS1631ADDR 0x90   //DS1631 address with all pins tied low
#define ACCESS_CONFIG 0xAC
#define START_CONVERT 0x51
#define READ_TEMP 0xAA

int16_t readTempDS1631() {
  uint8_t u8_lo, u8_hi;
  int16_t i16_temp;
  write1I2C1(DS1631ADDR, READ_TEMP);
  read2I2C1 (DS1631ADDR, &u8_hi, &u8_lo);
  i16_temp = u8_hi;
  return ((i16_temp<<8)|u8_lo);
}
//******************************************************************************
////////////////////////////////////////////////////////////////////////////////
//RTCC code*********************************************************************

#ifndef _LPOSCEN
#error "This example only works with a device that has a secondary oscillator."
#endif
#ifndef _RTCSYNC
#error "This example only works with a device that has an RTCC module."
#endif

typedef union _unionRTCC {
  struct {  //four 16 bit registers
    uint8_t yr;
    uint8_t null;
    uint8_t date;
    uint8_t month;
    uint8_t hour;
    uint8_t wday;
    uint8_t sec;
    uint8_t min;
  } u8;
  uint16_t regs[4];
} unionRTCC;

unionRTCC u_RTCC;

uint8_t getBCDvalue(char *sz_1) {
  char sz_buff[8];
  uint16_t u16_bin;
  uint8_t  u8_bcd;
  outString(sz_1);
  inStringEcho(sz_buff,7);
  sscanf(sz_buff,"%d", (int *)&u16_bin);
  u8_bcd = u16_bin/10;   //most significant digit
  u8_bcd = u8_bcd << 4;
  u8_bcd = u8_bcd | (u16_bin%10);
  return(u8_bcd);
}

void getDateFromUser(void) {
  u_RTCC.u8.yr = getBCDvalue("Enter year (0-99): ");
  u_RTCC.u8.month = getBCDvalue("Enter month (1-12): ");
  u_RTCC.u8.date = getBCDvalue("Enter day of month (1-31): ");
  //u_RTCC.u8.wday = getBCDvalue("Enter week day (0-6): ");
  u_RTCC.u8.hour = getBCDvalue("Enter hour (0-23): ");
  u_RTCC.u8.min = getBCDvalue("Enter min (0-59): ");
  u_RTCC.u8.sec = getBCDvalue("Enter sec(0-59): ");
}

//set date
void setRTCC(void) {
  uint8_t u8_i;
  __builtin_write_RTCWEN();   //enable write to RTCC, sets RTCWEN
  RCFGCALbits.RTCEN = 0;      //disable the RTCC
  RCFGCALbits.RTCPTR = 3;     //set pointer reg to start
  for (u8_i=0; u8_i<4; u8_i++) RTCVAL = u_RTCC.regs[u8_i];
  RCFGCALbits.RTCEN = 1;     //Enable the RTCC
  RCFGCALbits.RTCWREN = 0;   //can clear without unlock
}

void readRTCC(void) {
  uint8_t u8_i;
  RCFGCALbits.RTCPTR = 3;     //set pointer reg to start
  for (u8_i=0; u8_i<4; u8_i++) u_RTCC.regs[u8_i] = RTCVAL;
}

void printRTCC(void) {
  printf ("day/mon/yr: %2x/%2x/%2x, %02x:%02x:%02x \n",
          (uint16_t) u_RTCC.u8.date, (uint16_t) u_RTCC.u8.month,
          (uint16_t) u_RTCC.u8.yr, (uint16_t) u_RTCC.u8.hour, (uint16_t) u_RTCC.u8.min, (uint16_t) u_RTCC.u8.sec);
}

void writeConfigDS1631(uint8_t u8_i) {
  write2I2C1(DS1631ADDR, ACCESS_CONFIG, u8_i);
}

void startConversionDS1631() {
  write1I2C1(DS1631ADDR, START_CONVERT);
}
//******************************************************************************
////////////////////////////////////////////////////////////////////////////////
//EEPROM code*******************************************************************

#define EEPROM 0xA0   //LC515 address assuming both address pins tied low.
#define BLKSIZE (64)

void waitForWriteCompletion(uint8_t u8_i2cAddr) {
  uint8_t u8_ack, u8_savedSWDTEN;
  u8_savedSWDTEN = _SWDTEN;
  _SWDTEN = 1; //enable WDT so that do not get stuck in infinite loop!
  u8_i2cAddr = I2C_WADDR(u8_i2cAddr);  //write operation, R/W# = 0;
  do {
    startI2C1();
    u8_ack = putNoAckCheckI2C1(u8_i2cAddr);
    stopI2C1();
    //printf("stopped I2C ack = %d", (uint16_t)u8_ack);
  } while (u8_ack == I2C_NAK);
  _SWDTEN = u8_savedSWDTEN;  //restore WDT to original state
}

void memWriteLC515(uint8_t u8_i2cAddr,  uint16_t u16_MemAddr, uint8_t *pu8_buf) {
  uint8_t u8_AddrLo, u8_AddrHi;

  u8_AddrLo = u16_MemAddr & 0x00FF;
  u8_AddrHi = (u16_MemAddr >> 8);
  pu8_buf[0] = u8_AddrHi;   //place address into buffer
  pu8_buf[1] = u8_AddrLo;

  if (u16_MemAddr & 0x8000) {   
    // if MSB set , set block select bit
    u8_i2cAddr = u8_i2cAddr | 0x08;
  }
  waitForWriteCompletion(u8_i2cAddr);
  writeNI2C1(u8_i2cAddr,pu8_buf,BLKSIZE+2);
}

void memReadLC515(uint8_t u8_i2cAddr,  uint16_t u16_MemAddr, uint8_t *pu8_buf) {

  uint8_t u8_AddrLo, u8_AddrHi;

  u8_AddrLo = u16_MemAddr & 0x00FF;
  u8_AddrHi = (u16_MemAddr >> 8);

  if (u16_MemAddr & 0x8000) {
    // if MSB set , set block select bit
    u8_i2cAddr = u8_i2cAddr | 0x08;
  }
  waitForWriteCompletion(u8_i2cAddr);
  //set address counter
  write2I2C1(u8_i2cAddr,u8_AddrHi, u8_AddrLo);
  //read data
  readNI2C1(u8_i2cAddr,pu8_buf, BLKSIZE);
}
//******************************************************************************
////////////////////////////////////////////////////////////////////////////////
//Keypad code*******************************************************************
#define C0 _RA2
#define C1 _RA3
#define C2 _RB15

static inline void CONFIG_COLUMN() {
  CONFIG_RA2_AS_DIG_INPUT();
  ENABLE_RA2_PULLUP();
  CONFIG_RA3_AS_DIG_INPUT();
  ENABLE_RA3_PULLUP();
  CONFIG_RB15_AS_DIG_INPUT();
  ENABLE_RB15_PULLUP();
}

#define R0 _LATB0
#define R1 _LATB1
#define R2 _LATB2
#define R3 _LATB3
#define CONFIG_R0_DIG_OUTPUT() CONFIG_RB0_AS_DIG_OUTPUT()
#define CONFIG_R1_DIG_OUTPUT() CONFIG_RB1_AS_DIG_OUTPUT()
#define CONFIG_R2_DIG_OUTPUT() CONFIG_RB2_AS_DIG_OUTPUT()
#define CONFIG_R3_DIG_OUTPUT() CONFIG_RB3_AS_DIG_OUTPUT()
 

void CONFIG_ROW() {
  CONFIG_R0_DIG_OUTPUT();
  CONFIG_R1_DIG_OUTPUT();
  CONFIG_R2_DIG_OUTPUT();
  CONFIG_R3_DIG_OUTPUT();
}
static inline void DRIVE_ROW_LOW() {
  R0 = 0;
  R1 = 0;
  R2 = 0;
  R3 = 0;
}

static inline void DRIVE_ROW_HIGH() {
  R0 = 1;
  R1 = 1;
  R2 = 1;
  R3 = 1;
}

void configKeypad(void) {

  CONFIG_ROW();
  DRIVE_ROW_LOW();
  CONFIG_COLUMN();
  DELAY_US(1);     //wait for pullups to stabilize inputs
}

//drive one row low

void setOneRowLow(uint8_t u8_x) {

  switch (u8_x) {

    case 0:
      R0 = 0;
      R1 = 1;
      R2 = 1;
      R3 = 1;
      break;

    case 1:
      R0 = 1;
      R1 = 0;
      R2 = 1;
      R3 = 1;
      break;

    case 2:
      R0 = 1;
      R1 = 1;
      R2 = 0;
      R3 = 1;
      break; 

    default:
      R0 = 1;
      R1 = 1;
      R2 = 1;
      R3 = 0;
      break;
  }
}

#define NUM_ROWS 4
#define NUM_COLS 3

const uint8_t au8_keyTable[NUM_ROWS][NUM_COLS] = 
{ {'1', '2', '3'},

  {'4', '5', '6'},

  {'7', '8', '9'},
  
  {'*','0','#'}

};

const char words;
#define KEY_PRESSED() (!C0 || !C1 || !C2)   //any low
#define KEY_RELEASED() (C0 && C1 && C2)  //all high

uint8_t doKeyScan(void) {
  uint8_t u8_row, u8_col;
  //determine column
  if (!C0) u8_col = 0;
  else if (!C1) u8_col = 1;
  else if (!C2) u8_col = 2;
  else return('E'); //error
  //determine row
  for (u8_row = 0; u8_row < NUM_ROWS; u8_row++) {
    setOneRowLow(u8_row); //enable one row low
    if (KEY_PRESSED()) {
      DRIVE_ROW_LOW(); //return rows to driving low
      return(au8_keyTable[u8_row][u8_col]);
      //c = au8_keyTable[u8_row][u8_col];   
    }
  }

  DRIVE_ROW_LOW(); //return rows to driving low

  return('E'); //error

}

typedef enum {
        KEY_PAD_DEFAULT_OP,
        KEY_PAD_EEPROM_DISLAY_OP,
        KEY_PAD_HIGH_TEMP_INPUT_OP,
        KEY_PAD_LOW_TEMP_INPUT_OP,
} KEYPADOP;

typedef enum  {
  STATE_WAIT_FOR_PRESS = 0,
  STATE_WAIT_FOR_PRESS2,
  STATE_WAIT_FOR_RELEASE,
} ISRSTATE;

int mul = 1;
int input = 0;
int temperatureLimit_high = 0;
int temperatureLimit_low = 0;
// for handling the User input
void handleUserInput(char *userData,
                     int userDataSize,
                     int printHeader,
                     int inputComplete,
                     int op)
{
    char string[100];
    string[0] = '\0';
    printf("handleUserInput - %d, %d\n", userDataSize, printHeader);

    if (printHeader) //Display the Temperature threshold prompt
    {
        initLCD();
        //writeLCD(0x01,0,0,1); // clear display, move cursor to home
        if (op == KEY_PAD_HIGH_TEMP_INPUT_OP)
        {
            //writeLCD(0x01,0,0,1); // clear display, move cursor to home
            outStringLCD("T(high): ?");
        } else if (op == KEY_PAD_LOW_TEMP_INPUT_OP) {
            //writeLCD(0x01,0,0,1); // clear display, move cursor to home
            outStringLCD("T(low): ?");
        }
    } else {
        writeLCD(0xC0,0,0,1); // cursor to 2nd line.
        int index = 0;
        // Get data in the userData buffer and print the values
            for (index = 0; index < userDataSize; index++)
            {
                printf("handleUserInput userData[%d] = %d\n", userData[index], index);
                sprintf(string,"%s%c", string, userData[index]);
            }
            outStringLCD(string);
    }

    if (inputComplete) // if key-press input is done, start to display confirmation
    {
        int mul = 1;
        int input = 0;
        int index = 0;
        // Convert ASCII value to get Decimal value and print
        for (index = userDataSize - 1; index >= 0; index--)
        {
            input += (int)(userData[index] - '0') * mul;
            mul *= 10;
        }
        printf("user entered limit = %d\n", input);

        initLCD();
        if (op == KEY_PAD_HIGH_TEMP_INPUT_OP)
        {
            sprintf(string, "T(high) set as %d", input);
            outStringLCD(string);
            DELAY_MS(500);
            //printf("Temp is: %4.4f (C)\n",(double) f_tempC);
            DELAY_MS(500);
            temperatureLimit_high = input;
        } else if (op == KEY_PAD_LOW_TEMP_INPUT_OP) {
            sprintf(string, "T(low) set as %d", input);
            outStringLCD(string);
            DELAY_MS(500);
            //printf("Temp is: %4.4f (C)\n",(double) f_tempC);
            DELAY_MS(500);
            temperatureLimit_low = input;
        }
    }
}

ISRSTATE e_isrState = STATE_WAIT_FOR_PRESS;
KEYPADOP keyPadOp = KEY_PAD_DEFAULT_OP;
volatile uint8_t u8_newKey = 0;

void printLatestReadingsFromEEPROM(int numReadings)
{
    printf("testing 3 \n");
    uint8_t au8_buf[BLKSIZE+2];
    uint16_t u16_MemAddr = 0;
    uint8_t index;
    
    for (index = 0; index < numReadings; index++)
    {
        // read and check if it is written  
        memReadLC515(EEPROM, u16_MemAddr, au8_buf); // do read
        double temp_inC;
        // print time-stamp from EEPROM
        printf(" %2x/ %2x/ %2x, %02x:%02x:%02x\n",(uint16_t)au8_buf[0],(uint16_t)au8_buf[1],(uint16_t)au8_buf[2],(uint16_t)au8_buf[3],(uint16_t)au8_buf[4], (uint16_t)au8_buf[5]); 
        int16_t i16_temp = au8_buf[6] << 8 | au8_buf[7]; //read temp. from buffer and store to variable
        temp_inC = i16_temp;
        temp_inC = temp_inC/256; //get temp. in oC
        printf("Temp from eeprom is: %4.4f (C)\n", temp_inC); // print temp. in oC

        u16_MemAddr += BLKSIZE;
    }
}

char userData[10];
int  userDataSize = 0;
//Interrupt Service Routine for Timer3******************************************
void _ISR _T3Interrupt (void)
{
    switch (e_isrState)
    {
        case STATE_WAIT_FOR_PRESS:
            if (KEY_PRESSED() && (u8_newKey == 0)) {
                //ensure that key is sampled low for two consecutive interrupt periods
                e_isrState = STATE_WAIT_FOR_PRESS2;
            }
            break;

        case STATE_WAIT_FOR_PRESS2:
                if (KEY_PRESSED()) {
                    // a key is ready
                    u8_newKey = doKeyScan(); // do keyscan and store the pressed key value to u8_newKey
                    e_isrState = STATE_WAIT_FOR_RELEASE;
                } else {
                    e_isrState = STATE_WAIT_FOR_PRESS;
                }
                break;

        case STATE_WAIT_FOR_RELEASE:
            //keypad released
            if (KEY_RELEASED()) {
                printf("key released %c\n", u8_newKey);
                e_isrState = STATE_WAIT_FOR_PRESS;

                if (u8_newKey == '*') //check if the key pressed is â€˜*â€™
                {
                        keyPadOp = KEY_PAD_EEPROM_DISLAY_OP;
                        printLatestReadingsFromEEPROM(20);
                         keyPadOp = KEY_PAD_DEFAULT_OP;
                } else if (u8_newKey == '#') //check if the key pressed is â€˜#â€™
                  {
                    if (keyPadOp == KEY_PAD_DEFAULT_OP)
                    {
                        processingUserinput=1;
                        keyPadOp = KEY_PAD_HIGH_TEMP_INPUT_OP;
                        handleUserInput(userData, userDataSize,1, 0, keyPadOp);
                    } 
                    else if (keyPadOp == KEY_PAD_HIGH_TEMP_INPUT_OP) {
                        handleUserInput(userData, userDataSize,0, 1, keyPadOp);
                        userDataSize = 0;

                        // start processing temp low
                        keyPadOp = KEY_PAD_LOW_TEMP_INPUT_OP;
                        handleUserInput(userData, userDataSize,1, 0, keyPadOp);
                    } else if (keyPadOp == KEY_PAD_LOW_TEMP_INPUT_OP) {
                        handleUserInput(userData, userDataSize,0, 1, keyPadOp);
                        userDataSize = 0;
                        keyPadOp = KEY_PAD_DEFAULT_OP;
                        DELAY_MS(20);
                        processingUserinput=0;
                    }
                } else {
                    if ((keyPadOp == KEY_PAD_HIGH_TEMP_INPUT_OP) || (keyPadOp == KEY_PAD_LOW_TEMP_INPUT_OP)) //check status of keyPadOp
                    {
                        userData[userDataSize++] = u8_newKey; //get new key value to buffer
                        handleUserInput(userData, userDataSize, 0, 0, keyPadOp);
                    }
                }
                u8_newKey = 0;
            }
            break;

        default:
                e_isrState = STATE_WAIT_FOR_PRESS; //assign e_isrState to initial default state
                break;
    }
    _T3IF = 0;                 //clear the timer interrupt bit
}

#define ISR_PERIOD     15      // in ms

void  configTimer3(void) {
  //ensure that Timer2,3 configured as separate timers.
  T2CONbits.T32 = 0;     // 32-bit mode off
  //T3CON set like this for documentation purposes.
  //could be replaced by T3CON = 0x0020
  T3CON = T3_OFF | T3_IDLE_CON | T3_GATE_OFF
          | T3_SOURCE_INT
          | T3_PS_1_64 ;  //results in T3CON= 0x0020
  PR3 = msToU16Ticks (ISR_PERIOD, getTimerPrescale(T3CONbits)) - 1;
  TMR3  = 0;                       //clear timer3 value
  _T3IF = 0;                       //clear interrupt flag
  _T3IP = 1;                       //choose a priority
  _T3IE = 1;                       //enable the interrupt
  T3CONbits.TON = 1;               //turn on the timer
}
//******************************************************************************

float  f_tempC,f_tempF;
void showProjectHeader(){
initLCD();
outStringLCD("#Final  Project#");
}
// Configuring LED 1 and LED2
#define CONFIG_Green_LED2() CONFIG_RA1_AS_DIG_OUTPUT()
#define Green_LED2 (_RA1) 
#define VREF (3.3);//led1 state
#define CONFIG_Yellow_LED1() CONFIG_RA0_AS_DIG_OUTPUT()
#define Yellow_LED1 (_RA0)     //led1 state
#define VREF (3.3);

////////////////////////////////////////////////////////////////////////////////
//MAIN code*********************************************************************
////////////////////////////////////////////////////////////////////////////////

int main (void) {

configBasic(HELLO_MSG);
configControlLCD();
showProjectHeader();

int loopCount=0;


//DS1631 code******************************************************************* 
//  int16_t i16_temp;
//  float  f_tempC,f_tempF;
  configBasic(HELLO_MSG);
  configI2C1(400);            //configure I2C for 400 KHz
  writeConfigDS1631(0x0C);    //continuous conversion, 12-bit mode
  startConversionDS1631();    //start conversions
//******************************************************************************  
  
//EEPROM code*******************************************************************
  uint8_t au8_buf[BLKSIZE+2];  //2 extra bytes for address
  uint16_t u16_MemAddr;
//uint8_t u8_Mode;    
  u16_MemAddr = 0;     //start at location 0 in memory  
  // EEPROM INIT 
  configI2C1(400);            //configure I2C for 400 KHz
 
//******************************************************************************
  
// RTCC INIT part **************************************************************
__builtin_write_OSCCONL(OSCCON | 0x02);    //  OSCCON.SOSCEN=1;
configBasic(HELLO_MSG);   //say Hello!
getDateFromUser();    //get initial date/timesetRTCC();            //set the date/time
setRTCC();            //set the date/time    
printf("rtcc initialisation done\n");
    
printf("all initialisation done\n");

//Keypad code*******************************************************************

configBasic(HELLO_MSG);
/** PIO config ******/
configKeypad();
/** Configure the Timer */
configTimer3();
//******************************************************************************  
  while (1) {
      CONFIG_Green_LED2();
      CONFIG_Yellow_LED1();
     
    printf("executing\n");
    
//DS1631 code*******************************************************************
    int16_t i16_temp;
    float  f_tempC,f_tempF;  
    i16_temp = readTempDS1631(); // read current temperature and store it to i16_temp
    f_tempC = i16_temp;  //convert to floating point
    f_tempC = f_tempC/256;  //divide by precision
    f_tempF = f_tempC*9/5 + 32;
    
    printf("%2x/%2x/%2x, %02x:%02x:%02x Temp is: 0x%0X, %4.4f (C), %4.4f (F)\n",(uint16_t) u_RTCC.u8.date,(uint16_t) u_RTCC.u8.month,
          (uint16_t) u_RTCC.u8.yr, (uint16_t) u_RTCC.u8.hour, (uint16_t) u_RTCC.u8.min, (uint16_t) u_RTCC.u8.sec, i16_temp, (double) f_tempC, (double) f_tempF);  

    uint8_t u8_i;
    while (!RCFGCALbits.RTCSYNC) doHeartbeat();
      
    // Read from RTCC. Data will be in u_RTCC
    readRTCC();

    if (processingUserinput==0) {
        Green_LED2 = 0;
        Yellow_LED1 = 0;

            if(f_tempC <= temperatureLimit_low) // check if current temperature is lower than userlimit
            {   writeLCD(0x01,0,0,1); // clear display, move cursor to home
                outStringLCD("TEMP is LOW");
                Yellow_LED1 = 1;
                DELAY_MS(500);
                Green_LED2 = 0;
            }
             if(f_tempC >= temperatureLimit_high) // check if current temperature is greater than userlimit
            {  writeLCD(0x01,0,0,1); // clear display, move cursor to home
                outStringLCD("TEMP is HIGH");
                Green_LED2 = 1;
                DELAY_MS(500);
                Yellow_LED1 = 0;
            }

            char string[100];
            string[0] = '\0';
            writeLCD(0x01,0,0,1); // clear display, move cursor to home
            sprintf(string, "Temp is %4.2f C", f_tempC);
            outStringLCD(string);//}
            writeLCD(0xC0,0,0,1); // cursor to 2nd line.
            sprintf(string, "%2x/%1x/%2x,%02x:%02x:%02x",(uint16_t) u_RTCC.u8.date,(uint16_t) u_RTCC.u8.month,
                    (uint16_t) u_RTCC.u8.yr, (uint16_t) u_RTCC.u8.hour, (uint16_t) u_RTCC.u8.min, (uint16_t) u_RTCC.u8.sec );
            outStringLCD(string);
        }
    
    //first two buffer locations reserved for starting address
    u8_i = 2;
    au8_buf[u8_i++] = u_RTCC.u8.date;
    au8_buf[u8_i++] = u_RTCC.u8.month;
    au8_buf[u8_i++] = u_RTCC.u8.yr;
    au8_buf[u8_i++] = u_RTCC.u8.hour;
    au8_buf[u8_i++] = u_RTCC.u8.min;
    au8_buf[u8_i++] = u_RTCC.u8.sec;
    au8_buf[u8_i++] = (i16_temp & 0xFF00) >> 8;
    au8_buf[u8_i++] = i16_temp & 0xFF;  

    loopCount++;
    if(loopCount == 15){
    memWriteLC515(EEPROM,u16_MemAddr, au8_buf); // do write
    u16_MemAddr = u16_MemAddr + BLKSIZE;
    loopCount=0;
    }
    DELAY_MS(50);
    doHeartbeat();
  }
}
