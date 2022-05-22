//*****************************************************************************
//*****************************************************************************
//               Les defintions de variables du projet
//*****************************************************************************
//*****************************************************************************

//-------------------------------------------------------
// ADS1115 PINS  (connected to interrupt pin)
//-------------------------------------------------------
#define  alertReadyPin  2

//-------------------------------------------------------
// Rotary encoder pins 
//-------------------------------------------------------
#define encoderClk    8    // CLK
#define encoderData   9    // Data
#define encoderSwitch 10    // Switch Button

//----------------------MODE---------------------------------
// MODE PIN for ADS conversion mode
// 1=Differential
// 0=Single-Ended
//-------------------------------------------------------
#define  SingleEndedOrDifferentialPin  6
#define  DiffMode   1
#define  SingleMode 0

#define sample_rate_sizeTab sizeof(sample_rate) / sizeof( short)
#define DEFAULT_RATE_SINGLE       3           // 64HZ   (5 entrées: 0 à 4)
#define DEFAULT_RATE_DIFFERENTIAL 2           // 32HZ   (2 entrées)

#define DISPLAY_VALUE_TIME 2000               // 2 sec

//----------------------GAIN---------------------------------
#define gain_value_sizeTab sizeof(gain_value) / sizeof(float)
#define DEFAULT_GAIN_SINGLE 1    // 4.096 volts
#define DEFAULT_GAIN_DIFF   2    // 2.048 volts (better for Differential mode)

//----------------------I2C----------------------------------
#define NORMAL_I2C_SPEED   400000L      // Normal Speed 400000KHz
#define DEFAULT_I2C_SPEED  600000L      // Push Freq to 600000KHz
#define FAST_I2C_SPEED     800000L      // Only used for CAN reads

//-------------------------------------------------------
// Positions sur le LCD de diverses INFO
//-------------------------------------------------------
#define POS_HELLO_COL     2
#define POS_HELLO_LIG     1
#define POS_MSG_COL     0
#define POS_MSG_LIG     2

#define POS_DATE_COL      3
#define POS_DATE_LIG      0
#define POS_MODE_COL      1
#define POS_MODE_LIG      2

#define POS_SAMPLE_RATE   10            // Position sur LCD/colonne
#define POS_GAIN          10            // Position sur LCD/colonne
#define POS_CHAN_A_COL    0
#define POS_CHAN_A_LIG    0
#define POS_CHAN_B_COL    0
#define POS_CHAN_B_LIG    0
#define POS_CHAN_C_COL    0
#define POS_CHAN_C_LIG    0
#define POS_CHAN_D_COL    0
#define POS_CHAN_D_COL    0
