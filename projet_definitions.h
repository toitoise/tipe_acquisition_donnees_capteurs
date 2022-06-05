//*****************************************************************************
//*****************************************************************************
//               Les defintions de variables du projet
//*****************************************************************************
//*****************************************************************************

//-------------------------------------------------------
// ADS1115 PINS  (connected to interrupt pin)
//-------------------------------------------------------
#define  alertReadyPin  2

short muxStateforNextInterrupt=0x0;   // Variable pour changer de ie 0 par devo

//----------------------------------------------------------------
//----------------------DATA_RATE---------------------------------
//----------------------------------------------------------------
/* DateRate *********************************
DR=0 : 8 SPS            DR=4 : 128 SPS
DR=1 : 16 SPS           DR=5 : 250 SPS
DR=2 : 32 SPS           DR=6 : 450 SPS
DR=3 : 64 SPS           DR=7 : 860 SPS
********************************************/
const short sample_rate[]      ={ 8,  16, 32, 64, 128, 250, 475, 860 };   // Valeurs en Hz
const short sample_rate_prog[] ={ 0,   1,  2,  3,   4,   5,   6,   7 };   // Valeurs à programmer
#define sample_rate_sizeTab sizeof(sample_rate) / sizeof( short)
#define DEFAULT_RATE_DIFFERENTIAL 4           // 128   

//----------------------------------------------------------------
//---------------------------GAIN---------------------------------
//----------------------------------------------------------------
/* Gain interne *****************************
0 (gain = 2/3)  ± 6.144 volts 
1 (gain = 1)    ± 4.096 volts
2 (gain = 2)    ± 2.048 volts
4 (gain = 4)    ± 1.024 volts
8 (gain = 8)    ± 0.512 volts
16 (gain = 16)  ± 0.256 volts
********************************************/
//                           0       1       2       3       4       5
const float  gain_value[]={6.144 , 4.096 , 2.048 , 1.024 , 0.512 , 0.256};  // Valeurs en Volts +/-
const uint8_t gain_prog[]={  0   ,   1   ,   2   ,   4   ,   8   ,  16  };  // Valeurs à programmer
#define gain_value_sizeTab sizeof(gain_value) / sizeof(float)
#define DEFAULT_GAIN_DIFFERENTIAL   2    // 2.048 volts (better for Differential mode)

//-----------------------------
// Valeur à l'init
//-----------------------------
int16_t  CAN_value_A0_A1=0;                     // valeur a l'init
int16_t  CAN_value_A2_A3=0;                     // valeur à l'init
volatile bool DO_READ_ADC    =false;            // Utilisé par l'interrupt (MUST be volatile)

//----------------------I2C----------------------------------
#define NORMAL_I2C_SPEED   400000L      // Normal Speed 400000KHz
#define DEFAULT_I2C_SPEED  600000L      // Push Freq to 600000KHz
#define FAST_I2C_SPEED     800000L      // Only used for CAN reads
