#include <Arduino.h>
#include <ads1292r.h>
#include <SPI.h>
#include <WiFi.h>
#include <JY901.h>
#include <DFRobot_BMI160.h>  //步数陀螺仪

//步数陀螺仪
DFRobot_BMI160 bmi160;
//const int8_t i2c_addr = 0x69;
const int8_t i2c_addr = 0x68;
bool readStep = false;
/*
#if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_PRO
  //interrupt number of uno and mega2560 is 0
  int pbIn = 2;
#elif ARDUINO_AVR_LEONARDO
  //interrupt number of uno and leonardo is 0
  int pbIn = 3; 
#else
  int pbIn = 13;
#endif
*/
int pbIn = 17;
/*the bmi160 have two interrput interfaces*/
int int1 = 1;
int int2 = 2;

void stepChange()
{
  //once the step conter is changed, the value can be read 
  readStep = true;
}

void initBMI160()  //初始化
{
  //set and init the bmi160 i2c address  
  while (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("i2c init fail");
    delay(1000); 
  }
  
  //set interrput number to int1 or int2
  if (bmi160.setInt(int2) != BMI160_OK){
    Serial.println("set interrput fail");
    // while(1);
  }

  //set the bmi160 mode to step counter
  if (bmi160.setStepCounter() != BMI160_OK){
    Serial.println("set step fail");
    // while(1);
  }
  
  //set the bmi160 power model,contains:stepNormalPowerMode,stepLowPowerMode
  if (bmi160.setStepPowerMode(bmi160.stepNormalPowerMode) != BMI160_OK){
    Serial.println("set setStepPowerMode fail");
    // while(1);
  }

  attachInterrupt(pbIn, stepChange, FALLING);

  // Serial.println(pbIn);
}


//心率
ads1292r ADS1292;   // define class

//Packet format
#define  CES_CMDIF_PKT_START_1      0x0A
#define  CES_CMDIF_PKT_START_2      0xFA
#define  CES_CMDIF_TYPE_DATA        0x02
#define  CES_CMDIF_PKT_STOP_1       0x00
#define  CES_CMDIF_PKT_STOP_2       0x0B

#define TEMPERATURE 0
#define FILTERORDER         161
/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)
#define WAVE_SIZE  1

//******* ecg filter *********
#define MAX_PEAK_TO_SEARCH         5
#define MAXIMA_SEARCH_WINDOW      25
#define MINIMUM_SKIP_WINDOW       30
#define SAMPLING_RATE             125
#define TWO_SEC_SAMPLES           2 * SAMPLING_RATE
#define QRS_THRESHOLD_FRACTION    0.4
#define TRUE 1
#define FALSE 0

volatile uint8_t  SPI_Dummy_Buff[30];
uint8_t DataPacketHeader[16];
volatile signed long s32DaqVals[8];
uint8_t data_len = 7;
volatile byte SPI_RX_Buff[15] ;
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile bool ads1292dataReceived = false;
unsigned long uecgtemp = 0;
signed long secgtemp = 0;
int i, j;

//************** ecg *******************
int16_t CoeffBuf_40Hz_LowPass[FILTERORDER] =
{
  -72,    122,    -31,    -99,    117,      0,   -121,    105,     34,
  -137,     84,     70,   -146,     55,    104,   -147,     20,    135,
  -137,    -21,    160,   -117,    -64,    177,    -87,   -108,    185,
  -48,   -151,    181,      0,   -188,    164,     54,   -218,    134,
  112,   -238,     90,    171,   -244,     33,    229,   -235,    -36,
  280,   -208,   -115,    322,   -161,   -203,    350,    -92,   -296,
  361,      0,   -391,    348,    117,   -486,    305,    264,   -577,
  225,    445,   -660,     93,    676,   -733,   -119,    991,   -793,
  -480,   1486,   -837,  -1226,   2561,   -865,  -4018,   9438,  20972,
  9438,  -4018,   -865,   2561,  -1226,   -837,   1486,   -480,   -793,
  991,   -119,   -733,    676,     93,   -660,    445,    225,   -577,
  264,    305,   -486,    117,    348,   -391,      0,    361,   -296,
  -92,    350,   -203,   -161,    322,   -115,   -208,    280,    -36,
  -235,    229,     33,   -244,    171,     90,   -238,    112,    134,
  -218,     54,    164,   -188,      0,    181,   -151,    -48,    185,
  -108,    -87,    177,    -64,   -117,    160,    -21,   -137,    135,
  20,   -147,    104,     55,   -146,     70,     84,   -137,     34,
  105,   -121,      0,    117,    -99,    -31,    122,    -72
};
int16_t ECG_WorkingBuff[2 * FILTERORDER];
unsigned char Start_Sample_Count_Flag = 0;
unsigned char first_peak_detect = FALSE ;
unsigned int sample_index[MAX_PEAK_TO_SEARCH + 2] ;
uint16_t scaled_result_display[150];
uint8_t indx = 0;

int cnt = 0;
volatile uint8_t flag = 0;

int QRS_Second_Prev_Sample = 0 ;
int QRS_Prev_Sample = 0 ;
int QRS_Current_Sample = 0 ;
int QRS_Next_Sample = 0 ;
int QRS_Second_Next_Sample = 0 ;

static uint16_t QRS_B4_Buffer_ptr = 0 ;
/*   Variable which holds the threshold value to calculate the maxima */
int16_t QRS_Threshold_Old = 0;
int16_t QRS_Threshold_New = 0;
unsigned int sample_count = 0 ;

/* Variable which will hold the calculated heart rate */
volatile uint16_t QRS_Heart_Rate = 0 ;
int16_t ecg_wave_buff[1], ecg_filterout[1];

volatile uint8_t global_HeartRate = 0;
volatile uint8_t global_RespirationRate = 0;
long status_byte=0;
uint8_t LeadStatus=0;
boolean leadoff_deteted = true;
long time_elapsed=0;


void ECG_FilterProcess(int16_t * WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut);
void ECG_ProcessCurrSample(int16_t *CurrAqsSample, int16_t *FilteredOut);
void QRS_Algorithm_Interface(int16_t CurrSample);
static void QRS_process_buffer( void );
static void QRS_check_sample_crossing_threshold( uint16_t scaled_result );

void LMT70_GetTemp(void);
void PakageUpdate(void);
void PakageTcpSend(void);
void PakageTestSend(void);
void PakageUdpSend(void);

// IPAddress local_IP(192,168,1,1);
// IPAddress gateway(192,168,1,1);
// IPAddress subnet(255,255,255,0);

WiFiUDP Udp;
// WiFiServer server(8080);
// WiFiClient serverClients;

uint16_t LMT70_Temp = 0;

//数据封包
//Ax1 Ay1 Az1 Ax2 Ay2 Az2 EGG EGG EGG EGG HEARTRATE1 HEARTRATE2 HEARTRATE3 HEARTRATE4  TEMP1 TEMP2
volatile uint8_t Pakage[19] = {0x00};

void setup()
{
  delay(1000);
  // initalize the  data ready and chip select pins:

  //LMT70输入引脚
   pinMode(33,ANALOG);

  pinMode(ADS1292_DRDY_PIN, INPUT);  //6
  pinMode(ADS1292_CS_PIN, OUTPUT);    //7
  pinMode(ADS1292_START_PIN, OUTPUT);  //5
  pinMode(ADS1292_PWDN_PIN, OUTPUT);  //4

  Serial.begin(115200);  // Baudrate for serial communica

  ADS1292.ads1292_Init();  //initalize ADS1292 slave

  JY901.StartIIC();

  initBMI160();

  // WiFi.softAPConfig(local_IP,gateway,subnet);
  WiFi.softAP("OhhhFuckEsp32","88888888",5,0,4);
  // IPAddress my_ip=WiFi.softAPIP();
  // Serial.printf("%s",my_ip);
  // server.begin();
  //UDP部分
  Udp.begin(2345); //启用UDP监听以接收数据

  //TCP部分
  // server.begin();
  // server.setNoDelay(true);


  Serial.println("Initiliziation is done");
}




void loop()
{
  
  // Serial.printf("fuck\r\n");
  if ((digitalRead(ADS1292_DRDY_PIN)) == LOW)      // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  {
    SPI_RX_Buff_Ptr = ADS1292.ads1292_Read_Data(); // Read the data,point the data to a pointer

    for (i = 0; i < 9; i++)
    {
      SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);  // store the result data in array
    }
    ads1292dataReceived = true;
  }

  if (ads1292dataReceived == true)      // process the data
  {
    j = 0;
    for (i = 3; i < 9; i += 3)         // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
    {

      uecgtemp = (unsigned long) (  ((unsigned long)SPI_RX_Buff[i + 0] << 16) | ( (unsigned long) SPI_RX_Buff[i + 1] << 8) |  (unsigned long) SPI_RX_Buff[i + 2]);
      uecgtemp = (unsigned long) (uecgtemp << 8);
      secgtemp = (signed long) (uecgtemp);
      secgtemp = (signed long) (secgtemp >> 8);

      s32DaqVals[j++] = secgtemp;  //s32DaqVals[0] is Resp data and s32DaqVals[1] is ECG data
    }




    status_byte = (long)((long)SPI_RX_Buff[2] | ((long) SPI_RX_Buff[1]) <<8 | ((long) SPI_RX_Buff[0])<<16); // First 3 bytes represents the status
    status_byte  = (status_byte & 0x0f8000) >> 15;  // bit15 gives the lead status
    LeadStatus = (unsigned char ) status_byte ;  

    if(!((LeadStatus & 0x1f) == 0 ))
      leadoff_deteted  = true; 
    else
      leadoff_deteted  = false;
    
    ecg_wave_buff[0] = (int16_t)(s32DaqVals[1] >> 8) ;  // ignore the lower 8 bits out of 24bits
    

    if(leadoff_deteted == false) 
       {
          ECG_ProcessCurrSample(&ecg_wave_buff[0], &ecg_filterout[0]);   // filter out the line noise @40Hz cutoff 161 order
          QRS_Algorithm_Interface(ecg_filterout[0]);             // calculate 
       }
    else
       ecg_filterout[0] = 0;


    if(millis() > time_elapsed)  // update every one second
    {
      // Udp.beginPacket("192.168.1.2",1234); //准备发送数据
      if(leadoff_deteted == true) // lead in not connected
      {
        // Serial.println("ECG lead error!!! ensure the leads are properly connected");
        // Udp.write((const uint8_t*)("ECG lead error!!! ensure the leads are properly connected"),60);
        // Udp.endPacket(); 
      }
      else
      {
        // PakageUpdate();
        // PakageTestSend();
        // PakageUdpSend();
        // PakageTcpSend();

      //  Serial.print("Heart rate: ");
      //  Udp.write((const uint8_t*)("Heart rate: "),12);
      //  Serial.print(global_HeartRate);
      //  Udp.printf("%d",global_HeartRate);
      //  Serial.println("BPM");
      //  Udp.write((const uint8_t*)("BPM"),3);
      //  Udp.endPacket();
      }
      time_elapsed += 1000;
    }
Serial.printf("hello\r\n");
    PakageUpdate();
    PakageTestSend();
    // PakageTcpSend();
    PakageUdpSend();
    


  }

  ads1292dataReceived = false;
  SPI_RX_Buff_Count = 0;

  // PakageUpdate();
  PakageTestSend();


  // delay(500);

}





void ECG_FilterProcess(int16_t * WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut)
{

  int32_t acc = 0;   // accumulator for MACs
  int  k;

  // perform the multiply-accumulate
  for ( k = 0; k < 161; k++ )
  {
    acc += (int32_t)(*CoeffBuf++) * (int32_t)(*WorkingBuff--);
  }
  // saturate the result
  if ( acc > 0x3fffffff )
  {
    acc = 0x3fffffff;
  } else if ( acc < -0x40000000 )
  {
    acc = -0x40000000;
  }
  // convert from Q30 to Q15
  *FilterOut = (int16_t)(acc >> 15);
  //*FilterOut = *WorkingBuff;
}




void ECG_ProcessCurrSample(int16_t *CurrAqsSample, int16_t *FilteredOut)
{
  static uint16_t ECG_bufStart = 0, ECG_bufCur = FILTERORDER - 1, ECGFirstFlag = 1;
  static int16_t ECG_Pvev_DC_Sample, ECG_Pvev_Sample;/* Working Buffer Used for Filtering*/
  //  static short ECG_WorkingBuff[2 * FILTERORDER];
  int16_t *CoeffBuf;
  int16_t temp1, temp2, ECGData;

  /* Count variable*/
  uint16_t Cur_Chan;
  int16_t FiltOut = 0;
  //  short FilterOut[2];
  CoeffBuf = CoeffBuf_40Hz_LowPass;         // Default filter option is 40Hz LowPass

  if  ( ECGFirstFlag )                // First Time initialize static variables.
  {
    for ( Cur_Chan = 0 ; Cur_Chan < FILTERORDER; Cur_Chan++)
    {
      ECG_WorkingBuff[Cur_Chan] = 0;
    }
    ECG_Pvev_DC_Sample = 0;
    ECG_Pvev_Sample = 0;
    ECGFirstFlag = 0;
  }

  temp1 = NRCOEFF * ECG_Pvev_DC_Sample;       //First order IIR
  ECG_Pvev_DC_Sample = (CurrAqsSample[0]  - ECG_Pvev_Sample) + temp1;
  ECG_Pvev_Sample = CurrAqsSample[0];
  temp2 = ECG_Pvev_DC_Sample >> 2;
  ECGData = (int16_t) temp2;

  /* Store the DC removed value in Working buffer in millivolts range*/
  ECG_WorkingBuff[ECG_bufCur] = ECGData;
  ECG_FilterProcess(&ECG_WorkingBuff[ECG_bufCur], CoeffBuf, (int16_t*)&FiltOut);
  /* Store the DC removed value in ECG_WorkingBuff buffer in millivolts range*/
  ECG_WorkingBuff[ECG_bufStart] = ECGData;

  /* Store the filtered out sample to the LeadInfo buffer*/
  FilteredOut[0] = FiltOut ;//(CurrOut);

  ECG_bufCur++;
  ECG_bufStart++;

  if ( ECG_bufStart  == (FILTERORDER - 1))
  {
    ECG_bufStart = 0;
    ECG_bufCur = FILTERORDER - 1;
  }
  return ;
}


void QRS_Algorithm_Interface(int16_t CurrSample)
{
  //  static FILE *fp = fopen("ecgData.txt", "w");
  static int16_t prev_data[32] = {0};
  int16_t i;
  long Mac = 0;
  prev_data[0] = CurrSample;

  for ( i = 31; i > 0; i--)
  {
    Mac += prev_data[i];
    prev_data[i] = prev_data[i - 1];

  }
  Mac += CurrSample;
  Mac = Mac >> 2;
  CurrSample = (int16_t) Mac;
  QRS_Second_Prev_Sample = QRS_Prev_Sample ;
  QRS_Prev_Sample = QRS_Current_Sample ;
  QRS_Current_Sample = QRS_Next_Sample ;
  QRS_Next_Sample = QRS_Second_Next_Sample ;
  QRS_Second_Next_Sample = CurrSample ;



  QRS_process_buffer();
}

static void QRS_process_buffer( void )
{

  int16_t first_derivative = 0 ;
  int16_t scaled_result = 0 ;

  static int16_t Max = 0 ;

  /* calculating first derivative*/
  first_derivative = QRS_Next_Sample - QRS_Prev_Sample  ;



  /*taking the absolute value*/

  if (first_derivative < 0)
  {
    first_derivative = -(first_derivative);
  }


  scaled_result = first_derivative;

  if ( scaled_result > Max )
  {
    Max = scaled_result ;
  }


  QRS_B4_Buffer_ptr++;
  if (QRS_B4_Buffer_ptr ==  TWO_SEC_SAMPLES)
  {
    QRS_Threshold_Old = ((Max * 7) / 10 ) ;
    QRS_Threshold_New = QRS_Threshold_Old ;
   // if (Max > 70)
      first_peak_detect = TRUE ;
    Max = 0;
 
    //  ecg_wave_buff[0] = first_derivative;
    QRS_B4_Buffer_ptr = 0;


  }


  if ( TRUE == first_peak_detect )
  {
    QRS_check_sample_crossing_threshold( scaled_result ) ;
  }


}


static void QRS_check_sample_crossing_threshold( uint16_t scaled_result )
{
  /* array to hold the sample indexes S1,S2,S3 etc */

  static uint16_t s_array_index = 0 ;
  static uint16_t m_array_index = 0 ;

  static unsigned char threshold_crossed = FALSE ;
  static uint16_t maxima_search = 0 ;
  static unsigned char peak_detected = FALSE ;
  static uint16_t skip_window = 0 ;
  static long maxima_sum = 0 ;
  static unsigned int peak = 0;
  static unsigned int sample_sum = 0;
  static unsigned int nopeak = 0;
  uint16_t Max = 0 ;
  uint16_t HRAvg;
  // uint16_t  RRinterval = 0;

  if ( TRUE == threshold_crossed  )
  {
    /*
    Once the sample value crosses the threshold check for the
    maxima value till MAXIMA_SEARCH_WINDOW samples are received
    */
    sample_count ++ ;
    maxima_search ++ ;

    if ( scaled_result > peak )
    {
      peak = scaled_result ;
    }

    if ( maxima_search >= MAXIMA_SEARCH_WINDOW )
    {
      // Store the maxima values for each peak
      maxima_sum += peak ;
      maxima_search = 0 ;


      threshold_crossed = FALSE ;
      peak_detected = TRUE ;
    }

  }
  else if ( TRUE == peak_detected )
  {
    /*
    Once the sample value goes below the threshold
    skip the samples untill the SKIP WINDOW criteria is meet
    */
    sample_count ++ ;
    skip_window ++ ;

    if ( skip_window >= MINIMUM_SKIP_WINDOW )
    {
      skip_window = 0 ;
      peak_detected = FALSE ;
    }

    if ( m_array_index == MAX_PEAK_TO_SEARCH )
    {
      sample_sum = sample_sum / (MAX_PEAK_TO_SEARCH - 1);
      HRAvg =  (uint16_t) sample_sum  ;

      // Compute HR without checking LeadOffStatus
      QRS_Heart_Rate = (uint16_t) 60 *  SAMPLING_RATE;
      QRS_Heart_Rate =  QRS_Heart_Rate / HRAvg ;
      if (QRS_Heart_Rate > 250)
        QRS_Heart_Rate = 250 ;
 //     Serial.println(QRS_Heart_Rate);

      /* Setting the Current HR value in the ECG_Info structure*/
      maxima_sum =  maxima_sum / MAX_PEAK_TO_SEARCH;
      Max = (int16_t) maxima_sum ;
      /*  calculating the new QRS_Threshold based on the maxima obtained in 4 peaks */
      maxima_sum = Max * 7;
      maxima_sum = maxima_sum / 10;
      QRS_Threshold_New = (int16_t)maxima_sum;

      /* Limiting the QRS Threshold to be in the permissible range*/
      if (QRS_Threshold_New > (4 * QRS_Threshold_Old))
      {
        QRS_Threshold_New = QRS_Threshold_Old;
      }

      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;

      sample_sum = 0;
    }
  }
  else if ( scaled_result > QRS_Threshold_New )
  {
    /*
      If the sample value crosses the threshold then store the sample index
    */
    Start_Sample_Count_Flag = 1;
    sample_count ++ ;
    m_array_index++;
    threshold_crossed = TRUE ;
    peak = scaled_result ;
    nopeak = 0;

    /*  storing sample index*/
    sample_index[ s_array_index ] = sample_count ;
    if ( s_array_index >= 1 )
    {
      sample_sum += sample_index[ s_array_index ] - sample_index[ s_array_index - 1 ] ;
    }
    s_array_index ++ ;
  }

  else if (( scaled_result < QRS_Threshold_New ) && (Start_Sample_Count_Flag == 1))
  {
    sample_count ++ ;
    nopeak++;
    if (nopeak > (3 * SAMPLING_RATE))
    {
      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      peak_detected = FALSE ;
      sample_sum = 0;

      first_peak_detect = FALSE;
      nopeak = 0;

      QRS_Heart_Rate = 0;

    }
  }
  else
  {
    nopeak++;
    if (nopeak > (3 * SAMPLING_RATE))
    {
      /* Reset heart rate computation sate variable in case of no peak found in 3 seconds */
      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      peak_detected = FALSE ;
      sample_sum = 0;
      first_peak_detect = FALSE;
      nopeak = 0;
      QRS_Heart_Rate = 0;
    }
  }

  global_HeartRate = (uint8_t)QRS_Heart_Rate;
  //Serial.println(global_HeartRate);

}


void LMT70_GetTemp(void)
{
  volatile int m;
  m = analogRead(33);
  float j = -0.2885*m/4096.0*m/4096.0+3.5889*m/4096.0+0.0891;
  float n = j*1000.0;
  float k = (-0.0000084515)*n*n+(-0.176928)*n+204.393;

  LMT70_Temp = k*1000;

  // float j = m/4095.0;
  // float j = -0.2885*m/4096.0*m/4096.0+3.5889*m/4096.0+0.0891

  //  Serial.printf("%d\r\n",LMT70_Temp);
}

void PakageUpdate(void)
{
  uint16_t stepCounter = 0;

  //第一个陀螺仪
  Pakage[0] = JY901.ReadWord(0x34);
  Pakage[2] = JY901.ReadWord(0x35);
  Pakage[4] = JY901.ReadWord(0x36);

  //第二个陀螺仪
  // Pakage[6] = 0;
  
  if (bmi160.readStepCounter(&stepCounter)==BMI160_OK){
    Serial.printf("fuck\r\n");
    Serial.println(stepCounter);
    Pakage[6] = stepCounter;
    Pakage[7] = stepCounter>>8;
  }
  else
  {
    Pakage[6] = 0;
    Pakage[7] = 0;
  }
  
  Pakage[7] = 0;
  Pakage[8] = 0;
  Pakage[9] = 0;
  Pakage[10] = 0;
  Pakage[11] = 0;

  //EGC数据
  Pakage[12] = s32DaqVals[0];            // 4 bytes ECG data
  Pakage[13] = s32DaqVals[0] >> 8;
  Pakage[14] = s32DaqVals[0] >> 16;
  Pakage[15] = s32DaqVals[0] >> 24;  

  //心率数据
  Pakage[16] = global_HeartRate;

  //温度ADC
  LMT70_GetTemp();
  Pakage[17] = LMT70_Temp;
  Pakage[18] = LMT70_Temp >> 8;
}

// void PakageTcpSend(void)
// {
//   if(server.hasClient())
//   {
//     serverClients = server.available();
//   }

//   // if(!serverClients|| !serverClients.connected())
//   // {
//   //   serverClients.stop();
//   // }

//   if(serverClients.available())
//   {
//     serverClients.write((const char*)Pakage,19);
//   }

// }

void PakageTestSend(void)
{
  for(uint8_t i=0;i < 19; i++)
  {
    Serial.write(Pakage[i]);
  }
}

void PakageUdpSend(void)
{
  Udp.beginPacket("192.168.4.2",1234); //准备发送数据
  for(uint8_t fuck = 0;fuck < 19; fuck++)
  {
    Udp.write(Pakage[fuck]);
  }
  // Udp.write((const uint8_t*)"OhhhFcukyouMotherFucker\r\n",25);
  Udp.endPacket();
}

