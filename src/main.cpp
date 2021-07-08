#include <Arduino.h>
#include <arm_math.h>
#include <TeensyTimerTool.h>

using namespace TeensyTimerTool;

/**
 * Let's try to use Teensy 4.0 as a DSP companion for ADC values read from an external device (using UART interface)
 */
#define SAMPLING_FREQUENCY (500.0)
#define FFT_LENGTH (512)
#define FREQUENCY_ONE (200.0)
#define FREQUENCY_ZERO (180.0)
#define CORRELATION_LENGTH (FFT_LENGTH * 2 - 1)
#define CORRELATION_THRESHOLD (0.8)
#define BYTES_PER_MESSAGE (3)
#define DEBUG_PIN (15)
float32_t chirpSamplesBuffer[FFT_LENGTH];
float32_t workingChirpSamplesBuffer[FFT_LENGTH];
float32_t knownChirpSamplesBuffer[FFT_LENGTH] = {
    0.062491,
    0.038129,
    -0.015963,
    -0.057609,
    -0.054336,
    -0.008697,
    0.043723,
    0.062052,
    0.031998,
    -0.023005,
    -0.060071,
    -0.050299,
    -0.001309,
    0.048702,
    0.060739,
    0.025417,
    -0.029723,
    -0.061688,
    -0.045554,
    0.006098,
    0.052996,
    0.058572,
    0.018479,
    -0.036022,
    -0.062437,
    -0.040168,
    0.013420,
    0.056544,
    0.055581,
    0.011280,
    -0.041815,
    -0.062307,
    -0.034218,
    0.020552,
    0.059297,
    0.051807,
    0.003923,
    -0.047020,
    -0.061301,
    -0.027785,
    0.027395,
    0.061215,
    0.047305,
    -0.003489,
    -0.051563,
    -0.059433,
    -0.020962,
    0.033853,
    0.062272,
    0.042138,
    -0.010852,
    -0.055381,
    -0.056728,
    -0.013844,
    0.039834,
    0.062453,
    0.036377,
    -0.018063,
    -0.058419,
    -0.053225,
    -0.006531,
    0.045255,
    0.061756,
    0.030105,
    -0.025019,
    -0.060635,
    -0.048973,
    0.000874,
    0.050040,
    0.060189,
    0.023409,
    -0.031624,
    -0.061999,
    -0.044033,
    0.008266,
    0.054120,
    0.057776,
    0.016383,
    -0.037783,
    -0.062490,
    -0.038472,
    0.015542,
    0.057439,
    0.054549,
    0.009127,
    -0.043411,
    -0.062102,
    -0.032371,
    0.022600,
    0.059949,
    0.050556,
    0.001743,
    -0.048428,
    -0.060840,
    -0.025814,
    0.029339,
    0.061617,
    0.045851,
    -0.005665,
    -0.052764,
    -0.058722,
    -0.018894,
    0.035666,
    0.062417,
    0.040501,
    -0.012994,
    -0.056358,
    -0.055778,
    -0.011708,
    0.041491,
    0.062339,
    0.034581,
    -0.020141,
    -0.059158,
    -0.052049,
    -0.004357,
    0.046732,
    0.061384,
    0.028174,
    -0.027004,
    -0.061126,
    -0.047588,
    0.003055,
    0.051316,
    0.059566,
    0.021371,
    -0.033486,
    -0.062234,
    -0.042458,
    0.010424,
    0.055178,
    0.056909,
    0.014268,
    -0.039498,
    -0.062467,
    -0.036730,
    0.017646,
    0.058263,
    0.053452,
    0.006963,
    -0.044954,
    -0.061821,
    -0.030485,
    0.024620,
    0.060529,
    0.049242,
    -0.000439,
    -0.049778,
    -0.060305,
    -0.023811,
    0.031248,
    0.061943,
    0.044340,
    -0.007835,
    -0.053901,
    -0.057940,
    -0.016803,
    0.037436,
    0.062485,
    0.038814,
    -0.015121,
    -0.057266,
    -0.054760,
    -0.009557,
    0.043097,
    0.062149,
    0.032742,
    -0.022194,
    -0.059825,
    -0.050810,
    -0.002178,
    0.048152,
    0.060938,
    0.026209,
    -0.028955,
    -0.061543,
    -0.046145,
    0.005232,
    0.052530,
    0.058869,
    0.019308,
    -0.035308,
    -0.062394,
    -0.040831,
    0.012569,
    0.056168,
    0.055973,
    0.012135,
    -0.041165,
    -0.062368,
    -0.034942,
    0.019729,
    0.059017,
    0.052289,
    0.004791,
    -0.046442,
    -0.061464,
    -0.028562,
    0.026611,
    0.061034,
    0.047869,
    -0.002620,
    -0.051067,
    -0.059696,
    -0.021779,
    0.033118,
    0.062194,
    0.042776,
    -0.009995,
    -0.054972,
    -0.057087,
    -0.014691,
    0.039160,
    0.062478,
    0.037081,
    -0.017229,
    -0.058104,
    -0.053676,
    -0.007395,
    0.044651,
    0.061883,
    0.030864,
    -0.024220,
    -0.060419,
    -0.049509,
    0.000004,
    0.049514,
    0.060417,
    0.024213,
    -0.030871,
    -0.061884,
    -0.044646,
    0.007403,
    0.053680,
    0.058102,
    0.017221,
    -0.037087,
    -0.062478,
    -0.039154,
    0.014698,
    0.057090,
    0.054969,
    0.009987,
    -0.042781,
    -0.062193,
    -0.033112,
    0.021787,
    0.059698,
    0.051062,
    0.002613,
    -0.047874,
    -0.061033,
    -0.026604,
    0.028569,
    0.061466,
    0.046437,
    -0.004799,
    -0.052293,
    -0.059014,
    -0.019721,
    0.034949,
    0.062368,
    0.041159,
    -0.012142,
    -0.055976,
    -0.056165,
    -0.012561,
    0.040837,
    0.062394,
    0.035302,
    -0.019315,
    -0.058872,
    -0.052526,
    -0.005224,
    0.046150,
    0.061541,
    0.028948,
    -0.026217,
    -0.060940,
    -0.048147,
    0.002186,
    0.050815,
    0.059823,
    0.022187,
    -0.032749,
    -0.062150,
    -0.043092,
    0.009565,
    0.054764,
    0.057263,
    0.015113,
    -0.038820,
    -0.062485,
    -0.037430,
    0.016810,
    0.057943,
    0.053897,
    0.007827,
    -0.044346,
    -0.061942,
    -0.031241,
    0.023818,
    0.060307,
    0.049773,
    0.000431,
    -0.049247,
    -0.060527,
    -0.024613,
    0.030492,
    0.061822,
    0.044949,
    -0.006971,
    -0.053456,
    -0.058260,
    -0.017639,
    0.036736,
    0.062467,
    0.039492,
    -0.014275,
    -0.056912,
    -0.055174,
    -0.010416,
    0.042463,
    0.062234,
    0.033480,
    -0.021379,
    -0.059568,
    -0.051312,
    -0.003047,
    0.047593,
    0.061125,
    0.026996,
    -0.028181,
    -0.061386,
    -0.046727,
    0.004365,
    0.052054,
    0.059156,
    0.020133,
    -0.034587,
    -0.062340,
    -0.041485,
    0.011716,
    0.055782,
    0.056354,
    0.012987,
    -0.040507,
    -0.062417,
    -0.035660,
    0.018901,
    0.058725,
    0.052760,
    0.005658,
    -0.045856,
    -0.061615,
    -0.029332,
    0.025821,
    0.060842,
    0.048423,
    -0.001751,
    -0.050560,
    -0.059947,
    -0.022593,
    0.032378,
    0.062103,
    0.043406,
    -0.009135,
    -0.054553,
    -0.057436,
    -0.015535,
    0.038479,
    0.062490,
    0.037777,
    -0.016391,
    -0.057779,
    -0.054116,
    -0.008258,
    0.044038,
    0.061998,
    0.031617,
    -0.023416,
    -0.060191,
    -0.050035,
    -0.000866,
    0.048978,
    0.060634,
    0.025012,
    -0.030111,
    -0.061757,
    -0.045250,
    0.006539,
    0.053229,
    0.058416,
    0.018055,
    -0.036383,
    -0.062454,
    -0.039828,
    0.013852,
    0.056731,
    0.055377,
    0.010845,
    -0.042143,
    -0.062272,
    -0.033846,
    0.020969,
    0.059435,
    0.051559,
    0.003481,
    -0.047310,
    -0.061214,
    -0.027388,
    0.027792,
    0.061303,
    0.047015,
    -0.003931,
    -0.051812,
    -0.059294,
    -0.020544,
    0.034224,
    0.062308,
    0.041810,
    -0.011288,
    -0.055584,
    -0.056541,
    -0.013412,
    0.040174,
    0.062436,
    0.036016,
    -0.018486,
    -0.058575,
    -0.052992,
    -0.006091,
    0.045559,
    0.061686,
    0.029716,
    -0.025425,
    -0.060741,
    -0.048697,
    0.001316,
    0.050304,
    0.060069,
    0.022997,
    -0.032005,
    -0.062053,
    -0.043717,
    0.008705,
    0.054340,
    0.057606,
    0.015956,
    -0.038135,
    -0.062491,
    -0.038123,
    0.015971,
    0.057612,
    0.054332,
    0.008689,
    -0.043729,
    -0.062051,
    -0.031991,
    0.023012,
    0.060073,
    0.050294,
    0.001301,
    -0.048707,
    -0.060737,
    -0.025410,
    0.029730,
    0.061689,
    0.045549,
    -0.006106,
    -0.053000,
    -0.058569,
    -0.018471,
    0.036029,
    0.062437,
    0.040162,
    -0.013427,
    -0.056548,
    -0.055577,
    -0.011273,
    0.041821,
    0.062307,
    0.034211,
    -0.020559,
    -0.059299,
    -0.051803,
    -0.003915,
    0.047025,
    0.061300,
    0.027778,
    -0.027402,
    -0.061217,
    -0.047300,
    0.003497,
    0.051567,
    0.059430,
    0.020955,
    -0.033859,
    -0.062273,
    -0.042132,
    0.010860,
    0.055384,
    0.056725,
    0.013836,
    -0.039840,
    -0.062453,
    -0.036371,
    0.018070,
    0.058422,
    0.053221,
    0.006523,
    -0.045261,
};
float32_t symbolSamplesBuffer[FFT_LENGTH];
float32_t fftBuffer[FFT_LENGTH];
float32_t magnitudeBuffer[FFT_LENGTH / 2];
uint_fast32_t symbolBufferIndex = 0;
uint_fast8_t symbolBitCounter = 0;
uint_fast8_t symbolByteCounter = 0;
float32_t correlationBuffer[CORRELATION_LENGTH];
volatile uint32_t sample;
boolean receivingData = false;
float32_t chirpOffset;
float32_t chirpNorm;
float32_t maxCorrelationValue;
arm_rfft_fast_instance_f32 fftInstance;
uint16_t freq_one_bin;
uint16_t freq_zero_bin;
uint16_t maxMagnitudeBufferIndex = 0;
float32_t maxMagnitudeBuffer = 0.0;
uint8_t receivedData[BYTES_PER_MESSAGE];
uint8_t uartBuffer[128];
volatile boolean sampleAvailable = false;

Timer t1;

void adc_start(uint8_t mux, uint8_t aref);

void timerCallback()
{
  // digitalWriteFast(DEBUG_PIN,!digitalReadFast(DEBUG_PIN));
  sample = analogRead(A0);
  sampleAvailable = true;
}

void setup()
{
  Serial.begin(115200); //reading as fast as possible
  Serial2.begin(115200);
  memset(chirpSamplesBuffer, 0, sizeof(float32_t) * FFT_LENGTH);
  memset(workingChirpSamplesBuffer,0, sizeof(float32_t) * FFT_LENGTH);
  arm_rfft_fast_init_f32(&fftInstance, FFT_LENGTH);
  freq_one_bin = round((FREQUENCY_ONE * FFT_LENGTH) / SAMPLING_FREQUENCY);
  freq_zero_bin = round((FREQUENCY_ZERO * FFT_LENGTH) / SAMPLING_FREQUENCY);
  analogReadRes(10);
  analogReadAveraging(4);
  pinMode(DEBUG_PIN, OUTPUT);
  t1.beginPeriodic(timerCallback, 2'000);
  delay(2000);
  Serial.printf("Frequency one bin: %u\r\n",freq_one_bin);
  Serial.printf("Frequency zero bin: %u\r\n",freq_zero_bin);
}

void loop()
{
  //DEBUG
  // if (Serial.available() >= 4)
  //END DEBUG
  if (sampleAvailable)
  {
    //DEBUG
    // Serial.readBytes((char *)(&sample), 4);
    //END DEBUG
    sampleAvailable = false;
    digitalWriteFast(DEBUG_PIN, !digitalReadFast(DEBUG_PIN));

    if (receivingData)
    {
      symbolSamplesBuffer[symbolBufferIndex++] = sample;
      if (symbolBufferIndex == FFT_LENGTH)
      {
        symbolBufferIndex = 0;
        //computing FFT
        arm_rfft_fast_f32(&fftInstance, symbolSamplesBuffer, fftBuffer, 0);
        arm_cmplx_mag_f32(fftBuffer, magnitudeBuffer, FFT_LENGTH);
        maxMagnitudeBufferIndex = 0;
        maxMagnitudeBuffer = 0.0;
        for (size_t i = 1; i < FFT_LENGTH / 2; i++)
        {
          if (maxMagnitudeBuffer < magnitudeBuffer[i])
          {
            maxMagnitudeBuffer = magnitudeBuffer[i];
            maxMagnitudeBufferIndex = i;
          }
        }
        Serial.printf("Max bin index: %u\r\n", maxMagnitudeBufferIndex);
        if (maxMagnitudeBufferIndex == freq_one_bin)
        {
          //TODO: check bit ordering
          receivedData[symbolByteCounter] |= 1 << (7 - symbolBitCounter);
          Serial.println("Received one");
        }
        else
        {
          receivedData[symbolByteCounter] &= ~(1 << (7 - symbolBitCounter));
          Serial.println("Received zero");
        }
        symbolBitCounter++;
        //if reached end of byte
        if (symbolBitCounter == 8)
        {
          symbolBitCounter = 0;
          symbolByteCounter++;
          if (symbolByteCounter == BYTES_PER_MESSAGE)
          {
            receivingData = 0;
            //clear chirp buffer
            memset(chirpSamplesBuffer,0,sizeof(float32_t)*FFT_LENGTH);
          }
        }
      }
    }
    else
    {
      //receiving chirp
      // Serial.printf("%u\r\n",sample);
      chirpOffset = 0.0;
      chirpNorm = 0.0;
      maxCorrelationValue = 0.0;
      for (size_t i = 0; i < FFT_LENGTH - 1; i++)
      {
        chirpSamplesBuffer[i] = chirpSamplesBuffer[i + 1];
        //TODO: optimize (memory puzzle)
        chirpOffset += chirpSamplesBuffer[i];
      }
      chirpSamplesBuffer[FFT_LENGTH - 1] = sample;
      chirpOffset += sample;
      chirpOffset /= FFT_LENGTH;
      //subtracting offset and  computing norm
      for (size_t i = 0; i < FFT_LENGTH; i++)
      {
        workingChirpSamplesBuffer[i] = chirpSamplesBuffer[i] - chirpOffset;
        chirpNorm += workingChirpSamplesBuffer[i] * workingChirpSamplesBuffer[i];
      }
      chirpNorm = sqrt(chirpNorm);
      //normalizing
      for (size_t i = 0; i < FFT_LENGTH; i++)
      {
        workingChirpSamplesBuffer[i] /= chirpNorm;
      }
      //correlation
      arm_correlate_f32(workingChirpSamplesBuffer, FFT_LENGTH, knownChirpSamplesBuffer, FFT_LENGTH, correlationBuffer);
      for (size_t i = 0; i < CORRELATION_LENGTH; i++)
      {
        if (abs(correlationBuffer[i]) > maxCorrelationValue)
          maxCorrelationValue = abs(correlationBuffer[i]);
      }
      Serial.printf("Max correlation %f\r\n", maxCorrelationValue);
      // Serial.printf("%f\r\n", workingChirpSamplesBuffer[FFT_LENGTH - 1]);
      if (maxCorrelationValue > CORRELATION_THRESHOLD)
      {
        Serial.println("Receiving data");
        symbolBitCounter = symbolByteCounter = symbolBufferIndex = 0;
        receivingData = 1;
      }
    }
  }
}
