#include <Arduino.h>
#include <arm_math.h>
#include <TeensyTimerTool.h>

using namespace TeensyTimerTool;

/**
 * Let's try to use Teensy 4.0 as a DSP companion for ADC values read from an external device (using UART interface)
 */
#define SAMPLING_FREQUENCY (500.0)
#define FFT_LENGTH (512)
#define FREQUENCY_ONE (247.0703)
#define FREQUENCY_ZERO (232.4219)
#define FREQUENCY_CHIRP (72.7778)
#define SYMBOL_FREQUENCY_INTERVAL (2)
#define CHIRP_MAGNITUDE_THRESHOLD (14.0)
#define CHIRP_MAGNITUDE_INTERVAL (2)
#define BYTES_PER_MESSAGE (3)
#define ADC_SENSOR_PIN (14)
#define ADC_RESOLUTION (10)
#define ADC_SAMPLE_AVERAGE (4)
#define DEBUG_PIN (15)
float32_t chirpSamplesBuffer[FFT_LENGTH];
float32_t workingChirpSamplesBuffer[FFT_LENGTH];
float32_t symbolSamplesBuffer[FFT_LENGTH];
float32_t fftBuffer[FFT_LENGTH];
float32_t magnitudeBuffer[FFT_LENGTH / 2];
uint_fast32_t symbolBufferIndex = 0;
uint_fast8_t symbolBitCounter = 0;
uint_fast8_t symbolByteCounter = 0;
volatile uint32_t sample;
boolean receivingData = false;
float32_t chirpOffset;
float32_t chirpNorm;
float32_t maxCorrelationValue;
arm_rfft_fast_instance_f32 fftInstance;
uint16_t freq_one_bin;
uint16_t freq_zero_bin;
uint16_t freq_chirp_bin;
uint16_t maxMagnitudeBufferIndex = 0;
float32_t maxMagnitudeBuffer = 0.0;
uint8_t receivedData[BYTES_PER_MESSAGE+1];
uint8_t uartBuffer[128];
volatile boolean sampleAvailable = false;

      
Timer t1;

void adc_start(uint8_t mux, uint8_t aref);

void timerCallback()
{
  // digitalWriteFast(DEBUG_PIN,!digitalReadFast(DEBUG_PIN));
  sample = analogRead(ADC_SENSOR_PIN);
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
  freq_chirp_bin = round((FREQUENCY_CHIRP * FFT_LENGTH) / SAMPLING_FREQUENCY);
  analogReadRes(ADC_RESOLUTION);
  analogReadAveraging(ADC_SAMPLE_AVERAGE);
  pinMode(DEBUG_PIN, OUTPUT);
  t1.beginPeriodic(timerCallback, 2'000);
  Serial.printf("Frequency one bin: %u\r\n",freq_one_bin);
  Serial.printf("Frequency zero bin: %u\r\n",freq_zero_bin);
  Serial.printf("Frequency chirp bin: %u\r\n",freq_chirp_bin);
  delay(5000);
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
        Serial.printf("Freq one bin index: %u\r\n", freq_one_bin);
        Serial.printf("Freq zero index: %u\r\n", freq_zero_bin);

        if (abs(maxMagnitudeBufferIndex-freq_one_bin) <= SYMBOL_FREQUENCY_INTERVAL)
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
            receivedData[BYTES_PER_MESSAGE] = 0;
            Serial.println((char*)receivedData);
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
      maxMagnitudeBuffer = 0.0;
      maxMagnitudeBufferIndex = 0;
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
      //fft
      arm_rfft_fast_f32(&fftInstance,workingChirpSamplesBuffer,fftBuffer,0);
      arm_cmplx_mag_f32(fftBuffer,magnitudeBuffer,FFT_LENGTH);
      //find max magnitude value and index
      for (size_t i = 0; i < FFT_LENGTH/2; i++)
      {
       if(magnitudeBuffer[i] > maxMagnitudeBuffer) {
         maxMagnitudeBuffer = magnitudeBuffer[i];
         maxMagnitudeBufferIndex = i;
       } 
      }
      
      // Serial.printf("Max magnitude %f\r\n", maxMagnitudeBuffer);
      // Serial.printf("Max magnitude index %u\r\n", maxMagnitudeBufferIndex);
      // Serial.printf("Chirp bin: %u\r\n", freq_chirp_bin);
      if(abs(maxMagnitudeBufferIndex - freq_chirp_bin) <= CHIRP_MAGNITUDE_INTERVAL && 
      maxMagnitudeBuffer >= CHIRP_MAGNITUDE_THRESHOLD) {
        receivingData = true;
        symbolBufferIndex = 0;
        symbolBitCounter = 0;
        symbolByteCounter = 0;
      }
      // Serial.printf("%f\r\n", workingChirpSamplesBuffer[FFT_LENGTH - 1]);
    }
  }
}
