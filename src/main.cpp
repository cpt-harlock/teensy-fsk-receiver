#include <Arduino.h>
#include <arm_math.h>
/**
 * Let's try to use Teensy 4.0 as a DSP companion for ADC values read from an external device (using UART interface)
 */
#define SAMPLING_FREQUENCY (1400.0)
#define FFT_LENGTH (512)
#define FREQUENCY_ONE (200.0)
#define FREQUENCY_ZERO (180.0)
#define CORRELATION_LENGTH (FFT_LENGTH * 2 - 1)
#define CORRELATION_THRESHOLD (0.93)
#define BYTES_PER_MESSAGE (1)
float32_t chirpSamplesBuffer[FFT_LENGTH];
float32_t workingChirpSamplesBuffer[FFT_LENGTH];
float32_t knownChirpSamplesBuffer[FFT_LENGTH] = {
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    -0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    -0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    -0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
    0.044194,
    -0.000000,
    -0.044194,
    -0.062500,
    -0.044194,
    -0.000000,
    0.044194,
    0.062500,
};
float32_t symbolSamplesBuffer[FFT_LENGTH];
float32_t fftBuffer[FFT_LENGTH];
float32_t magnitudeBuffer[FFT_LENGTH/2];
uint_fast32_t symbolBufferIndex = 0;
uint_fast8_t symbolBitCounter = 0;
uint_fast8_t symbolByteCounter = 0;
float32_t correlationBuffer[CORRELATION_LENGTH];
uint32_t sample;
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

void adc_start(uint8_t mux, uint8_t aref);

void setup()
{
  Serial.begin(38400); //reading as fast as possible
  Serial2.begin(115200);
  memset(chirpSamplesBuffer, 0, sizeof(float32_t) * FFT_LENGTH);
  arm_rfft_fast_init_f32(&fftInstance,FFT_LENGTH);
  freq_one_bin = round((FREQUENCY_ONE * FFT_LENGTH)/SAMPLING_FREQUENCY);
  freq_zero_bin = round((FREQUENCY_ZERO * FFT_LENGTH)/SAMPLING_FREQUENCY);
  analogReadRes(12); 
  analogReadAveraging(1);
}

void loop()
{
  sample = analogRead(A0);
  if (1)
  {
    //read float32_t as binary (4 bytes)
    // size_t readBytes = Serial2.readBytesUntil('\n',uartBuffer,128);
    // uartBuffer[readBytes] = '\0';
    // sample = atoi((char*)uartBuffer);
    if (receivingData)
    {
      symbolSamplesBuffer[symbolBufferIndex++] = sample;
      if(symbolBufferIndex == FFT_LENGTH) {
        symbolBufferIndex = 0;
        //computing FFT
        arm_rfft_fast_f32(&fftInstance,symbolSamplesBuffer,fftBuffer,0);
        arm_cmplx_mag_f32(fftBuffer,magnitudeBuffer,FFT_LENGTH);
        maxMagnitudeBufferIndex = 0;
        maxMagnitudeBuffer = 0.0;
        for(size_t i = 1; i < FFT_LENGTH/2; i++)
        {
          if(maxMagnitudeBuffer < magnitudeBuffer[i]) {
            maxMagnitudeBuffer = magnitudeBuffer[i];
            maxMagnitudeBufferIndex = i;
          }
        }
        if(maxMagnitudeBufferIndex == freq_one_bin) {
          //TODO: check bit ordering
          receivedData[symbolByteCounter] |= 1 << (7 - symbolBitCounter);
          Serial.println("Received one");
        }
        else {
          receivedData[symbolByteCounter] &= ~(1 << (7 - symbolBitCounter));        
          Serial.println("Received zero");
        }
        symbolBitCounter++;
        //if reached end of byte
        if(symbolBitCounter == 8) {
          symbolBitCounter = 0;
          symbolByteCounter++;
          if(symbolByteCounter == BYTES_PER_MESSAGE) {
            receivingData = 0;
          }
        }
      }
    }
    else
    {
      //receiving chirp

      // Serial.printf("Data from serial %f\r\n",sample);
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
        if (correlationBuffer[i] > maxCorrelationValue)
          maxCorrelationValue = correlationBuffer[i];
      }
      // Serial.printf("Max correlation %f\r\n", maxCorrelationValue);
      Serial.printf("Sample %u\r\n",sample);
      if (maxCorrelationValue > CORRELATION_THRESHOLD)
      {
        Serial.println("Receiving data");
        symbolBitCounter = symbolByteCounter = symbolBufferIndex = 0;
        receivingData = 1;
      }
    }
  }
}
