#include <Arduino.h>
#include <arm_math.h>
/**
 * Let's try to use Teensy 4.0 as a DSP companion for ADC values read from an external device (using UART interface)
 */
#define SAMPLING_FREQUENCY (495.0)
#define FFT_LENGTH (512)
#define FREQUENCY_ONE (200.0)
#define FREQUENCY_ZERO (180.0)
#define CORRELATION_LENGTH (FFT_LENGTH * 2 - 1)
#define CORRELATION_THRESHOLD (0.93)
#define BYTES_PER_MESSAGE (1)
#define DEBUG_PIN (15)
float32_t chirpSamplesBuffer[FFT_LENGTH];
float32_t workingChirpSamplesBuffer[FFT_LENGTH];
float32_t knownChirpSamplesBuffer[FFT_LENGTH] = {
    -0.057421,
    0.057105,
    -0.020603,
    -0.028955,
    0.060165,
    -0.053249,
    0.012590,
    0.036047,
    -0.061841,
    0.048448,
    -0.004354,
    -0.042499,
    0.062421,
    -0.042788,
    -0.003959,
    0.048198,
    -0.061894,
    0.036369,
    0.012202,
    -0.053041,
    0.060269,
    -0.029305,
    -0.020229,
    0.056944,
    -0.057575,
    0.021721,
    0.027897,
    -0.059837,
    0.053860,
    -0.013752,
    -0.035070,
    0.061668,
    -0.049189,
    0.005539,
    0.041621,
    -0.062406,
    0.043646,
    0.002772,
    -0.047434,
    0.062037,
    -0.037329,
    -0.011034,
    0.052405,
    -0.060567,
    0.030349,
    0.019101,
    -0.056447,
    0.058024,
    -0.022832,
    -0.026828,
    0.059488,
    -0.054450,
    0.014909,
    0.034080,
    -0.061473,
    0.049912,
    -0.006722,
    -0.040728,
    0.062368,
    -0.044487,
    -0.001585,
    0.046652,
    -0.062157,
    0.038274,
    0.009863,
    -0.051750,
    0.060844,
    -0.031382,
    -0.017966,
    0.055929,
    -0.058451,
    0.023933,
    0.025751,
    -0.059117,
    0.055022,
    -0.016060,
    -0.033078,
    0.061256,
    -0.050616,
    0.007902,
    0.039820,
    -0.062308,
    0.045313,
    0.000396,
    -0.045854,
    0.062255,
    -0.039206,
    -0.008687,
    0.051076,
    -0.061098,
    0.032404,
    0.016825,
    -0.055391,
    0.058858,
    -0.025027,
    -0.024663,
    0.058725,
    -0.055573,
    0.017206,
    0.032065,
    -0.061016,
    0.051303,
    -0.009080,
    -0.038897,
    0.062225,
    -0.046122,
    0.000792,
    0.045040,
    -0.062331,
    0.040124,
    0.007509,
    -0.050383,
    0.061331,
    -0.033414,
    -0.015677,
    0.054834,
    -0.059243,
    0.026111,
    0.023567,
    -0.058311,
    0.056104,
    -0.018345,
    -0.031039,
    0.060754,
    -0.051970,
    0.010254,
    0.037961,
    -0.062120,
    0.046915,
    -0.001981,
    -0.044209,
    0.062384,
    -0.041027,
    -0.006328,
    0.049673,
    -0.061541,
    0.034411,
    0.014524,
    -0.054256,
    0.059607,
    -0.027186,
    -0.022462,
    0.057876,
    -0.056615,
    0.019478,
    0.030002,
    -0.060470,
    0.052619,
    -0.011424,
    -0.037010,
    0.061992,
    -0.047690,
    0.003168,
    0.043362,
    -0.062414,
    0.041915,
    0.005144,
    -0.048944,
    0.061729,
    -0.035397,
    -0.013365,
    0.053658,
    -0.059949,
    0.028251,
    0.021349,
    -0.057421,
    0.057105,
    -0.020603,
    -0.028955,
    0.060165,
    -0.053249,
    0.012590,
    0.036047,
    -0.061841,
    0.048448,
    -0.004354,
    -0.042499,
    0.062421,
    -0.042788,
    -0.003959,
    0.048198,
    -0.061894,
    0.036369,
    0.012202,
    -0.053041,
    0.060269,
    -0.029305,
    -0.020229,
    0.056944,
    -0.057575,
    0.021721,
    0.027897,
    -0.059837,
    0.053860,
    -0.013752,
    -0.035070,
    0.061668,
    -0.049189,
    0.005539,
    0.041621,
    -0.062406,
    0.043646,
    0.002772,
    -0.047434,
    0.062037,
    -0.037329,
    -0.011034,
    0.052405,
    -0.060567,
    0.030349,
    0.019101,
    -0.056447,
    0.058024,
    -0.022832,
    -0.026828,
    0.059488,
    -0.054450,
    0.014909,
    0.034080,
    -0.061473,
    0.049912,
    -0.006722,
    -0.040728,
    0.062368,
    -0.044487,
    -0.001585,
    0.046652,
    -0.062157,
    0.038274,
    0.009863,
    -0.051750,
    0.060844,
    -0.031382,
    -0.017966,
    0.055929,
    -0.058451,
    0.023933,
    0.025751,
    -0.059117,
    0.055022,
    -0.016060,
    -0.033078,
    0.061256,
    -0.050616,
    0.007902,
    0.039820,
    -0.062308,
    0.045313,
    0.000396,
    -0.045854,
    0.062255,
    -0.039206,
    -0.008687,
    0.051076,
    -0.061098,
    0.032404,
    0.016825,
    -0.055391,
    0.058858,
    -0.025027,
    -0.024663,
    0.058725,
    -0.055573,
    0.017206,
    0.032065,
    -0.061016,
    0.051303,
    -0.009080,
    -0.038897,
    0.062225,
    -0.046122,
    0.000792,
    0.045040,
    -0.062331,
    0.040124,
    0.007509,
    -0.050383,
    0.061331,
    -0.033414,
    -0.015677,
    0.054834,
    -0.059243,
    0.026111,
    0.023567,
    -0.058311,
    0.056104,
    -0.018345,
    -0.031039,
    0.060754,
    -0.051970,
    0.010254,
    0.037961,
    -0.062120,
    0.046915,
    -0.001981,
    -0.044209,
    0.062384,
    -0.041027,
    -0.006328,
    0.049673,
    -0.061541,
    0.034411,
    0.014524,
    -0.054256,
    0.059607,
    -0.027186,
    -0.022462,
    0.057876,
    -0.056615,
    0.019478,
    0.030002,
    -0.060470,
    0.052619,
    -0.011424,
    -0.037010,
    0.061992,
    -0.047690,
    0.003168,
    0.043362,
    -0.062414,
    0.041915,
    0.005144,
    -0.048944,
    0.061729,
    -0.035397,
    -0.013365,
    0.053658,
    -0.059949,
    0.028251,
    0.021349,
    -0.057421,
    0.057105,
    -0.020603,
    -0.028955,
    0.060165,
    -0.053249,
    0.012590,
    0.036047,
    -0.061841,
    0.048448,
    -0.004354,
    -0.042499,
    0.062421,
    -0.042788,
    -0.003959,
    0.048198,
    -0.061894,
    0.036369,
    0.012202,
    -0.053041,
    0.060269,
    -0.029305,
    -0.020229,
    0.056944,
    -0.057575,
    0.021721,
    0.027897,
    -0.059837,
    0.053860,
    -0.013752,
    -0.035070,
    0.061668,
    -0.049189,
    0.005539,
    0.041621,
    -0.062406,
    0.043646,
    0.002772,
    -0.047434,
    0.062037,
    -0.037329,
    -0.011034,
    0.052405,
    -0.060567,
    0.030349,
    0.019101,
    -0.056447,
    0.058024,
    -0.022832,
    -0.026828,
    0.059488,
    -0.054450,
    0.014909,
    0.034080,
    -0.061473,
    0.049912,
    -0.006722,
    -0.040728,
    0.062368,
    -0.044487,
    -0.001585,
    0.046652,
    -0.062157,
    0.038274,
    0.009863,
    -0.051750,
    0.060844,
    -0.031382,
    -0.017966,
    0.055929,
    -0.058451,
    0.023933,
    0.025751,
    -0.059117,
    0.055022,
    -0.016060,
    -0.033078,
    0.061256,
    -0.050616,
    0.007902,
    0.039820,
    -0.062308,
    0.045313,
    0.000396,
    -0.045854,
    0.062255,
    -0.039206,
    -0.008687,
    0.051076,
    -0.061098,
    0.032404,
    0.016825,
    -0.055391,
    0.058858,
    -0.025027,
    -0.024663,
    0.058725,
    -0.055573,
    0.017206,
    0.032065,
    -0.061016,
    0.051303,
    -0.009080,
    -0.038897,
    0.062225,
    -0.046122,
    0.000792,
    0.045040,
    -0.062331,
    0.040124,
    0.007509,
    -0.050383,
    0.061331,
    -0.033414,
    -0.015677,
    0.054834,
    -0.059243,
    0.026111,
    0.023567,
    -0.058311,
    0.056104,
    -0.018345,
    -0.031039,
    0.060754,
    -0.051970,
    0.010254,
    0.037961,
    -0.062120,
    0.046915,
    -0.001981,
    -0.044209,
    0.062384,
    -0.041027,
    -0.006328,
    0.049673,
    -0.061541,
    0.034411,
    0.014524,
    -0.054256,
    0.059607,
    -0.027186,
    -0.022462,
    0.057876,
    -0.056615,
    0.019478,
    0.030002,
    -0.060470,
    0.052619,
    -0.011424,
    -0.037010,
    0.061992,
    -0.047690,
    0.003168,
    0.043362,
    -0.062414,
    0.041915,
    0.005144,
    -0.048944,
    0.061729,
    -0.035397,
    -0.013365,
    0.053658,
    -0.059949,
    0.028251,
    0.021349,
    -0.057421,
    0.057105,
    -0.020603,
    -0.028955,
    0.060165,
    -0.053249,
    0.012590,
    0.036047,
    -0.061841,
    0.048448,
    -0.004354,
    -0.042499,
    0.062421,
    -0.042788,
    -0.003959,
    0.048198,
    -0.061894,
};
float32_t symbolSamplesBuffer[FFT_LENGTH];
float32_t fftBuffer[FFT_LENGTH];
float32_t magnitudeBuffer[FFT_LENGTH / 2];
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
  Serial.begin(115200); //reading as fast as possible
  Serial2.begin(115200);
  memset(chirpSamplesBuffer, 0, sizeof(float32_t) * FFT_LENGTH);
  arm_rfft_fast_init_f32(&fftInstance, FFT_LENGTH);
  freq_one_bin = round((FREQUENCY_ONE * FFT_LENGTH) / SAMPLING_FREQUENCY);
  freq_zero_bin = round((FREQUENCY_ZERO * FFT_LENGTH) / SAMPLING_FREQUENCY);
  analogReadRes(12);
  analogReadAveraging(1);
  pinMode(DEBUG_PIN, OUTPUT);
}

void loop()
{
  sample = analogRead(A0);
  digitalWrite(DEBUG_PIN, 1);
  if (1)
  {
    //read float32_t as binary (4 bytes)
    // size_t readBytes = Serial2.readBytesUntil('\n',uartBuffer,128);
    // uartBuffer[readBytes] = '\0';
    // sample = atoi((char*)uartBuffer);
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
      digitalWrite(DEBUG_PIN, 0);
      for (size_t i = 0; i < CORRELATION_LENGTH; i++)
      {
        if (correlationBuffer[i] > maxCorrelationValue)
          maxCorrelationValue = correlationBuffer[i];
      }
      Serial.printf("Max correlation %f\r\n", maxCorrelationValue);
      // Serial.printf("Sample %u\r\n",sample);
      if (maxCorrelationValue > CORRELATION_THRESHOLD)
      {
        Serial.println("Receiving data");
        symbolBitCounter = symbolByteCounter = symbolBufferIndex = 0;
        receivingData = 1;
      }
    }
  }
}
