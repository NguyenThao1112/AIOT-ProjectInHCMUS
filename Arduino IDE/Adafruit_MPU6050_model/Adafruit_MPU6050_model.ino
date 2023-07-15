/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <motion-detect_inferencing.h>

#define FREQUENCY_HZ 60
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))
#define epsilon 0.1
#define SDA 14
#define SCL 15
//#define PUSH_BUTTON 4

// objeto da classe Adafruit_MPU6050
Adafruit_MPU6050 mpu;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

static unsigned long last_interval_ms = 0;

void setup() {
  Wire.begin(SDA, SCL);
  Serial.begin(115200);
  //pinMode(PUSH_BUTTON, INPUT);

  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  /*
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  */
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  /*
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }
  */

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  /*
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
  */

  //Serial.println("");
  delay(100);

  //Serial.print("Features: ");
  //Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  //Serial.print("Label count: ");
  //Serial.println(EI_CLASSIFIER_LABEL_COUNT);
}

void loop() {
  sensors_event_t a, g, temp;

  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();

    mpu.getEvent(&a, &g, &temp);

    features[feature_ix++] = a.acceleration.x;
    features[feature_ix++] = a.acceleration.y;
    features[feature_ix++] = a.acceleration.z;
    
    if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      //Serial.println("Running the inference...");
      signal_t signal;
      ei_impulse_result_t result;
      int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        //ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
      }

      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);

      if (res != 0) return;

      //ei_printf("Predictions ");
      //ei_printf("(DSP: %d ms., Classification: %d ms.)",
      //          result.timing.dsp, result.timing.classification);
      //ei_printf(": \n");

      //
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        //ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
      }

      size_t max_id = find_Max(result);
      if (result.classification[max_id].value >= 0.6) {
        //Serial.println(result.classification[max_id].label);
        if (result.classification[max_id].label == "left_right") {
          if (features[0] < 0) {
            //Serial.println(a.acceleration.x);
            Serial.println("left");
          }
          else if (features[0] > 0) {
            //Serial.println(a.acceleration.x);
            Serial.println("right");
          }
          else {
            Serial.println("nothing");
          }
        }
        else if (result.classification[max_id].label == "up_down") {
          if (features[1] > 0) {
            Serial.println("up");
          }
          else if (features[1] < 0) {
            Serial.println("down");
          }
          else {
            Serial.println("nothing");
          }
        }
      }
      else {
        size_t flag = 0;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT - 1; ix++) {
          if (ix != max_id && abs(result.classification[ix].value - result.classification[max_id].value) <= epsilon) {
            Serial.println("nothing");
            flag = -1;
            break;                 
          }
        }

        if (flag == 0) {
          //Serial.println(result.classification[max_id].label);
          if (result.classification[max_id].label == "left_right") {
            if (features[0] < 0) {
              //Serial.println(a.acceleration.x);
              Serial.println("left");
            }
            else if (features[0] > 0) {
              //Serial.println(a.acceleration.x);
              Serial.println("right");
            }
            else {
              Serial.println("nothing");
            }
          }
          else if (result.classification[max_id].label == "up_down") {
            if (features[1] > 0) {
              Serial.println("up");
            }
            else if (features[1] < 0) {
              Serial.println("down");
            }
            else {
              Serial.println("nothing");
            }
          }
        }
      }
      
      feature_ix = 0;
    }
  }
}
/*
void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);
  }
}
*/
size_t find_Max(ei_impulse_result_t arr) {
  size_t maxid = 0;
  for (size_t ix = 1; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (arr.classification[ix].value > arr.classification[maxid].value) {
      maxid = ix;
    }
  }
  return maxid;
}