/*  AR4 Annin Robot Control Software Arduino Nano sketch
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling AR2 software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

    Log:

*/


#include <Servo.h>

String inData;


Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;


const int Input2 = 2;
const int Input3 = 3;
const int Input4 = 4;
const int Input5 = 5;
const int Input6 = 6;
const int Input7 = 7;

const int Output8 = 8;
const int Output9 = 9;
const int Output10 = 10;
const int Output11 = 11;
const int Output12 = 12;
const int Output13 = 13;

// ---------- ACS712 (5A) current sensor ----------
const int ACS_PIN = A7;        // ACS712 AOUT wired here
const float ACS_VCC = 5.0;     // Nano analog reference
const float ACS_SENS = 0.185;  // 185 mV per amp (5A version)

int acsZeroRaw = 512;  // will be calibrated at startup

// ---- Gripper tracking ----
int gripperCmd = 20;  // last commanded gripper position
const int GRIPPER_MIN = 0;
const int GRIPPER_MAX = 45;

// ---- Current limit behavior ----
const float CURRENT_LIMIT = 0.8;              // amps
const unsigned long BACKOFF_INTERVAL = 3000;  // 3 seconds

unsigned long lastBackoffMs = 0;


void setup() {
  // run once:
  Serial.begin(9600);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, INPUT);


  pinMode(Input2, INPUT_PULLUP);
  pinMode(Input3, INPUT_PULLUP);
  pinMode(Input4, INPUT_PULLUP);
  pinMode(Input5, INPUT_PULLUP);
  pinMode(Input6, INPUT_PULLUP);
  pinMode(Input7, INPUT_PULLUP);


  pinMode(Output8, OUTPUT);
  pinMode(Output9, OUTPUT);
  pinMode(Output10, OUTPUT);
  pinMode(Output11, OUTPUT);
  pinMode(Output12, OUTPUT);
  pinMode(Output13, OUTPUT);


  servo0.attach(A0);
  servo1.attach(A1);
  servo2.attach(A2);
  servo3.attach(A3);
  servo4.attach(A4);
  servo5.attach(A5);
  servo6.attach(A6);

  servo0.write(20);

  // Calibrate ACS712 zero-current offset
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += analogRead(ACS_PIN);
    delay(2);
  }
  acsZeroRaw = sum / 200;
}

float readAcsCurrent() {
  const int samples = 20;
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(ACS_PIN);
  }

  float raw = sum / (float)samples;

  // Convert ADC reading to voltage
  float volts = (raw * ACS_VCC) / 1023.0;
  float zeroVolts = (acsZeroRaw * ACS_VCC) / 1023.0;

  // Convert voltage difference to current
  float amps = (volts - zeroVolts) / ACS_SENS;

  // amps absolute value - magnitude
  if (amps < 0) amps = -amps;

  return amps;
}

void gripperBackoff() {
  float amps = readAcsCurrent();
  unsigned long now = millis();

  if (amps > CURRENT_LIMIT) {
    if (now - lastBackoffMs >= BACKOFF_INTERVAL) {
      lastBackoffMs = now;

      if (gripperCmd < GRIPPER_MAX) {
        gripperCmd += .25;
        if (gripperCmd > GRIPPER_MAX) gripperCmd = GRIPPER_MAX;

        servo0.write(gripperCmd);
      }
    }
  } else {
    // below threshold → reset timer so it must stay high again
    lastBackoffMs = now;
  }
}

void loop() {
  //start loop
  gripperBackoff();
  while (Serial.available() > 0) {
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n') {
      String function = inData.substring(0, 2);

      //-----COMMAND TEST GRIPPER AMPERAGE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "TG") {
        int raw = analogRead(A7);
        float volts = raw * (5.0 / 1023.0);
        Serial.println(readAcsCurrent(), 3);
        //Serial.println(volts, 3);
      }


      //-----COMMAND TO MOVE SERVO---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "SV") {
        int SVstart = inData.indexOf('V');
        int POSstart = inData.indexOf('P');
        int servoNum = inData.substring(SVstart + 1, POSstart).toInt();
        int servoPOS = inData.substring(POSstart + 1).toInt();
        if (servoNum == 0) {
          gripperCmd = servoPOS;
          servo0.write(gripperCmd);
        }
        if (servoNum == 1) {
          servo1.write(servoPOS);
        }
        if (servoNum == 2) {
          servo2.write(servoPOS);
        }
        if (servoNum == 3) {
          servo3.write(servoPOS);
        }
        if (servoNum == 4) {
          servo4.write(servoPOS);
        }
        if (servoNum == 5) {
          servo5.write(servoPOS);
        }
        if (servoNum == 6) {
          servo6.write(servoPOS);
        }

        Serial.print("Servo Done");
      }


      //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "JF") {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH) {
          Serial.println("T");
        }
        if (digitalRead(IJInputNum) == LOW) {
          Serial.println("F");
        }
      }
      //-----COMMAND SET OUTPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "ON") {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        Serial.print("Done");
      }
      //-----COMMAND SET OUTPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "OF") {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT 5v INPUT---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WI") {
        int inputVal = -1;
        int inputIndex = inData.indexOf('A');
        int valueIndex = inData.indexOf('B');
        int timoutIndex = inData.indexOf('C');
        int input = inData.substring(inputIndex + 1, valueIndex).toInt();
        int value = inData.substring(valueIndex + 1, timoutIndex).toInt();
        int timeout = inData.substring(timoutIndex + 1).toInt();

        unsigned long timeoutMillis = (unsigned long)timeout * 1000;
        unsigned long startTime = millis();
        bool aborted = false;

        while ((millis() - startTime < timeoutMillis) && (inputVal != value)) {
          inputVal = digitalRead(input);

          // Check for incoming serial stop/cancel command
          if (Serial.available() > 0) {
            String stopCmd = Serial.readStringUntil('\n');
            stopCmd.trim();

            if (stopCmd == "STOPWI" || stopCmd == "STOP") {
              aborted = true;
              break;
            }
          }

          delay(10);
        }

        delay(5);

        if (aborted) {
          Serial.println("Nano Stopped");
        } else if (inputVal == value) {
          Serial.println("Done");
        } else {
          Serial.println("Timeout");
        }
      }

      //-----STOP COMMAND ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "ST") {
        Serial.println("Nano Inactive Stopped");
      }

      //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "TM") {
        String echo = inData.substring(2);
        Serial.println(echo);
      }




      else {
        inData = "";  // Clear recieved buffer
      }
    }
  }
}
