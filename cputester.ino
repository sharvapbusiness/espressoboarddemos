#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* set displayEquation to true to display equation, core, time info 
   set displayEquation to false to only display core, time info
   
   set time unit according to what unit of time you would like to display
   set to "1" display in microseconds
   set to "2" display in milliseconds
   set to "3" display in seconds

   Change between the options anytime by simply typing 1, 2, or 3 in the serial monitor at any time.
*/

bool displayEquation = false; 
int timeUnit = 3; 

void taskCore0(void *parameter);
void taskCore1(void *parameter);

void setup() {
  Serial.begin(115200);
  delay(1000); 
  xTaskCreatePinnedToCore(taskCore0, "TaskCore0", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskCore1, "TaskCore1", 10000, NULL, 1, NULL, 1);
}

void loop() {
  if (Serial.available() > 0) {
    int userInput = Serial.parseInt();
    if (userInput >= 1 && userInput <= 3) {
      timeUnit = userInput;
      Serial.print("Time unit set to ");
      if (timeUnit == 1) {
        Serial.println("microseconds");
      } else if (timeUnit == 2) {
        Serial.println("milliseconds");
      } else if (timeUnit == 3) {
        Serial.println("seconds");
      }
    } else {
      Serial.println("Invalid input. Enter 1 for microseconds, 2 for milliseconds, or 3 for seconds.");
    }
    delay(1000); 
  }
}

float solveCubicEquation(float a, float b, float c, float d);

void taskCore0(void *parameter) {
  for (;;) {
    float a = random(10000, 1000000); // Coefficient for x^3
    float b = random(-100000, 100000); // Coefficient for x^2
    float c = random(-100000, 100000); // Coefficient for x
    float d = random(-100000, 100000); // Constant term

    if (displayEquation) {
      Serial.print("Core 0: Equation: " + String(a) + "x^3 + " + String(b) + "x^2 + " + String(c) + "x + " + String(d));
    }
    unsigned long startTime = micros();

    for (int i = 0; i < 100000; ++i) {
      float result = sqrt(i) * sin(i) + cos(i);
    }

    float root = solveCubicEquation(a, b, c, d);
    unsigned long endTime = micros();
    if (displayEquation) {
      Serial.println(", Root: " + String(root));
    }
    Serial.print("Core 0: Time taken to solve the equation: ");
    printTime(endTime - startTime);
    delay(1000);
  }
}

// Task function for Core 1
void taskCore1(void *parameter) {
  for (;;) {
    float a = random(10000, 1000000); // Coefficient for x^3
    float b = random(-100000, 100000); // Coefficient for x^2
    float c = random(-100000, 100000); // Coefficient for x
    float d = random(-100000, 100000); // Constant term

    if (displayEquation) {
      Serial.print("Core 1: Equation: " + String(a) + "x^3 + " + String(b) + "x^2 + " + String(c) + "x + " + String(d));
    }

    unsigned long startTime = micros();

    for (int i = 0; i < 100000; ++i) {
      float result = sqrt(i) * sin(i) + cos(i);
    }

    float root = solveCubicEquation(a, b, c, d);
    unsigned long endTime = micros();
    if (displayEquation) {
      Serial.println(", Root: " + String(root));
    }
    Serial.print("Core 1: Time taken to solve the equation: ");
    printTime(endTime - startTime);
    delay(1000);
  }
}

float solveCubicEquation(float a, float b, float c, float d) {
  float q = (3 * a * c - b * b) / (9 * a * a);
  float r = (9 * a * b * c - 27 * a * a * d - 2 * b * b * b) / (54 * a * a * a);
  float delta = q * q * q + r * r;

  if (delta >= 0) {
    float s = cbrt(r + sqrt(delta));
    float t = cbrt(r - sqrt(delta));
    float root = -b / (3 * a) - (s + t) / 2;
    return root;
  } else {
    float theta = acos(r / sqrt(-q * q * q));
    float root1 = 2 * sqrt(-q) * cos(theta / 3) - b / (3 * a);
    return root1;
  }
}

void printTime(unsigned long microseconds) {
  if (timeUnit == 1) {
    Serial.print(microseconds);
    Serial.println(" microseconds");
  } else if (timeUnit == 2) {
    Serial.print(microseconds / 1000);
    Serial.println(" milliseconds");
  } else if (timeUnit == 3) {
    Serial.print(microseconds / 1000000.0);
    Serial.println(" seconds");
  }
}
