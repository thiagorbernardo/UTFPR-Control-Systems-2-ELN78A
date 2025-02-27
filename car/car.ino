// LibrerÃ­a para el ESP32
#include <WiFi.h>
#include <Adafruit_VL53L0X.h>

class AxisController
{
private:
  int ENA;
  int ENA_CHANNEL;
  int IN1;
  int IN2;
  int IN3;
  int IN4;
  int ENB;
  int ENB_CHANNEL;

  const int frequency = 5000;
  const int resolution = 8;

public:
  AxisController(int ENA, int IN1, int IN2, int IN3, int IN4, int ENB, int ENA_CHANNEL, int ENB_CHANNEL)
  {
    this->ENA = ENA;
    this->IN1 = IN1;
    this->IN2 = IN2;
    this->IN3 = IN3;
    this->IN4 = IN4;
    this->ENB = ENB;
    this->ENA_CHANNEL = ENA_CHANNEL;
    this->ENB_CHANNEL = ENB_CHANNEL;

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    ledcSetup(ENA_CHANNEL, frequency, resolution);
    ledcSetup(ENB_CHANNEL, frequency, resolution);
    ledcAttachPin(ENA, ENA_CHANNEL);
    ledcAttachPin(ENB, ENB_CHANNEL);
  }

  void moveForward(int speed)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    setSpeed(speed);
  }

  void moveBackward(int speed)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    setSpeed(speed);
  }

  void moveLeft(int speed)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    setSpeed(speed);
  }

  void moveRight(int speed)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    setSpeed(speed);
  }

  void stop()
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    setSpeed(0);
  }

  void setSpeed(int speed)
  {
    ledcWrite(ENA_CHANNEL, speed);
    ledcWrite(ENB_CHANNEL, speed);
  }
};

// Pines del puente H L298N
const int F_ENA = 4;  // Preto
const int F_IN1 = 16; // Branco
const int F_IN2 = 17; // Cinza
const int F_IN3 = 18; // Roxo
const int F_IN4 = 12; // Azul
const int F_ENB = 23; // Verde

const int B_ENA = 32; // Azul
const int B_IN1 = 33; // Verde
const int B_IN2 = 25; // Amarelo
const int B_IN3 = 26; // Laranja
const int B_IN4 = 27; // Vermelho
const int B_ENB = 14; // Marrom

AxisController forwardAxis(F_ENA, F_IN1, F_IN2, F_IN3, F_IN4, F_ENB, 0, 1);
AxisController backwardAxis(B_ENA, B_IN1, B_IN2, B_IN3, B_IN4, B_ENB, 0, 1);

// comando começa em 20, recua ate 30, vai ate 8

class LaserSensor
{
private:
  Adafruit_VL53L0X lox;
  bool initialized;

public:
  LaserSensor() : lox(Adafruit_VL53L0X())
  {
    initialized = false;
  }

  bool begin()
  {
    Serial.println("Initializing laser sensor...");
    initialized = lox.begin();
    if (!initialized)
    {
      Serial.println(F("Failed to boot VL53L0X"));
    }
    return initialized;
  }

  float getDistance()
  {
    if (!initialized)
    {
      return -1;
    }

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4)
    {
      // Convert from millimeters to centimeters for consistency with ultrasonic sensor
      float distanceCm = measure.RangeMilliMeter / 10.0;

      // Serial.print("Distance: ");
      // Serial.println(distanceCm);

      return distanceCm;
    }
    else
    {
      // Serial.println("Out of range");
      return -1;
    }
  }
};

LaserSensor laserSensor;

float distance = 0;
int speed = 220;

enum State
{
  WAITING_TO_START,
  STABILIZING_30,
  STABILIZING_8,
  STOPPED
};

State currentState = WAITING_TO_START;
unsigned long stateStartTime = 0;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100;      // 0.1 seconds
const unsigned long STABILIZE_30_DURATION = 6; // 4 seconds for each goal
const unsigned long TOTAL_DURATION = 40;       // 40 seconds total

float goalDistance = 30.0; // Starting goal is 30cm

// Add this variable to track total runtime
unsigned long experimentStartTime = 0;

// Add this constant near the other constants
const float TOLERANCE = 0.5; // ±0.5cm tolerance

void setup()
{
  Serial.begin(115200);

  // Initialize laser sensor
  if (!laserSensor.begin())
  {
    Serial.println("Failed to initialize laser sensor!");
    while (1)
      ;
  }

  // Serial.println("Waiting for command...");
  // Serial.println("Commands: 'W' (forward), 'S' (backward), 'A' (left), 'D' (right), 'X' (stop)");
}

bool hasStarted = false;
float stepTime = 0.0;

void loop()
{
  unsigned long currentTime = millis();
  distance = laserSensor.getDistance();

  // Send data to the serial port every 0.1 seconds
  if (currentTime - lastPrintTime >= PRINT_INTERVAL && currentState != STOPPED)
  {
    Serial.print(stepTime);
    Serial.print(",");
    Serial.print(goalDistance);
    Serial.print(",");
    Serial.println(distance);
    lastPrintTime = currentTime;
    stepTime += 0.1;
  }

  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    if (command.equals("START"))
    {
      hasStarted = true;
      experimentStartTime = currentTime;
      currentState = WAITING_TO_START;
      stepTime = 0.0;
      Serial.println("DISCARD_DATA");
    }
    return;
  }

  if (!hasStarted)
  {
    return;
  }

  if (stepTime >= TOTAL_DURATION)
  {
    currentState = STOPPED;
    forwardAxis.stop();
    backwardAxis.stop();
    Serial.println("STOP");
    return;
  }

  switch (currentState)
  {
  case WAITING_TO_START:
    currentState = STABILIZING_30;
    stateStartTime = currentTime;
    goalDistance = 30.0;
    stepTime = 0.0;
    break;

  case STABILIZING_30:
    if (stepTime >= STABILIZE_30_DURATION)
    {
      currentState = STABILIZING_8;
      stateStartTime = currentTime;
      goalDistance = 8.0;
    }
    else
    {
      if (distance < goalDistance - TOLERANCE)
      {
        forwardAxis.moveBackward(speed);
        backwardAxis.moveBackward(speed);
      }
      else if (distance > goalDistance + TOLERANCE)
      {
        forwardAxis.moveForward(speed);
        backwardAxis.moveForward(speed);
      }
      else
      {
        forwardAxis.stop();
        backwardAxis.stop();
      }
    }
    break;

  case STABILIZING_8:
    if (distance < goalDistance - TOLERANCE)
    {
      forwardAxis.moveBackward(speed);
      backwardAxis.moveBackward(speed);
    }
    else if (distance > goalDistance + TOLERANCE)
    {
      forwardAxis.moveForward(speed);
      backwardAxis.moveForward(speed);
    }
    else
    {
      forwardAxis.stop();
      backwardAxis.stop();
    }
    break;

  case STOPPED:
    forwardAxis.stop();
    backwardAxis.stop();
    Serial.println("STOP");
    break;
  }
}