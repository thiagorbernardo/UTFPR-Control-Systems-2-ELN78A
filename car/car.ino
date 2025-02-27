// LibrerÃ­a para el ESP32
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
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

      Serial.print("Distance (cm): ");
      Serial.println(distanceCm);

      return distanceCm;
    }
    else
    {
      Serial.println("Out of range");
      return -1;
    }
  }
};

LaserSensor laserSensor;

float distance = 0;

// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Callback for handling BLE connections
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("Device disconnected");
    // Stop the car when disconnected
    forwardAxis.stop();
    backwardAxis.stop();
  }
};

// Callback for handling received commands
class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0)
    {
      char command = value[0];
      handleCommand(command);
    }
  }
};

void setup()
{
  Serial.begin(115200);

  // Initialize BLE
  BLEDevice::init("ESP32-Car");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE device ready to connect!");

  // Initialize laser sensor
  if (!laserSensor.begin())
  {
    Serial.println("Failed to initialize laser sensor!");
    while (1)
      ;
  }
}

void handleCommand(char command)
{
  static bool isMovingForward = false;
  static int speed = 255;

  command = toupper(command);

  switch (command)
  {
  case 'W':
    distance = laserSensor.getDistance();
    if (distance > 0 && distance <= 5)
    {
      Serial.println("Obstacle detected within 5cm, cannot move forward");
      forwardAxis.stop();
      backwardAxis.stop();
      isMovingForward = false;
    }
    else
    {
      Serial.println("No obstacle detected, moving forward");
      forwardAxis.moveForward(speed);
      backwardAxis.moveForward(speed);
      isMovingForward = true;
    }
    break;
  case 'S':
    Serial.println("Moving backward");
    forwardAxis.moveBackward(speed);
    backwardAxis.moveBackward(speed);
    isMovingForward = false;
    break;
  case 'A':
    Serial.println("Moving left");
    forwardAxis.moveLeft(speed);
    backwardAxis.moveLeft(speed);
    isMovingForward = false;
    break;
  case 'D':
    Serial.println("Moving right");
    forwardAxis.moveRight(speed);
    backwardAxis.moveRight(speed);
    isMovingForward = false;
    break;
  case 'X':
    Serial.println("Stopping");
    forwardAxis.stop();
    backwardAxis.stop();
    isMovingForward = false;
    break;
  default:
    Serial.println("Invalid command");
    forwardAxis.stop();
    backwardAxis.stop();
    isMovingForward = false;
    break;
  }
}

void loop()
{
  static bool isMovingForward = false;

  // Handle BLE connection events
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // Give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // Restart advertising
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }

  // Continuously check distance if moving forward
  if (isMovingForward)
  {
    distance = laserSensor.getDistance();
    if (distance > 0 && distance <= 5)
    {
      Serial.println("Obstacle detected within 5cm, stopping");
      forwardAxis.stop();
      backwardAxis.stop();
      isMovingForward = false;
    }
  }
}