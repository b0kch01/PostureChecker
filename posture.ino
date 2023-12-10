#include <Wire.h>
// #include <SoftwareSerial.h>

// SoftwareSerial EEBlue(10, 11); // RX | TX

const int MPU_ADDR = 0x68;   // I2C address of the MPU-6050
const int RUNNING_AVG = 100;  // Running average size
const int CYCLE_DELAY = 2000;  // How long each report takes

float elapsedTime, currentTime, previousTime;

// global storage
float gyro_x, gyro_y, gyro_z;

void setup() {
  Serial.begin(9600);
  // EEBlue.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.println("Starting...");

  delay(2000);
  calculate_IMU_error();
}

// get average
float average(float *array) {
  float sum = 0;
  for (int i = 0; i < RUNNING_AVG; ++i) {
    sum += *(array++);
  }
  return sum / RUNNING_AVG;
}

struct GyroData {
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

void pretty(float number) {
  // Check if the number is negative
  bool isNegative = number < 0;

  // Extract the integer and fractional parts of the number
  int intPart = (int)abs(number);
  int fracPart = (int)(abs(number - intPart) * 100);

  // Print the formatted number with padding
  Serial.print("    ");  // Adjust the number of spaces as needed
  if (isNegative && intPart == 0) {
    Serial.print("-");  // Print the negative sign for numbers between -1.0 and 0.0
  } else {
    Serial.print("+");
  }
  Serial.print(intPart);
  Serial.print(".");

  // Ensure two digits in the fractional part
  if (fracPart < 10) {
    Serial.print("0");
  }
  Serial.print(fracPart);
}

void logData(struct GyroData data) {
  Serial.println("\033[0H\033[0J");

  Serial.println("     roll    pitch      yaw");

  pretty(data.gyro_x);
  pretty(data.gyro_y);
  pretty(data.gyro_z);

  float sum = abs(data.gyro_x) + abs(data.gyro_y) + abs(data.gyro_z);
  Serial.println("\n");
  Serial.println(sum > 0.5 ? "    STRAIGHTEN YOUR BACK!! :((" : "         doing great :)");


  // Serial.print(data.gyro_x);
  // Serial.print("/");
  // Serial.print(data.gyro_y);
  // Serial.print("/");
  // Serial.println(data.gyro_z);

  // EEBlue.write(data.gyro_y);
}

struct GyroData grabData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                     // starting with register 0x3B (GYRO_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false);          // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 6, true);  // request a total of 7*2=14 registers

  float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
  float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
  float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value

  float accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 76.45;  // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  float accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 4.85;

  previousTime = currentTime;                         // Previous time is stored before the actual time read
  currentTime = millis();                             // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);  // Read 4 registers total, each axis value is stored in 2 registers

  float GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  float GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  float GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // error correction
  GyroX -= 0.69;
  GyroY -= 3.17;
  GyroZ -= 1.52;


  float gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  float gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  float yaw = yaw + GyroZ * elapsedTime;

  float roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  float pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  return { roll, pitch, yaw };
}

void loop() {
  float runningAverage_x[RUNNING_AVG];
  float runningAverage_y[RUNNING_AVG];
  float runningAverage_z[RUNNING_AVG];

  for (int i = 0; i < RUNNING_AVG; ++i) {
    struct GyroData results = grabData();

    runningAverage_x[i] = results.gyro_x;
    runningAverage_y[i] = results.gyro_y;
    runningAverage_z[i] = results.gyro_z;

    delay(CYCLE_DELAY / RUNNING_AVG);
  }

  logData({ average(runningAverage_x),
            average(runningAverage_y),
            average(runningAverage_z) });
}

void calculate_IMU_error() {
  float AccX, AccY, AccZ, AccErrorX, AccErrorY;
  float GyroX, GyroY, GyroZ, GyroErrorX, GyroErrorY, GyroErrorZ;
  int c = 0;
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}