// esp32s3 dev module, USB CDC on boot enabled
#include <math.h>
#define SCL 1
#define SDA 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  
  pinMode(SCL, OUTPUT_OPEN_DRAIN);  // SCL is always output
  pinMode(SDA, OUTPUT_OPEN_DRAIN); 
  digitalWrite(SCL, HIGH);
  digitalWrite(SDA, HIGH);
  delay(1000);

  bool setup = false;

  // I2C write to MPU-6050

  uint8_t rw = 0x00;
  while (!setup) {
    startTransaction();  // initiate start condition

    sendAddress(0x68, rw);  // send device address and write
    if (!readACK()) {
      Serial.printf("NACK recieved on setup 1\n");
      stopTransaction();
    } else {
      setup = true;
    }
    
  }

  sendData(0x6B);  // send register address
  if (!readACK()) {
    Serial.printf("NACK recieved on setup 2\n");
    return;
  }

  sendData(0x00);  // write data to target register address
  if (!readACK()) {
    Serial.printf("NACK recieved on setup 3\n");
    return;
  }

  stopTransaction();  // stop condition


  Serial.printf("Setup Complete!\n");
}


void loop() {


  int16_t accel_xraw, accel_yraw, accel_zraw;  // initialize raw acceleration variables
  uint8_t raw_data[6];       // array with 6 bytes of data

  startTransaction();
  uint8_t rw = 0x00;
  sendAddress(0x68, rw);  // send device address and write
  if (!readACK()) {
    Serial.printf("NACK recieved\n");
    stopTransaction();
  }
  sendData(0x3B);  // send register address 0x3B
  if (!readACK()) { Serial.printf("NACK recieved\n"); }
  stopTransaction();

  startTransaction();  // 2nd transaction to send device register with read bit
  rw = 0x01;
  sendAddress(0x68, rw);  // send device address and read
  if (!readACK()) { Serial.printf("NACK recieved\n"); }

  for (int i = 0; i < 6; ++i) {
    raw_data[i] = readData();

    if (i == 5) sendACK(false);  // send nack on last one so slave stops
    else sendACK(true);          // send ack so slave continues
  }

  stopTransaction();

  // assemble into signed 16-bit values
  accel_xraw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  accel_yraw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  accel_zraw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  float accel_x = accel_xraw / 16384.0f;
  float accel_y = accel_yraw / 16384.0f;
  float accel_z = accel_zraw / 16384.0f;

  float angleXY = atan2(accel_x, accel_y);
  float angleXZ = atan2(accel_x, accel_z);
  float angleYZ = atan2(accel_y, accel_z);

  Serial.printf("XY angle: %0.2f degrees\n", angleXY * RAD_TO_DEG);
  Serial.printf("XZ angle: %0.2f degrees\n", angleXZ * RAD_TO_DEG);
  Serial.printf("YZ angle: %0.2f degrees\n", angleYZ * RAD_TO_DEG);

  delayMicroseconds(1000000);  // delay 1 second each loop
}


void startTransaction() {  // drives SDA low while SCL is high
  digitalWrite(SCL, HIGH);
  delayMicroseconds(1);
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA, HIGH);  // make sure SDA is high
  delayMicroseconds(1);
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL, LOW);  // set scl low for transaction to start
  delayMicroseconds(1);
}


void stopTransaction() {  // drives SDA high while SCL is high
  digitalWrite(SCL, LOW);
  delayMicroseconds(1);

  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA, LOW);
  delayMicroseconds(1);

  digitalWrite(SCL, HIGH);
  delayMicroseconds(1);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(10);

  digitalWrite(SCL, HIGH);  // release SCL and SDA
  digitalWrite(SDA, HIGH);
  delayMicroseconds(1);
}


void sendAddress(uint8_t address, uint8_t rw) {

  uint8_t out = (address << 1) | rw;

  for (int i = 7; i >= 0; --i) {
    digitalWrite(SCL, LOW);  // only change while scl is low
    delayMicroseconds(1);

    pinMode(SDA, OUTPUT_OPEN_DRAIN);  // make sure SDA is set to output
    if (out & (1 << i)) {
      digitalWrite(SDA, HIGH);  // write 1 if the ith bit is 1
    } else {
      digitalWrite(SDA, LOW);  // else write 0
    }

    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
    digitalWrite(SCL, LOW);
    delayMicroseconds(1);
  }

  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA, HIGH);  // release SDA
  delayMicroseconds(1);
}

bool readACK() {
  digitalWrite(SCL, LOW);  // make sure scl starts low
  delayMicroseconds(1);

  pinMode(SDA, INPUT_PULLUP);  // set SDA to input so slave can control
  delayMicroseconds(1);

  digitalWrite(SCL, HIGH);  // change clock to high
  delayMicroseconds(10);

  bool ack = (digitalRead(SDA) == LOW);  // if SDA is low, ack is true, if SDA is high, there was an error

  digitalWrite(SCL, LOW);
  delayMicroseconds(1);

  pinMode(SDA, OUTPUT_OPEN_DRAIN);  // take back control of SDA
  digitalWrite(SDA, HIGH);

  return ack;
}

void sendData(uint8_t data) {
  for (int i = 7; i >= 0; --i) {
    digitalWrite(SCL, LOW);  // only change while scl is low
    delayMicroseconds(1);

    pinMode(SDA, OUTPUT_OPEN_DRAIN);  // make sure SDA is set to output
    if (data & (1 << i)) {
      digitalWrite(SDA, HIGH);  // write 1 if the ith bit is 1
    } else {
      digitalWrite(SDA, LOW);  // else write 0
    }

    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
    digitalWrite(SCL, LOW);
    delayMicroseconds(1);
  }
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA, HIGH);  // release SDA
  delayMicroseconds(1);
}


uint8_t readData() {
  uint8_t data = 0x00;
  pinMode(SDA, INPUT_PULLUP);  // change SDA to input
  digitalWrite(SCL, LOW);      // start clock low
  delayMicroseconds(1);

  for (int i = 7; i >= 0; --i) {
    digitalWrite(SCL, HIGH);  // pulse clock high
    delayMicroseconds(10);

    if (digitalRead(SDA)) {
      data = data | (1 << i);
    }
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
  }
  return data;
}


void sendACK(bool ack) {
  digitalWrite(SCL, LOW);           // make sure clk is low
  pinMode(SDA, OUTPUT_OPEN_DRAIN);  // make sure SDA is output
  delayMicroseconds(1);

  if (ack) {
    digitalWrite(SDA, LOW);
  } else {
    digitalWrite(SDA, HIGH);  // nack
  }
  delayMicroseconds(10);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(SCL, LOW);
  delayMicroseconds(10);

}
