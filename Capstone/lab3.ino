// Luke Peter CSCE 491 Spring 2026

float m[16];
float b[16]; 

void setup() {
  Serial.begin(115200);
  Serial.printf("Entering auto-calibration mode, please wait\n");
  // initialized U2UXD at 9600 baud with pin 12 as rx and pin 13 as tx
  Serial2.begin(9600, SERIAL_8N1, 12, 13);
  Serial2.print("*IDN?\r\n");        // queries psu for identification
  Serial2.print("SYST:REM\r\n");     // puts psu in remote mode.  Use Serial2.print("*SYST:LOC\r\n"); to re-enable front panel
  Serial2.print("INST:NSEL 1\r\n");  // selects channel 1
  Serial2.print("OUTP 1\r\n");
  Serial.printf("Taking control of power supply...success\n");


  pinMode(15, INPUT);  // sets pin mode to input so voltage can be read
  float x[285]; 
  float y[285];
  float yvolt = 0;

  Serial.printf("Autocalibrating");
  // y is the actual voltage the psu is set to, x is the voltage read after y is set
  for (int i = 0; i < 285; i++) {
    yvolt = (float)i * 0.05;
    y[i] = (float)yvolt * 0.232558;  // y voltage converted to 0-3.3 scale for calculations 
    //Serial.printf("chunk %d: yvolt = %f, y[i] = %f\n", i, yvolt, y[i]);
    Serial2.printf("VOLT %.2f\r\n", yvolt);  // sets voltage of channel 1 to y volts
    delay(500);
    int xint = analogRead(15);  // reads voltage as int from 0-4095
    x[i] = (float)xint * 0.000806;  // converts voltage to float value between 0-3.3
  }


  
  float sumxy[16];
  float sumx[16];
  float sumy[16];
  float sumx2[16];
  int index = 0;
  int n = 18;
  for (int i = 0; i < 16; i++) {  // calculate the 16 different sums
    if (i == 15) { n = 15; }  // on final loop set n to 15
      for (int j = 0; j < n; j++) {  // first 15 sums will cover 18 voltages each (n is 18)
        sumxy[i] += x[index] * y[index];
        sumx[i] += x[index];
        sumy[i] += y[index];
        sumx2[i] += x[index] * x[index];
        index++;  // increase index each time to go to next voltage
        //Serial.printf("chunk %d: sumxy = %f, sumx = %f\n", i, sumxy[i], sumx[i]);
      }
  }

  n = 18;
  // now that we have all the sums, calculate m and b
  for(int i = 0; i<16; i++) {
    if(i == 15) { n = 15; }  // set n to 15 on last loop
    m[i] = (n*sumxy[i] - sumx[i]*sumy[i])/(n*sumx2[i]-(sumx[i]*sumx[i]));
    b[i] = (sumy[i]*sumx2[i] - sumx[i]*sumxy[i])/(n*sumx2[i]-(sumx[i]*sumx[i]));
    //Serial.printf("chunk %d: y = %f x + %f\n", i, m[i], b[i]);
  }

  Serial2.print("SYST:LOC\r\n");  // release control of psu to front panel
  Serial.printf("Entering monitoring mode");
}



void loop() {
  int rawint = analogRead(15);
  float raw = (float)rawint * (3.3f / 4095.f);  // converts to raw 0-3.3 volt
  float corrected = 0;
  int index = rawint / 256;  // should give an index 0-15
  corrected = (m[index]*raw) + b[index];
  float actual = corrected * (43.f / 10.f);
  Serial.printf("Raw voltage: %.2f V [ADC read as] %d, corrected voltage: %.2f V,\n   actual voltage: %.2f V\n", raw, rawint, corrected, actual);
  //Serial.printf(" [ADC read as] %d", rawint);
  //Serial.printf(", corrected voltage: %.2f V", corrected);
  //Serial.printf(",\n   actual voltage: %.2f V\n", actual);
  delay(1000);

}








