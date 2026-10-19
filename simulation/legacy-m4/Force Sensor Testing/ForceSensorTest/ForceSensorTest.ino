const int sensorPin = A0;
const int V_inPin = A1;
const int samples = 100;  // number of readings to average

const int a = 1187.7440;
const int b = -548.7972-180;

void setup() {
  Serial.begin(115200);
}

void loop() {
  float ForceSum = 0;
  float VinSum = 0;

  // take 5 readings and average them
  for (int i = 0; i < samples; i++) {
    ForceSum += analogRead(sensorPin);
    VinSum += analogRead(V_inPin);
  }

  float ForceAnalog = ForceSum / samples;

  float ForceVoltage = ForceAnalog * (5.0 / 1023.0);


  float force = a*ForceVoltage + b;
  // Serial.print("Voltage(V):");
  // Serial.print(ForceVoltage, 3);
  // Serial.print(" ,");
  Serial.print("Force_1(N):");
  Serial.println(force, 3);
  // Serial.print(" , ");
  // Serial.print(ForceVoltage, 3);
  // Serial.print(",");
  // Serial.print("Force_1(N):");
  // Serial.println(a*VoltageIn + b, 3);




  delay(50);  // overall update rate
}
