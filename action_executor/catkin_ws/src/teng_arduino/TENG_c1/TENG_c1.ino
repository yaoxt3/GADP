int sensorValue;
float outputValue;
int sensorValue1;
float outputValue1;
int sensorValue2;
float outputValue2;
int sensorValue3;
float outputValue3; 


String a, b;
int flag = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(analogRead(0));

}

void loop() {
  if (Serial.available() > 0)
  {
    String rx = Serial.readString();
    if (flag == 0)
      flag = 1;
    else
      flag = 0;

  }



  //if (flag > 0){
  if (1) {
    Serial.flush();
    // put your main code here, to run repeatedly:
    sensorValue = analogRead(A0);
    outputValue = sensorValue / 1023.0 * 5.0;
//
//    sensorValue1 = analogRead(A2);/

    sensorValue2 = analogRead(A4);
    outputValue2 = sensorValue2 / 1023.0 * 5.0;
//
//    sensorValue3 = analogRead(A7);
//    outputValue3 = sensorValue3 / 1023.0 * 5.0;
//


 
    a = String(outputValue)+ "," + String(outputValue2);
    Serial.println(a);
  }
    delay(70);
}
