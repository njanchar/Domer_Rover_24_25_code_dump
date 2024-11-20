int num_joints = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    char c = Serial.read();
    if(c=='r'){
      float j[] = {3.14,3.14,3.14,3.14,3.14,3.14,3.14,3.14};
      Serial.write((char*)j, num_joints*sizeof(float));
    }
  }

}
