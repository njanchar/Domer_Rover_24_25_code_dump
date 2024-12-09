
uint8_t read_buf[4];

uint8_t test_buf2[4];
uint8_t test_buf3[20];
uint8_t write_me[4];
float test_num = 30.5;
float test_num2 = 30.0;
float test_num3 = 50.00;
float test_num4 = 6500.45;
float test_num5 = 12345.67;
int new_num = 300;
uint8_t test_buf[4];
uint8_t test_buf4[4];
uint8_t test_buf5[4];
uint8_t test_buf6[4];

void setup() {
  // put your setup code here, to run once:
  delay(500);
  Serial.begin(9600);
  memcpy(&(test_buf[0]), &(test_num), sizeof(test_buf));
  memcpy(&(test_buf2[0]), &(test_num2), sizeof(test_buf2));
  memcpy(&(test_buf4[0]), &(test_num3), sizeof(test_buf4));
  memcpy(&(test_buf5[0]), &(test_num4), sizeof(test_buf5));
  memcpy(&(test_buf6[0]), &(test_num5), sizeof(test_buf6));
  for(int i=0;i<sizeof(test_buf);i++){
    test_buf3[i] = test_buf[i];
  }
  for(int i=0;i<sizeof(test_buf2);i++){
    test_buf3[i+4] = test_buf2[i];
  }
  for(int i=0;i<sizeof(test_buf4);i++){
    test_buf3[i+8] = test_buf4[i];
  }
  for(int i=0;i<sizeof(test_buf5);i++){
    test_buf3[i+12] = test_buf5[i];
  }
   for(int i=0;i<sizeof(test_buf6);i++){
    test_buf3[i+16] = test_buf6[i];
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
   // uint8_t incomingByte = Serial.read();
    Serial.readBytes(read_buf, 4);
    for(int i=0;i<sizeof(read_buf);i++){
    test_buf3[i+16] = read_buf[i];
    }
    delay(50);
    Serial.write(test_buf3,sizeof(test_buf3));
   // Serial.write(test_buf3, sizeof(test_buf3));
  }
 // delay(300);
  //Serial.print('A');
}
