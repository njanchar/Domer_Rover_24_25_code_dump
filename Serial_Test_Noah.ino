
uint8_t read_buf[2];
uint8_t test_buf[4];
uint8_t test_buf2[4];
int test_num = 305;
int test_num2 = 455;

void setup() {
  // put your setup code here, to run once:
  delay(500);
  Serial.begin(9600);
  memcpy(&(test_buf[0]), &(test_num), sizeof(test_buf));
  memcpy(&(test_buf2[0]), &(test_num2), sizeof(test_buf2));
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
    uint8_t incomingByte = Serial.read();
    Serial.readBytes(read_buf, 2);
    delay(50);
    Serial.write(test_buf,sizeof(test_buf));
    Serial.write(test_buf2,sizeof(test_buf2));
  }
 // delay(300);
  //Serial.print('A');
}
