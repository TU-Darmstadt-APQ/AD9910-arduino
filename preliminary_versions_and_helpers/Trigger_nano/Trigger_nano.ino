

void setup() {
  // put your setup code here, to run once:
  DDRD = DDRD | B11111100;  // this is safer as it sets pins 2 to 7 as outputs
                    // without changing the value of pins 0 & 1, which are RX & TX
}

void loop() {
  // put your main code here, to run repeatedly:
  PORTD = B00000100;
  delayMicroseconds(2);
  PORTD = B00000000;
  delayMicroseconds(2);
  PORTD = B00000100;
  delayMicroseconds(2);
  PORTD = B00000000;
  
  delay(10);
}
