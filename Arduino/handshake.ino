int ACK = 0;
int NAK = 1;
int HELLO = 2;
int reply;
int handshake_flag;

void handshake() {
  while (handshake_flag == 1) {
    if (Serial.available()) {
      reply = Serial.read();
      if (reply == HELLO) {
        Serial.println(ACK);
      }
      if (reply == ACK) {
        handshake_flag = 0;
      }
    }
  }
}

void setup() {
Serial.begin(9600);
handshake_flag = 1;
}
  
void loop() {
  handshake();
}
