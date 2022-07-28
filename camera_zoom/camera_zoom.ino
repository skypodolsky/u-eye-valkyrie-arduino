#define PIN 7
#define ADDRESS 1
#define DECISION_THRESHOLD 10

#define CAMERA_ZOOM_IN 0x20
#define CAMERA_ZOOM_OUT 0x40
#define CAMERA_SET_ZOOM 0x4F

void sendPelcoDFrame(byte command, byte data1, byte data2)
{
  byte bytes[7] = {0xFF, ADDRESS, 0x00, command, data1, data2, 0x00};
  byte crc = (bytes[1] + bytes[2] + bytes[3] + bytes[4] + bytes[5]) % 0x100;
  bytes[6] = crc;

  for (int i = 0; i < 7; i++)
    Serial.write(bytes[i]);
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN, INPUT);
}

void loop() {
  static uint16_t zoom_position = 1;
  static int prev_duration = 0;
  int curr_duration = pulseIn(PIN, HIGH);
  int delta;
  
  if (prev_duration == 0) {
    prev_duration = curr_duration;
    return;
  }

  delta = curr_duration - prev_duration;
  
  if (delta > DECISION_THRESHOLD) {
    sendPelcoDFrame(CAMERA_ZOOM_IN, 0, 0);
    //zoom_position += 3000;
    //sendPelcoDFrame(CAMERA_SET_ZOOM, zoom_position >> 8, zoom_position & 0xFF);
  }

  if (delta < -DECISION_THRESHOLD) {
    sendPelcoDFrame(CAMERA_ZOOM_OUT, 0, 0);
    //zoom_position -= 3000;
    //sendPelcoDFrame(CAMERA_SET_ZOOM, zoom_position >> 8, zoom_position & 0xFF);
  }

  prev_duration = curr_duration;
  delay(1000);
}
