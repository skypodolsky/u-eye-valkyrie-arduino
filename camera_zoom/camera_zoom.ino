// connection-related
#define PIN 7 // D7 pin to connect to the PX4
#define ADDRESS 1 // CAMERA ADDRESS

// you can touch this
#define CAMERA_ZOOM_MIN 0  // min zoom value
#define CAMERA_ZOOM_MAX 7 // max zoom value
#define CAMERA_ZOOM_POSITIONS CAMERA_ZOOM_MAX // only integer zoom values (10 positions for x10 camera)
#define CAMERA_PWM_US_MIN 1050 // absolute min, ignore outside this window
#define CAMERA_PWM_US_MAX 2050 // absolute max, ignore outside this window
#define CAMERA_PWM_US_EFF_MIN 1050 // can leave the same
#define CAMERA_PWM_US_EFF_MAX 2000 // needs to be lesser to allow x10 zoom

// do not touch this unless you like debugging
#define CAMERA_PWM_RANGE (CAMERA_PWM_US_EFF_MAX - CAMERA_PWM_US_EFF_MIN)
#define CAMERA_USECS_PER_ZOOM_POSITION (CAMERA_PWM_RANGE / CAMERA_ZOOM_POSITIONS)

#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))

void sendPelcoDZoomFrame(byte command, byte data1, byte data2)
{
  byte bytes[7] = {0xFF, ADDRESS, 0x00, command, data1, data2, 0x00};
  byte crc = (bytes[1] + bytes[2] + bytes[3] + bytes[4] + bytes[5]) % 0x100;
  bytes[6] = crc;

  for (int i = 0; i < ARRAY_LENGTH(bytes); i++)
    Serial.write(bytes[i]);
}

void sendViscaZoomFrame(int zoom)
{
  /** 100% max */
  if (zoom > CAMERA_ZOOM_MAX)
    zoom = CAMERA_ZOOM_MAX;

  if (zoom < CAMERA_ZOOM_MIN)
    zoom = CAMERA_ZOOM_MIN;

  const int zoom_min = 0x3BF;
  const int zoom_max = 0x2E3F;
  const int zoom_range = zoom_max - zoom_min;
  int zoom_value = zoom_min + ((double) zoom * (double) zoom_range) / (double) CAMERA_ZOOM_MAX;

  //Serial.print(zoom_value, HEX);
  //Serial.print('\n');
  
  byte cmd[] = { 0x81, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00, 0xFF };

  for (int i = 7; i > 3; i--) {
    cmd[i] = zoom_value & 0x0F;
    zoom_value = zoom_value >> 4;
  }

#if 0
  for (int i = 0; i < ARRAY_LENGTH(cmd); i++) {
    //Serial.print(cmd[i], HEX);
    //Serial.print(' ');
  }
#endif

  for (int i = 0; i < ARRAY_LENGTH(cmd); i++)
    Serial.write(cmd[i]);
}

void sendViscaFocusFar()
{
  byte cmd[] = { 0x81, 0x01, 0x04, 0x08, 0x03, 0xFF };
 
  for (int i = 0; i < ARRAY_LENGTH(cmd); i++)
    Serial.write(cmd[i]);
}

void sendViscaZoomDisplayOnFrame()
{
  byte cmd[] = { 0x81, 0x01, 0x04, 0x00, 0x02, 0xFF };
 
  for (int i = 0; i < ARRAY_LENGTH(cmd); i++)
    Serial.write(cmd[i]);
}

void sendViscaClearFrame()
{
  byte cmd[] = { 0x88, 0x01, 0x00, 0x01, 0xFF };
 
  for (int i = 0; i < ARRAY_LENGTH(cmd); i++)
    Serial.write(cmd[i]);
}

void sendViscaAddressFrame()
{
  byte cmd[] = { 0x88, 0x30, 0x01, 0xFF };
 
  for (int i = 0; i < ARRAY_LENGTH(cmd); i++)
    Serial.write(cmd[i]);
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN, INPUT);

  sendViscaAddressFrame();
  delay(500);
  sendViscaClearFrame();
  delay(500);
}

void loop() {
  int pwm_duration = pulseIn(PIN, HIGH);

  // we handle an overflow case
  if (pwm_duration > CAMERA_PWM_US_MAX) {
    return;
  }

  if (pwm_duration < CAMERA_PWM_US_MIN) {
    return;
  }

  int diff = pwm_duration - CAMERA_PWM_US_EFF_MIN;
  int zoom = ((double) diff * (double) CAMERA_ZOOM_MAX) / CAMERA_PWM_RANGE;

#if 0
  Serial.println("---");
  Serial.print("duration: ");
  Serial.println(pwm_duration);
  Serial.print("zoom: ");
  Serial.println(zoom);
#endif

  static int prev_zoom = 0;

  if (!prev_zoom)
    prev_zoom = zoom;

  if (zoom != prev_zoom)
    sendViscaZoomFrame(zoom);

  prev_zoom = zoom;
  delay(20);
}
