#include <Servo.h>
#include <SoftwareSerial.h>

#define MAVLINK_MSG_MAGIC_V1          0xFE
#define MAVLINK_MSG_MAGIC_V2          0xFD
#define MAVLINK_MSG_MANUAL_CONTROL    0x45
#define MAVLINK_MSG_MAX_LEN           280

#define BUTTON_ZOOM_IN_1              0x08
#define BUTTON_ZOOM_OUT_1             0x04
#define BUTTON_ZOOM_IN_2              0x80
#define BUTTON_ZOOM_OUT_2             0x40
#define BUTTON_LOCK_FIRST             0x20
#define BUTTON_LOCK_SECOND            0x10
#define BUTTON_ENGAGE_FIRST           0x22
#define BUTTON_ENGAGE_SECOND          0x12

#define SERVO_LOCK_VAL                180
#define SERVO_UNLOCK_VAL              0
#define SERVO_FIRST_PIN               5
#define SERVO_SECOND_PIN              6

#define ZOOM_MIN                      1
#define ZOOM_MAX                      10
#define ZOOM_UPDATE_TIMEOUT           700

#define CAMERA_ZOOM_MIN_VALUE         0x3BF
#define CAMERA_ZOOM_MAX_VALUE         0x2E3F
#define CAMERA_SERIAL_RX_PIN          2
#define CAMERA_SERIAL_TX_PIN          3

#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))

uint8_t zoom = ZOOM_MIN;
unsigned long zoom_prev_update;

Servo servo_first;
Servo servo_second;
bool servo_first_locked;
bool servo_second_locked;

SoftwareSerial camera_serial(CAMERA_SERIAL_RX_PIN, CAMERA_SERIAL_TX_PIN);

void setup() {
  servo_first.attach(SERVO_FIRST_PIN);
  servo_second.attach(SERVO_SECOND_PIN);

  servo_first.write(SERVO_UNLOCK_VAL);
  servo_second.write(SERVO_UNLOCK_VAL);

  Serial.begin(57600);
  Serial.setTimeout(200);

  camera_serial.begin(9600);
  sendViscaZoomFrame(ZOOM_MIN);
}

static void sendViscaZoomFrame(int zoom)
{
  byte cmd[] = { 0x81, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00, 0xFF };
  int zoom_value = map(zoom, 1, 10, CAMERA_ZOOM_MIN_VALUE, CAMERA_ZOOM_MAX_VALUE); 

  for (int i = 7; i > 3; i--) {
    cmd[i] = zoom_value & 0x0F;
    zoom_value = zoom_value >> 4;
  }

  for (int i = 0; i < ARRAY_LENGTH(cmd); i++)
    camera_serial.write(cmd[i]);
}

static void handle_btn_press(uint16_t bitmap)
{
  switch (bitmap) {
    case BUTTON_ZOOM_IN_1:
    case BUTTON_ZOOM_IN_2:
      if ((millis() - zoom_prev_update) > ZOOM_UPDATE_TIMEOUT) {
        if (zoom < ZOOM_MAX) {
          zoom++;
          sendViscaZoomFrame(zoom);
        }
          
        Serial.print("Zoom = ");
        Serial.println(zoom);
        zoom_prev_update = millis();
      }
      break;
    case BUTTON_ZOOM_OUT_1:
    case BUTTON_ZOOM_OUT_2:
      if ((millis() - zoom_prev_update) > ZOOM_UPDATE_TIMEOUT) {
        if (zoom > ZOOM_MIN) {
          zoom--;
          sendViscaZoomFrame(zoom);
        }
          
        Serial.print("Zoom = ");
        Serial.println(zoom);
        zoom_prev_update = millis();
      }
      break;
    case BUTTON_LOCK_FIRST:
      if (!servo_first_locked) {
        servo_first.write(SERVO_LOCK_VAL);
        Serial.println("Servo 1 locked");
        servo_first_locked = true;
      }
      break;
    case BUTTON_LOCK_SECOND:
      if (!servo_second_locked) {
        servo_second.write(SERVO_LOCK_VAL);
        Serial.println("Servo 2 locked");
        servo_second_locked = true;
      }
      break;
    case BUTTON_ENGAGE_FIRST:
      servo_first.write(SERVO_UNLOCK_VAL);
      Serial.println("Servo 1 unlocked");
      servo_first_locked = false;
      break;
    case BUTTON_ENGAGE_SECOND:
      servo_second.write(SERVO_UNLOCK_VAL);
      Serial.println("Servo 2 unlocked");
      servo_second_locked = false;
      break;
  }
}

static void comm_receive() {
  char buf[MAVLINK_MSG_MAX_LEN] = { 0 };

  while (Serial.available() > 2) { // Magic + length
    bool msg_type_mv2 = false;
    uint8_t hdr_len;
    uint8_t magic = Serial.read();

    // Serial.println(magic, HEX);
    switch (magic) {
      case MAVLINK_MSG_MAGIC_V1:
        hdr_len = 6;
        msg_type_mv2 = false;
        break;
      case MAVLINK_MSG_MAGIC_V2:
        hdr_len = 10;
        msg_type_mv2 = true;
        break;
      default:
        continue;
    }

    uint8_t payload_length = Serial.read();
    uint16_t total_length = payload_length + hdr_len;
    if (Serial.readBytes(buf, total_length) == total_length) {
      uint8_t *data = NULL;
      uint32_t msg_id = 0xFF;

      if (msg_type_mv2) {
        data = &buf[8];
        msg_id = (buf[7] << 16) | (buf[6] << 8) | (buf[5]);
      } else {
        data = &buf[4];
        msg_id = buf[3];
      }

      if (msg_id == MAVLINK_MSG_MANUAL_CONTROL) {
        uint16_t btn_bits = data[9] | data[8];
        handle_btn_press(btn_bits);
        // Serial.println(btn_bits, HEX);
      }
    }
  }
}

void loop() {
  comm_receive();
}
