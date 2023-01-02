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
#define BUTTON_LOCK_FIRST             0x10
#define BUTTON_LOCK_SECOND            0x20
#define BUTTON_ENGAGE_FIRST           0x12
#define BUTTON_ENGAGE_SECOND          0x22

#define SERVO_LOCK_VAL                90
#define SERVO_UNLOCK_VAL              0
#define SERVO_FIRST_PIN               5
#define SERVO_SECOND_PIN              6
#define SENSOR_FIRST_PIN              7
#define SENSOR_SECOND_PIN             8
#define BUZZER_PIN                    2

#define ZOOM_MIN                      1
#define ZOOM_MAX                      10
#define ZOOM_UPDATE_TIMEOUT           700

#define CAMERA_ZOOM_MIN_VALUE         0x3BF
#define CAMERA_ZOOM_MAX_VALUE         0x2E3F
#define CAMERA_SERIAL_RX_PIN          2
#define CAMERA_SERIAL_TX_PIN          3

typedef enum buzzer_status_t {
  BUZZER_OK,
  BUZZER_LOCK_FAILED,
  BUZZER_FIRST_LOCKED,
  BUZZER_SECOND_LOCKED
} buzzer_status_t;

#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))

uint8_t zoom = ZOOM_MIN;
unsigned long zoom_prev_update;

Servo servo_first;
Servo servo_second;
bool servo_first_locked;
bool servo_second_locked;

SoftwareSerial camera_serial(CAMERA_SERIAL_RX_PIN, CAMERA_SERIAL_TX_PIN);

void setup() {
  //delay to hear buzzers on the backgroung of everything else
  delay(1000);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  buzzerNotification(BUZZER_OK);
  
  servo_first.attach(SERVO_FIRST_PIN);
  servo_second.attach(SERVO_SECOND_PIN);

  servo_first.write(SERVO_UNLOCK_VAL);
  servo_second.write(SERVO_UNLOCK_VAL);

  Serial.begin(57600);
  Serial.setTimeout(200);

  camera_serial.begin(9600);
  sendViscaZoomFrame(ZOOM_MIN);
}

static void buzzerNotification(buzzer_status_t code)
{
  switch (code) {
    case BUZZER_OK:
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);
    break;
    case BUZZER_FIRST_LOCKED:
      for (int i = 0; i < 1; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(50);
        digitalWrite(BUZZER_PIN, LOW);
        delay(50);
      }
    break;
    case BUZZER_SECOND_LOCKED:
      for (int i = 0; i < 2; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(50);
        digitalWrite(BUZZER_PIN, LOW);
        delay(50);
      }
    break;
    case BUZZER_LOCK_FAILED:
      for (int i = 0; i < 5; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(50);
        digitalWrite(BUZZER_PIN, LOW);
        delay(50);
      }
    break;
  }
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

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
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
        int i = 0;
        while (digitalRead(SENSOR_FIRST_PIN) != HIGH) {
          servo_first.write(SERVO_UNLOCK_VAL + i);
          delay(70);
          i += 2;

          if (i > (SERVO_LOCK_VAL - SERVO_UNLOCK_VAL)) {
            Serial.println("Failed to lock servo 1");
            servo_first.write(SERVO_UNLOCK_VAL);
            buzzerNotification(BUZZER_LOCK_FAILED);
            goto out;
          }
        }

        servo_first.write(SERVO_UNLOCK_VAL + i + 3);

        Serial.println("Servo 1 locked");
        buzzerNotification(BUZZER_FIRST_LOCKED);
        servo_first_locked = true;
      }
      break;
    case BUTTON_LOCK_SECOND:
      if (!servo_second_locked) {
        int i = 0;
        while (digitalRead(SENSOR_SECOND_PIN) != HIGH) {
          servo_second.write(SERVO_UNLOCK_VAL + i);
          delay(70);
          i += 2;

          if (i > (SERVO_LOCK_VAL - SERVO_UNLOCK_VAL)) {
            Serial.println("Failed to lock servo 2");
            servo_second.write(SERVO_UNLOCK_VAL);
            buzzerNotification(BUZZER_LOCK_FAILED);
            goto out;
          }
        }
        
        servo_second.write(SERVO_UNLOCK_VAL + i + 3);
        
        Serial.println("Servo 2 locked");
        buzzerNotification(BUZZER_SECOND_LOCKED);
        servo_second_locked = true;
      }
      break;
    case BUTTON_ENGAGE_FIRST:
      servo_first.write(SERVO_UNLOCK_VAL);
      delay(1000);
      Serial.println("Servo 1 unlocked");
      servo_first_locked = false;
      break;
    case BUTTON_ENGAGE_SECOND:
      servo_second.write(SERVO_UNLOCK_VAL);
      delay(1000);
      Serial.println("Servo 2 unlocked");
      servo_second_locked = false;
      break;
  }

  return;
  
out:
  serialFlush();
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

      Serial.println(msg_id, HEX);
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
