#include <Servo.h>

Servo myservo;
int pos = 90;

static const int SAFEGUARD_POSITION = 10;
static const int SENSITIVITY = 100;
int leftSensor = A0;
int rightSensor = A1;
int sensorValue;

typedef enum {
    NORMAL = 0,
    MANUAL = 1,
    SAFEGUARD = 2,
} status_t;

typedef enum {
  MOVE_LEFT = 0,
  MOVE_RIGHT = 1,
  GOTO_SAFEGUARD = 2,
  NORMAL_OPERATION = 3,
} incoming_t;

typedef enum {
  GOING_NORMAL = 0,
  GOING_MANUAL = 1,
  GOING_SAFEGUARD = 2,
} outgoing_t;

status_t status = NORMAL;

void sendMode(outgoing_t o) {
  if (o == GOING_NORMAL) {
    Serial.println("mode: normal");
  } else if (o == GOING_MANUAL) {
    Serial.println("mode: manual");
  } else if (o == GOING_SAFEGUARD) {
    Serial.println("mode: safeguard");
  }
}

void sendPosition(int p) {
  Serial.print("pos: ");
  Serial.print(p, DEC);
  Serial.println();
}

void adjustPosition(incoming_t i) {
  if (i == MOVE_LEFT) {
    pos += 5;
    if (pos > 170) {
      pos = 170;
    }
  } else {
    pos -= 5;
    if (pos < 10) {
      pos = 10;
    }
  }

  sendPosition(pos);
}

int getLightDiff() {
  int leftRead = analogRead(leftSensor);
  int rightRead = analogRead(rightSensor);

  return leftRead - rightRead;
}

void handleServo() {
    if (status == NORMAL) {
      int diff = getLightDiff();
      if (diff <= SENSITIVITY && diff >= -SENSITIVITY) {
        return;
      }

      adjustPosition(diff < 0 ? MOVE_LEFT : MOVE_RIGHT);
      myservo.write(pos);
    } else if (status == MANUAL) {
      myservo.write(pos);
    } else if (status == SAFEGUARD) {
      myservo.write(SAFEGUARD_POSITION);
    } else {
      Serial.println("Something's wrong, I can feel it");
    }
}

void handleRos() {
  while (Serial.available() > 0) {
    incoming_t incoming = Serial.read();
    if (incoming == MOVE_LEFT || incoming == MOVE_RIGHT) {
      if (status == SAFEGUARD) {
        // Ignoramos operación manual en safeguard
        return;
      }

      if (status != MANUAL) {
        status = MANUAL;
        sendMode(GOING_MANUAL);
      }
      adjustPosition(incoming);
    } else if (incoming == NORMAL_OPERATION && status != NORMAL) {
      status = NORMAL;
      sendMode(GOING_NORMAL);

      // Devolvemos el servo a la posición original en
      // caso de haber estado en safeguard
      myservo.write(pos);
    } else if (incoming == GOTO_SAFEGUARD && status != SAFEGUARD) {
      status = SAFEGUARD;
      sendMode(GOING_SAFEGUARD);
      sendPosition(SAFEGUARD_POSITION);
    }
  }
}

void setup()
{
  Serial.begin(115200);

  myservo.attach(9);
  myservo.write(pos);

  Serial.println("Arduino is ready");
}

void loop()
{
  delay(200);
  handleRos();
  handleServo();
}
