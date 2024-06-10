#include <Servo.h>

Servo myservo;
int pos = 90;

static const int RIGHT_BUTTON = 2;
static const int LEFT_BUTTON = 3;
static const int NORMAL_BUTTON = 4;
static const int SAFEGUARD_BUTTON = 5;

static const int SAFEGUARD_POSITION = 10;
static const int SENSITIVITY = 100;
int leftSensor = A0;
int rightSensor = A1;
int sensorValue;

typedef enum {
  NORMAL = 0,    // controlado por los sensores
  MANUAL = 1,    // controlado manualmente
  SAFEGUARD = 2, // modo salvaguarda
  LOW_LIGHT = 3, // como el salvaguarda, pero atiende sensores
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
  GOING_LOW_LIGHT = 3,
} outgoing_t;

status_t status = NORMAL;

void sendMode(outgoing_t o) {
  if (o == GOING_NORMAL) {
    Serial.println("mode: normal");
  } else if (o == GOING_MANUAL) {
    Serial.println("mode: manual");
  } else if (o == GOING_SAFEGUARD) {
    Serial.println("mode: safeguard");
  } else if (o == GOING_LOW_LIGHT) {
    Serial.println("mode: lowlight");
  }
}

void sendPosition(int p) {
  Serial.print("pos: ");
  Serial.print(p, DEC);
  Serial.println();
}

void adjustPositionSensors(int leftRead, int rightRead) {
  int diff = leftRead - rightRead;
  if (diff <= SENSITIVITY && diff >= -SENSITIVITY) {
    return;
  }

  adjustPosition(diff > 0 ? MOVE_LEFT : MOVE_RIGHT);
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

void handleSensors() {
  if (status != NORMAL && status != LOW_LIGHT) {
    return;
  }

  int leftRead = analogRead(leftSensor);
  int rightRead = analogRead(rightSensor);
  if (status == NORMAL) {
    if (leftRead < 250 && rightRead < 250) {
      setLowLightMode();
      return;
    }

    adjustPositionSensors(leftRead, rightRead);
  } else if (status == LOW_LIGHT) {
    if (leftRead >= 250 || rightRead >= 250) {
      setNormalMode();
      return;
    }
  }
}

void handleServo() {
    if (status == NORMAL || status == MANUAL || status == LOW_LIGHT) {
      myservo.write(pos);
    } else if (status == SAFEGUARD) {
      myservo.write(SAFEGUARD_POSITION);
    } else {
      Serial.println("Something's wrong, I can feel it");
    }
}

void manualMove(incoming_t incoming) {
  if (status == SAFEGUARD) {
    // Ignoramos operación manual en safeguard
    return;
  }

  if (status != MANUAL) {
    status = MANUAL;
    sendMode(GOING_MANUAL);
  }
  adjustPosition(incoming);
}

void setNormalMode() {
  status = NORMAL;
  sendMode(GOING_NORMAL);
  sendPosition(pos);
}

void setSafeguardMode() {
  status = SAFEGUARD;
  sendMode(GOING_SAFEGUARD);
  sendPosition(SAFEGUARD_POSITION);
}

void setLowLightMode() {
  status = LOW_LIGHT;
  sendMode(GOING_LOW_LIGHT);
  pos = SAFEGUARD_POSITION;
  sendPosition(pos);
}

void handleRos() {
  while (Serial.available() > 0) {
    incoming_t incoming = Serial.read();
    if (incoming == MOVE_LEFT || incoming == MOVE_RIGHT) {
      manualMove(incoming);
    } else if (incoming == NORMAL_OPERATION && status != NORMAL) {
      setNormalMode();
    } else if (incoming == GOTO_SAFEGUARD && status != SAFEGUARD) {
      setSafeguardMode();
    }
  }
}

void handleButtons() {
  if (digitalRead(SAFEGUARD_BUTTON) == LOW) {
    // Botón salvaguarda tiene prioridad máxima y anula el resto
    // de botones.
    if (status != SAFEGUARD) {
      setSafeguardMode();
    }
  } else if (digitalRead(NORMAL_BUTTON) == LOW && status != NORMAL) {
    setNormalMode();
  } else if (digitalRead(LEFT_BUTTON) == LOW) {
    manualMove(MOVE_LEFT);
  } else if (digitalRead(RIGHT_BUTTON) == LOW) {
    manualMove(MOVE_RIGHT);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(RIGHT_BUTTON, INPUT);
  pinMode(LEFT_BUTTON, INPUT);
  pinMode(NORMAL_BUTTON, INPUT);
  pinMode(SAFEGUARD_BUTTON, INPUT);

  myservo.attach(9);
  myservo.write(pos);

  Serial.println("Arduino is ready");
  sendMode(GOING_NORMAL);
  sendPosition(pos);
}

void loop() {
  delay(200);
  handleRos();
  handleButtons();
  handleSensors();
  handleServo();
}
