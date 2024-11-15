//left ->  A
int motorA_pin1 = 4;  
int motorA_pin2 = 16;

//right -> B
int motorB_pin1 = 18;
int motorB_pin2 = 17;


int delayTime = 5000;

void setup() {

  pinMode(motorA_pin1, OUTPUT);
  pinMode(motorA_pin2, OUTPUT);
  pinMode(motorB_pin1, OUTPUT);
  pinMode(motorB_pin2, OUTPUT);


  Serial.begin(115200);
}

void loop() {
  // Motor A Forward
  Serial.println("Motor A Forward");
  digitalWrite(motorA_pin1, HIGH);
  digitalWrite(motorA_pin2, LOW);
  delay(delayTime);

  // Motor A Reverse
  Serial.println("Motor A Reverse");
  digitalWrite(motorA_pin1, LOW);
  digitalWrite(motorA_pin2, HIGH);
  delay(delayTime);

  // Stop Motor A
  Serial.println("Motor A Stop");
  digitalWrite(motorA_pin1, LOW);
  digitalWrite(motorA_pin2, LOW);
  delay(delayTime);

  // Motor B Forward
  Serial.println("Motor B Forward");
  digitalWrite(motorB_pin1, HIGH);
  digitalWrite(motorB_pin2, LOW);
  delay(delayTime);

  // Motor B Reverse
  Serial.println("Motor B Reverse");
  digitalWrite(motorB_pin1, LOW);
  digitalWrite(motorB_pin2, HIGH);
  delay(delayTime);

  // Stop Motor B
  Serial.println("Motor B Stop");
  digitalWrite(motorB_pin1, LOW);
  digitalWrite(motorB_pin2, LOW);
  delay(delayTime);
}
