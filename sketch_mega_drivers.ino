// ==========================================================================
// Maxibot Mega — моторы ног (DRV8833) + 4 серво MG996R (бёдра и колени)
// Команды приходят от ESP8266 #2 (Face) по Serial3, а также с USB (115200).
//
// Протокол (пакет в угловых скобках, перевод строки НЕ нужен):
//   <D,l,r>            езда, l и r от -255 до 255
//   <S>                стоп моторов
//   <J,hl,hr,kl,kr>    углы серво 0..180: бедро Л, бедро П, колено Л, колено П
//                      (-1 = этот сустав не трогать)
//   <H>                суставы в исходную позу (все 90°)
//   <P>                пинг, ответ MEGA_OK
//
// Защита: если 500 мс нет новой <D,...>, моторы останавливаются.
// Серво не останавливаются — держат последний угол.
// ==========================================================================
#include <Servo.h>

// ---- DRV8833 (как в вашем скетче) ----
#define AIN1 5    // левый мотор (ШИМ)
#define AIN2 6    // левый мотор (ШИМ)
#define BIN1 9    // правый мотор (ШИМ)
#define BIN2 10   // правый мотор (ШИМ)
#define STBY 4    // EEP/SLEEP/STBY драйвера: HIGH = включён

// ---- Серво MG996R ----
#define SERVO_HIP_L   22
#define SERVO_HIP_R   23
#define SERVO_KNEE_L  24
#define SERVO_KNEE_R  25

#define ESP_BAUD      38400   // = MEGA_BAUD в motors.h на ESP
#define FAILSAFE_MS   500
#define SERVO_STEP    3       // плавность серво: градусов за шаг
#define SERVO_TICK_MS 15      // ... каждые 15 мс (≈200°/с)

const uint8_t servoPins[4] = {SERVO_HIP_L, SERVO_HIP_R, SERVO_KNEE_L, SERVO_KNEE_R};
Servo servos[4];
int servoCur[4]    = {90, 90, 90, 90};
int servoTarget[4] = {90, 90, 90, 90};

uint32_t lastDriveMs = 0;
bool motorsRunning = false;

// ---------------------------------------------------------------- моторы
// DRV8833: вперёд IN1 = ШИМ, IN2 = LOW; назад наоборот; стоп — оба LOW
void setMotorChannel(int pwm, int in1, int in2) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { analogWrite(in1, pwm);  digitalWrite(in2, LOW); }
  else if (pwm < 0) { digitalWrite(in1, LOW); analogWrite(in2, -pwm); }
  else              { digitalWrite(in1, LOW); digitalWrite(in2, LOW); }
}

void driveMotors(int left, int right) {
  setMotorChannel(left,  AIN1, AIN2);
  setMotorChannel(right, BIN1, BIN2);
  motorsRunning = (left != 0 || right != 0);
}

void stopMotors() {
  driveMotors(0, 0);
}

// ----------------------------------------------------------------- серво
void setServoTarget(int i, int angle) {
  if (angle < 0) return;                 // -1 = не трогать
  servoTarget[i] = constrain(angle, 0, 180);
}

void updateServos() {
  static uint32_t t = 0;
  if (millis() - t < SERVO_TICK_MS) return;
  t = millis();
  for (int i = 0; i < 4; i++) {
    int d = servoTarget[i] - servoCur[i];
    if (d == 0) continue;
    servoCur[i] += constrain(d, -SERVO_STEP, SERVO_STEP);
    servos[i].write(servoCur[i]);
  }
}

// ------------------------------------------------------------- протокол
void handleCommand(char *cmd, Stream &reply) {
  int a, b, c, d;
  switch (cmd[0]) {
    case 'D':
      if (sscanf(cmd, "D,%d,%d", &a, &b) == 2) {
        driveMotors(a, b);
        lastDriveMs = millis();
        reply.println(F("MEGA_OK"));
      }
      break;
    case 'S':
      stopMotors();
      reply.println(F("MEGA_OK"));
      break;
    case 'J':
      if (sscanf(cmd, "J,%d,%d,%d,%d", &a, &b, &c, &d) == 4) {
        setServoTarget(0, a); setServoTarget(1, b);
        setServoTarget(2, c); setServoTarget(3, d);
        reply.println(F("MEGA_OK"));
      }
      break;
    case 'H':
      for (int i = 0; i < 4; i++) servoTarget[i] = 90;
      reply.println(F("MEGA_OK"));
      break;
    case 'P':
      reply.println(F("MEGA_OK"));
      break;
  }
}

// Собирает пакет между '<' и '>' побайтно — не ждёт '\n' и не блокирует loop
struct Parser { char buf[40]; uint8_t len = 0; bool in = false; };
Parser espParser, usbParser;

void readPort(Stream &port, Parser &p) {
  while (port.available()) {
    char ch = port.read();
    if (ch == '<')               { p.in = true; p.len = 0; }
    else if (ch == '>' && p.in)  { p.buf[p.len] = 0; p.in = false; handleCommand(p.buf, port); }
    else if (p.in) {
      if (p.len < sizeof(p.buf) - 1) p.buf[p.len++] = ch;
      else p.in = false;         // слишком длинный пакет — выбросить
    }
  }
}

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  Serial3.begin(ESP_BAUD);   // RX3 = пин 15 (от ESP), TX3 = пин 14 (к ESP)

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();

  for (int i = 0; i < 4; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(servoCur[i]);
  }

  Serial.println(F("Maxibot Mega ready: DRV8833 + 4x MG996R"));
  Serial.println(F("Commands: <D,l,r> <S> <J,hl,hr,kl,kr> <H> <P>"));
}

void loop() {
  readPort(Serial3, espParser);
  readPort(Serial,  usbParser);

  if (motorsRunning && millis() - lastDriveMs > FAILSAFE_MS) {
    stopMotors();
    Serial.println(F("Failsafe: нет команд от ESP — стоп"));
  }

  updateServos();
}
