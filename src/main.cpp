#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Servo.h>
#include <math.h>

#define SENSOR A4
#define buttonPin1 38
#define buttonPin2 37
#define buttonPin3 36
#define buttonPin4 35
#define led 32

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int selectedProgram = 1;

#define STEP_PIN_X1 11
#define DIR_PIN_X1 9
#define STEP_PIN_X2 15
#define DIR_PIN_X2 13
#define STEP_PIN_Y 7
#define DIR_PIN_Y 5

#define TIMER2_MAX_FREQ 10000
#define STEP_PULSE_WIDTH 10

volatile long targetPositionX = 0;
volatile long currentPositionX = 0;
volatile bool stepStateX = false;
long moveStepsX = 0;
bool moveDirectionX = true;
volatile bool isMovingX = false;
volatile unsigned int speedDividerX = 1;
volatile unsigned int stepCounterX = 0;

volatile long targetPositionY = 0;
volatile long currentPositionY = 0;
volatile bool stepStateY = false;
long moveStepsY = 0;
bool moveDirectionY = true;
volatile bool isMovingY = false;
volatile unsigned int speedDividerY = 1;
volatile unsigned int stepCounterY = 0;

volatile bool bresenhamActive = false;
volatile long bresenhamTotalSteps = 0;
volatile long bresenhamErrorX = 0;
volatile long bresenhamErrorY = 0;
volatile long bresenhamStepsX = 0;
volatile long bresenhamStepsY = 0;
volatile unsigned int bresenhamSpeedDivider = 1;
volatile unsigned int bresenhamStepCounter = 0;

Servo servo;

double koef_x = 51.0;
double koef_y = 810.12;

int startX = 0;
int endX = 0;
int startY = 0;
int endY = 0;
bool ex = false;
int sum = 0;
int posX = 0;
int posY = 0;
int curX = 0;
int cntrX = 0;
int cntrY = 0;

double firstX = 0;
double firstY = 0;
double secondX = 0;
double secondY = 0;
float alpha = 0;

void setupTimer2();
void setSpeedX(int stepsPerSecond);
void setSpeedY(int stepsPerSecond);
float getPositionX();
float getPositionY();
void drawRotatedRectangle(float centerX, float centerY, float width, float height, float angleDeg, float speed, bool fill = false, float penWidth = 1.0);
void drawEllipse(int centerX_mm, int centerY_mm, int radiusX_mm, int radiusY_mm, float speed, int segments = 0);
void moveLine(int x1, int y1, int x2, int y2, float speed);
void s_up();
void s_down();
int sensor();
void moveToPoint(float x_mm, float y_mm, float speed);
void drawCircle(int centerX_mm, int centerY_mm, int radius_mm, float speed, int segments = 0);
void program1();
void program2();
void program3();
void program4();
void program5();
void program6();
void program7();
void program8();
void program9();
void program10();
void program11();
int selectProgram();
void drawProgramScreen(int prog);
void updateDisplay();

void setup() {
  Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;)
      ;
  }
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  pinMode(STEP_PIN_X1, OUTPUT);
  pinMode(DIR_PIN_X1, OUTPUT);
  pinMode(STEP_PIN_X2, OUTPUT);
  pinMode(DIR_PIN_X2, OUTPUT);
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(DIR_PIN_Y, OUTPUT);

  digitalWrite(STEP_PIN_X1, LOW);
  digitalWrite(DIR_PIN_X1, LOW);
  digitalWrite(STEP_PIN_X2, LOW);
  digitalWrite(DIR_PIN_X2, LOW);
  digitalWrite(STEP_PIN_Y, LOW);
  digitalWrite(DIR_PIN_Y, LOW);

  servo.attach(3);
  s_up();

  selectedProgram = selectProgram();
  display.clearDisplay();
  updateDisplay();
  setupTimer2();
}

int prog = 1;
int selectProgram() {
  bool programSelected = false;
  unsigned long lastButtonTime = 0;
  const int debounceDelay = 200;

  display.clearDisplay();
  drawProgramScreen(prog);

  while (!programSelected) {
    if (millis() - lastButtonTime > debounceDelay) {
      if (digitalRead(buttonPin4) == LOW) {
        prog--;
        if (prog < 1) prog = 11;
        drawProgramScreen(prog);
        lastButtonTime = millis();
      }
      if (digitalRead(buttonPin1) == LOW) {
        prog++;
        if (prog > 11) prog = 1;
        drawProgramScreen(prog);
        lastButtonTime = millis();
      }
      if (digitalRead(buttonPin2) == LOW) {
        programSelected = true;
      }
    }
    delay(10);
  }
  return prog;
}

void drawProgramScreen(int prog) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 0);
  display.println("SELECT PROGRAM");
  display.drawRect(30, 15, 68, 30, SSD1306_WHITE);
  display.setTextSize(3);
  display.setCursor(50, 20);
  if (prog < 10) display.print("0");
  display.print(prog);
  display.setTextSize(1);
  display.setCursor(3, 50);
  display.print("UP:1");
  display.setCursor(42, 50);
  display.print("DOWN:4");
  display.setCursor(90, 50);
  display.print("OK:2");
  display.display();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Program: ");
  display.println(selectedProgram);
  display.setCursor(0, 16);
  display.print("X: ");
  // display.println(getPositionX(), 1);
  display.println(alpha, 1);
  display.setCursor(0, 32);
  display.print("Y: ");
  display.println(getPositionY(), 1);
  display.setCursor(0, 48);
  if (isMovingX || isMovingY) {
    display.print("Moving...");
  } else {
    display.print("Idle");
  }
  display.display();
}

void setupTimer2() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS21);
  OCR2A = 199;
  TIMSK2 |= (1 << OCIE2A);
}

void setSpeedX(int stepsPerSecond) {
  if (stepsPerSecond > 0 && stepsPerSecond <= TIMER2_MAX_FREQ) {
    unsigned int divider = TIMER2_MAX_FREQ / stepsPerSecond;
    if (divider < 1) divider = 1;
    if (divider > 65535) divider = 65535;
    noInterrupts();
    speedDividerX = divider;
    stepCounterX = 0;
    interrupts();
  }
}

void setSpeedY(int stepsPerSecond) {
  if (stepsPerSecond > 0 && stepsPerSecond <= TIMER2_MAX_FREQ) {
    unsigned int divider = TIMER2_MAX_FREQ / stepsPerSecond;
    if (divider < 1) divider = 1;
    if (divider > 65535) divider = 65535;
    noInterrupts();
    speedDividerY = divider;
    stepCounterY = 0;
    interrupts();
  }
}

void moveTicksX(long ticks) {
  long steps = ticks * koef_x;
  if (steps == 0) return;
  noInterrupts();
  moveStepsX = labs(steps);
  moveDirectionX = (steps > 0);
  targetPositionX = currentPositionX + (moveDirectionX ? moveStepsX : -moveStepsX);
  isMovingX = true;
  stepCounterX = 0;
  interrupts();
  digitalWrite(DIR_PIN_X1, moveDirectionX ? LOW : HIGH);
  digitalWrite(DIR_PIN_X2, moveDirectionX ? LOW : HIGH);
}

void stopX() {
  noInterrupts();
  targetPositionX = currentPositionX;
  moveStepsX = 0;
  isMovingX = false;
  if (stepStateX) {
    digitalWrite(STEP_PIN_X1, LOW);
    digitalWrite(STEP_PIN_X2, LOW);
    stepStateX = false;
  }
  interrupts();
}

bool isMoveCompleteX() {
  return !isMovingX;
}

float getPositionX() {
  float pos;
  noInterrupts();
  pos = (float)currentPositionX / (float)koef_x;
  interrupts();
  return pos;
}

long getRemainingStepsX() {
  long remaining;
  noInterrupts();
  remaining = labs(targetPositionX - currentPositionX);
  interrupts();
  return remaining;
}

void moveTicksY(long ticks) {
  long steps = ticks * koef_y;
  if (steps == 0) return;
  noInterrupts();
  moveStepsY = labs(steps);
  moveDirectionY = (steps > 0);
  targetPositionY = currentPositionY + (moveDirectionY ? moveStepsY : -moveStepsY);
  isMovingY = true;
  stepCounterY = 0;
  interrupts();
  digitalWrite(DIR_PIN_Y, moveDirectionY ? LOW : HIGH);
}

void stopY() {
  noInterrupts();
  targetPositionY = currentPositionY;
  moveStepsY = 0;
  isMovingY = false;
  if (stepStateY) {
    digitalWrite(STEP_PIN_Y, LOW);
    stepStateY = false;
  }
  interrupts();
}

bool isMoveCompleteY() {
  return !isMovingY;
}

float getPositionY() {
  float pos;
  noInterrupts();
  pos = (float)currentPositionY / (float)koef_y;
  interrupts();
  return pos;
}

long getRemainingStepsY() {
  long remaining;
  noInterrupts();
  remaining = labs(targetPositionY - currentPositionY);
  interrupts();
  return remaining;
}

void moveBoth(int ticksX, int ticksY, float speed, bool flag) {
  long stepsX = 0;
  long stepsY = 0;
  if (flag == 0) {
    stepsX = ticksX * koef_x;
    stepsY = ticksY * koef_y;
  } else {
    stepsX = ticksX;
    stepsY = ticksY;
  }
  if (stepsX == 0 && stepsY == 0) return;
  long absStepsX = labs(stepsX);
  long absStepsY = labs(stepsY);
  long maxSteps = max(absStepsX, absStepsY);
  float actualSpeed = speed;
  if (actualSpeed <= 0) actualSpeed = 1000;
  if (actualSpeed > TIMER2_MAX_FREQ) actualSpeed = TIMER2_MAX_FREQ;
  unsigned int divider = TIMER2_MAX_FREQ / (unsigned int)actualSpeed;
  if (divider < 1) divider = 1;
  if (divider > 65535) divider = 65535;
  long targetX = currentPositionX + stepsX;
  long targetY = currentPositionY + stepsY;

  noInterrupts();
  bresenhamStepsX = absStepsX;
  bresenhamStepsY = absStepsY;
  bresenhamTotalSteps = maxSteps;
  bresenhamErrorX = maxSteps / 2;
  bresenhamErrorY = maxSteps / 2;
  bresenhamSpeedDivider = divider;
  bresenhamStepCounter = 0;
  bresenhamActive = true;
  moveDirectionX = (stepsX > 0);
  moveDirectionY = (stepsY > 0);
  targetPositionX = targetX;
  targetPositionY = targetY;
  isMovingX = true;
  isMovingY = true;
  interrupts();

  digitalWrite(DIR_PIN_X1, moveDirectionX ? LOW : HIGH);
  digitalWrite(DIR_PIN_X2, moveDirectionX ? LOW : HIGH);
  digitalWrite(DIR_PIN_Y, moveDirectionY ? LOW : HIGH);
}

bool isBothMoveComplete() {
  return !isMovingX && !isMovingY;
}

void stopBoth() {
  stopX();
  stopY();
  noInterrupts();
  bresenhamActive = false;
  interrupts();
}

void moveLine(int x1, int y1, int x2, int y2, float speed) {
  int dx = x2 - x1;
  int dy = y2 - y1;
  moveBoth(dx, dy, speed, 0);
}

void drawCircle(int centerX_mm, int centerY_mm, int radius_mm, float speed, int segments) {
  long cx = (long)centerX_mm * (long)koef_x;
  long cy = (long)centerY_mm * (long)koef_y;
  long rx = (long)radius_mm * (long)koef_x;
  long ry = (long)radius_mm * (long)koef_y;

  int actualSegments = segments;
  if (actualSegments == 0) {
    double circumference = 2.0 * M_PI * (double)(rx + ry) / 2.0;
    actualSegments = (int)circumference;
    if (actualSegments < 36) actualSegments = 36;
    if (actualSegments > 2000) actualSegments = 2000;
  }

  s_up();

  long startX = cx + rx;
  long startY = cy;
  long dx_start = startX - currentPositionX;
  long dy_start = startY - currentPositionY;
  if (dx_start != 0 || dy_start != 0) {
    moveBoth(dx_start, dy_start, speed, 1);
    while (!isBothMoveComplete()) {
      updateDisplay();
      delay(1);
    }
  }

  s_down();

  float angleStep = 2.0 * M_PI / (float)actualSegments;
  long prevX = startX;
  long prevY = startY;

  for (int i = 1; i <= actualSegments; i++) {
    float angle = (float)i * angleStep;
    long x = cx + (long)((double)rx * cos(angle));
    long y = cy + (long)((double)ry * sin(angle));

    long dx = x - prevX;
    long dy = y - prevY;

    moveBoth(dx, dy, speed, 1);
    while (!isBothMoveComplete()) {
      delay(1);
    }

    prevX = x;
    prevY = y;
  }

  s_up();
}

int sensor() {
  return constrain(map(analogRead(A4), 110, 860, 100, 0), 0, 100);
}

void moveToPoint(float x_mm, float y_mm, float speed) {
  float dx = x_mm - getPositionX();
  float dy = y_mm - getPositionY();
  if (fabs(dx) < 0.01 && fabs(dy) < 0.01) return;

  int idx = (int)(dx + (dx > 0 ? 0.5 : -0.5));
  int idy = (int)(dy + (dy > 0 ? 0.5 : -0.5));

  moveBoth(idx, idy, speed, 0);

  while (isBothMoveComplete() == false) {
    delay(1);
  }
}

void drawEllipse(int centerX_mm, int centerY_mm, int radiusX_mm, int radiusY_mm, float speed, int segments) {
  long cx = (long)centerX_mm * (long)koef_x;
  long cy = (long)centerY_mm * (long)koef_y;
  long rx = (long)radiusX_mm * (long)koef_x;
  long ry = (long)radiusY_mm * (long)koef_y;

  int actualSegments = segments;
  if (actualSegments == 0) {
    double a = (double)rx;
    double b = (double)ry;
    double h = ((a - b) * (a - b)) / ((a + b) * (a + b));
    double perimeter = M_PI * (a + b) * (1 + (3 * h) / (10 + sqrt(4 - 3 * h)));
    actualSegments = (int)perimeter;
    if (actualSegments > 2000) actualSegments = 2000;
  }

  s_up();

  long startX = cx + rx;
  long startY = cy;
  long dx_start = startX - currentPositionX;
  long dy_start = startY - currentPositionY;
  if (dx_start != 0 || dy_start != 0) {
    moveBoth(dx_start, dy_start, speed, 1);
    while (!isBothMoveComplete()) {
      updateDisplay();
      delay(1);
    }
  }

  s_down();

  double angleStep = 2.0 * M_PI / (double)actualSegments;
  long prevX = startX;
  long prevY = startY;

  for (int i = 1; i <= actualSegments; i++) {
    double angle = (double)i * angleStep;
    long x = cx + (long)((double)rx * cos(angle));
    long y = cy + (long)((double)ry * sin(angle));

    long dx = x - prevX;
    long dy = y - prevY;

    moveBoth(dx, dy, speed, 1);
    while (!isBothMoveComplete()) {
      updateDisplay();
      delay(1);
    }

    prevX = x;
    prevY = y;
  }

  s_up();
}

void drawRotatedRectangle(float centerX, float centerY, float width, float height, float angleDeg, float speed, bool fill, float penWidth) {
  float angleRad = angleDeg * M_PI / 180.0;
  float cosA = cos(angleRad);
  float sinA = sin(angleRad);

  float w2 = width / 2.0;
  float h2 = height / 2.0;

  if (fill) {
    s_up();
    for (float yLocal = -h2; yLocal <= h2; yLocal += penWidth) {
      float xLeftLocal = -w2;
      float xRightLocal = w2;

      float xLeft = centerX + xLeftLocal * cosA - yLocal * sinA;
      float yLeft = centerY + xLeftLocal * sinA + yLocal * cosA;
      float xRight = centerX + xRightLocal * cosA - yLocal * sinA;
      float yRight = centerY + xRightLocal * sinA + yLocal * cosA;

      moveToPoint(xLeft, yLeft, speed);
      s_down();
      moveToPoint(xRight, yRight, speed);
      s_up();
    }
    moveToPoint(centerX, centerY, speed);
  }

  s_up();

  float corners[4][2] = {
    { -w2, -h2 },
    { w2, -h2 },
    { w2, h2 },
    { -w2, h2 }
  };

  float absCorners[4][2];
  for (int i = 0; i < 4; i++) {
    float x = corners[i][0];
    float y = corners[i][1];
    absCorners[i][0] = centerX + x * cosA - y * sinA;
    absCorners[i][1] = centerY + x * sinA + y * cosA;
  }

  moveToPoint(absCorners[0][0], absCorners[0][1], speed);
  s_down();
  for (int i = 1; i <= 4; i++) {
    int idx = i % 4;
    moveToPoint(absCorners[idx][0], absCorners[idx][1], speed);
  }
  s_up();
}

ISR(TIMER2_COMPA_vect) {
  static unsigned long stepStartTimeX = 0;
  static unsigned long stepStartTimeY = 0;
  unsigned long currentMicros = micros();

  if (bresenhamActive) {
    bresenhamStepCounter++;
    if (bresenhamStepCounter >= bresenhamSpeedDivider) {
      bresenhamStepCounter = 0;
      bresenhamErrorX += bresenhamStepsX;
      bresenhamErrorY += bresenhamStepsY;
      if (bresenhamErrorX >= bresenhamTotalSteps) {
        bresenhamErrorX -= bresenhamTotalSteps;
        if (stepStateX && (currentMicros - stepStartTimeX >= STEP_PULSE_WIDTH)) {
          digitalWrite(STEP_PIN_X1, LOW);
          digitalWrite(STEP_PIN_X2, LOW);
          stepStateX = false;
        }
        if (!stepStateX) {
          if (moveDirectionX) currentPositionX++;
          else currentPositionX--;
          digitalWrite(STEP_PIN_X1, HIGH);
          digitalWrite(STEP_PIN_X2, HIGH);
          stepStateX = true;
          stepStartTimeX = currentMicros;
        }
      }
      if (bresenhamErrorY >= bresenhamTotalSteps) {
        bresenhamErrorY -= bresenhamTotalSteps;
        if (stepStateY && (currentMicros - stepStartTimeY >= STEP_PULSE_WIDTH)) {
          digitalWrite(STEP_PIN_Y, LOW);
          stepStateY = false;
        }
        if (!stepStateY) {
          if (moveDirectionY) currentPositionY++;
          else currentPositionY--;
          digitalWrite(STEP_PIN_Y, HIGH);
          stepStateY = true;
          stepStartTimeY = currentMicros;
        }
      }
      if (currentPositionX == targetPositionX && currentPositionY == targetPositionY) {
        bresenhamActive = false;
        isMovingX = false;
        isMovingY = false;
      }
    }
  } else {
    if (isMovingX && (currentPositionX != targetPositionX)) {
      stepCounterX++;
      if (stepCounterX >= speedDividerX) {
        stepCounterX = 0;
        if (stepStateX && (currentMicros - stepStartTimeX >= STEP_PULSE_WIDTH)) {
          digitalWrite(STEP_PIN_X1, LOW);
          digitalWrite(STEP_PIN_X2, LOW);
          stepStateX = false;
        }
        if (!stepStateX) {
          if (moveDirectionX) currentPositionX++;
          else currentPositionX--;
          digitalWrite(STEP_PIN_X1, HIGH);
          digitalWrite(STEP_PIN_X2, HIGH);
          stepStateX = true;
          stepStartTimeX = currentMicros;
        }
      }
    } else {
      if (isMovingX && currentPositionX == targetPositionX) isMovingX = false;
      if (stepStateX) {
        digitalWrite(STEP_PIN_X1, LOW);
        digitalWrite(STEP_PIN_X2, LOW);
        stepStateX = false;
      }
    }
    if (isMovingY && (currentPositionY != targetPositionY)) {
      stepCounterY++;
      if (stepCounterY >= speedDividerY) {
        stepCounterY = 0;
        if (stepStateY && (currentMicros - stepStartTimeY >= STEP_PULSE_WIDTH)) {
          digitalWrite(STEP_PIN_Y, LOW);
          stepStateY = false;
        }
        if (!stepStateY) {
          if (moveDirectionY) currentPositionY++;
          else currentPositionY--;
          digitalWrite(STEP_PIN_Y, HIGH);
          stepStateY = true;
          stepStartTimeY = currentMicros;
        }
      }
    } else {
      if (isMovingY && currentPositionY == targetPositionY) isMovingY = false;
      if (stepStateY) {
        digitalWrite(STEP_PIN_Y, LOW);
        stepStateY = false;
      }
    }
  }
}

void s_up() {
  delay(100);
  servo.write(70);
  delay(400);
}

void s_down() {
  delay(100);
  servo.write(145);
  delay(400);
}

void program1() {
  s_up();
  moveBoth(0, 50, 10000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }
  delay(1000);
  setSpeedX(3000);
  moveTicksX(99999);
  while (!isMoveCompleteX()) {
    if (sensor() < 50) {
      stopX();
      break;
    }
  }
  moveBoth(-2, 0, 10000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }
  s_down();
  moveBoth(0, 100, 10000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }
  s_up();
  moveBoth(-36, 0, 5000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }
  delay(1000);
  s_down();
  moveBoth(0, -100, 20000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }
  delay(1000);
  updateDisplay();
  delay(100000);
}

void program2() {
  moveBoth(220, 81, 10000, 0);
  while (!isBothMoveComplete()) {
    if (sensor() < 65) {
      stopBoth();
      updateDisplay();
      startY = (int)getPositionY();
      startX = (int)getPositionX();
      ex = true;
      break;
    }
  }

  moveBoth(-220, 81, 20000, 0);
  while (!isBothMoveComplete() || ex) {
    if (sensor() < 65) {
      stopBoth();
      updateDisplay();
      startY = (int)getPositionY();
      startX = (int)getPositionX();
      ex = true;
      break;
    }
  }

  setSpeedX(3000);
  moveTicksX(-startX + 30);
  while (!isMoveCompleteX()) {
    delay(1);
  }

  setSpeedX(3000);
  moveTicksX(99999);
  while (!isMoveCompleteX()) {
    if (sensor() < 60) {
      stopX();
      break;
    }
  }

  moveBoth(5, -startY + 15, 20000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }

  setSpeedY(20000);
  moveTicksY(99999);
  while (!isMoveCompleteY()) {
    if (sensor() < 50) {
      stopY();
      updateDisplay();
      delay(1000);
      startX = (int)getPositionY();
      break;
    }
  }

  setSpeedY(20000);
  moveTicksY(10);
  while (!isMoveCompleteY()) {
    delay(1);
  }

  moveTicksY(99999);
  while (!isMoveCompleteY()) {
    if (sensor() > 50) {
      stopY();
      updateDisplay();
      endX = (int)getPositionY();
      break;
    }
  }
  delay(1000);
  moveBoth(0, ((endX - startX) / 2) * -1, 20000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }
  s_down();
  setSpeedX(3000);
  moveTicksX(-100);
  while (!isMoveCompleteX()) {
    delay(1);
  }
}

void program3() {
  ex = false;
  for (int i = 0; i <= 16; ++i) {
    moveBoth(220, 0, 3000, 0);
    while (!isBothMoveComplete()) {
      if (sensor() < 65) {
        stopBoth();
        updateDisplay();
        startY = (int)getPositionY();
        startX = (int)getPositionX();
        ex = true;
        break;
      }
    }
    if (ex) {
      break;
    }

    moveBoth(-220, 10, 5000, 0);
    while (!isBothMoveComplete()) {
      delay(1);
    }
  }

  setSpeedX(3000);
  moveTicksX(-startX + 10);
  while (!isMoveCompleteX()) {
    delay(1);
  }

  setSpeedX(3000);
  moveTicksX(99999);
  while (!isMoveCompleteX()) {
    if (sensor() < 60) {
      startX = (int)getPositionX();
      stopX();
      break;
    }
  }

  moveBoth(5, -startY, 20000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }

  setSpeedY(20000);
  moveTicksY(99999);
  while (!isMoveCompleteY()) {
    if (sensor() < 50) {
      stopY();
      updateDisplay();
      delay(1000);
      startY = (int)getPositionY();
      break;
    }
  }

  setSpeedY(20000);
  moveTicksY(10);
  while (!isMoveCompleteY()) {
    delay(1);
  }

  moveTicksY(99999);
  while (!isMoveCompleteY()) {
    if (sensor() > 50) {
      stopY();
      updateDisplay();
      endY = (int)getPositionY();
      break;
    }
  }

  delay(1000);
  moveBoth(0, ((endY - startY) / 2) * -1, 20000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }

  delay(1000);
  moveBoth(260 - startX - 10, 0, 3000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }

  setSpeedX(3000);
  moveTicksX(-99999);
  while (!isMoveCompleteX()) {
    if (sensor() < 60) {
      endX = (int)getPositionX();
      stopX();
      break;
    }
  }

  delay(1000);

  moveBoth((endX - endY) / 2 * -1 - 56, 0, 1000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }

  int radius = 5;
  float speed = 2500;
  s_down();
  delay(2000);
  drawCircle((int)getPositionX(), (int)getPositionY(), radius, speed, 200);
  drawEllipse((int)getPositionX(), (int)getPositionY(), radius, radius, 4000, 36);

  delay(100000);
}

void program4() {
  for (int i = 0; i <= 32; ++i) {
    moveBoth(210, 0, 3500, 0);
    while (!isBothMoveComplete()) {
      if (sensor() < 50 && abs(getPositionX() - (float)curX) >= 2.0) {
        curX = (int)getPositionX();
        posX += curX;
        posY += (int)getPositionY();
        sum += 1;
        digitalWrite(led, HIGH);
        updateDisplay();
      }
      delay(1);
      digitalWrite(led, LOW);
    }
    delay(100);

    moveBoth(-210, 5, 5000, 0);
    while (!isBothMoveComplete()) {
      delay(1);
    }
    delay(200);
    curX = 0;
  }
  delay(1000);
  if (sum > 0) {
    moveBoth(posX / sum - (int)getPositionX() - 51, posY / sum - (int)getPositionY(), 10000, 0);
    while (!isBothMoveComplete()) {
      delay(1);
    }
  }
  drawEllipse((int)getPositionX(), (int)getPositionY(), 10, 10, 4000, 36);
}

void program5() {  
  for (int i = 0; i <= 16; ++i) {
    moveBoth(210, 0, 3500, 0);
    while (!isBothMoveComplete()) {
      if (sensor() < 50 && abs(getPositionX() - (float)curX) >= 2.0) {
        curX = (int)getPositionX();
        posX += curX;
        posY += (int)getPositionY();
        sum += 1;
        digitalWrite(led, HIGH);
        updateDisplay();
      }
      delay(1);
      digitalWrite(led, LOW);
    }
    delay(100);

    moveBoth(-210, 9.5, 5000, 0);
    while (!isBothMoveComplete()) {
      delay(1);
    }
    delay(200);
    curX = 0;
  }
  delay(1000);
  if (sum > 0) {
    cntrX = posX / sum - (int)getPositionX();
    cntrY = posY / sum - (int)getPositionY();
    moveBoth(cntrX, cntrY, 5000, 0);
    while (!isBothMoveComplete()) {
      delay(1);
    }
  }
  delay(1500);
  moveBoth(0, 999.0, 10000, 0);
  while (!isBothMoveComplete()) {
    if (sensor() > 50) {
      stopBoth();
      firstX = (int)getPositionX();
      firstY = (int)getPositionY();
      updateDisplay();
      break;
    }
  }
  moveBoth(-10, 15, 6000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }

  moveBoth(0, -999.0, 10000, 0);
  while (!isBothMoveComplete()) {
    if (sensor() < 50) {
      stopBoth();
      secondX = (int)getPositionX();
      secondY = (int)getPositionY();
      break;
    } 
  }
  cntrX = posX / sum - (int)getPositionX();
  cntrY = posY / sum - (int)getPositionY();
  moveBoth(cntrX, cntrY, 5000, 0);
  while (!isBothMoveComplete()) {
    delay(1);
  }
  delay(2000);
  alpha = atan2(10, firstY - secondY) * 180 / M_PI;
  updateDisplay();
  drawRotatedRectangle(getPositionX()-51, getPositionY(), 100, 62, alpha, 5000, 0, 1);

}

void program6() {
  setSpeedX(1000);
  setSpeedY(3000);
  moveBoth(30, 20, 1000, 0);
  while (!isBothMoveComplete()) {
    updateDisplay();
    delay(1);
  }
  setSpeedX(3000);
  setSpeedY(1000);
  moveBoth(-30, -20, 1000, 0);
  while (!isBothMoveComplete()) {
    updateDisplay();
    delay(1);
  }
}

void program7() {
  setSpeedY(500);
  moveTicksY(50);
  while (!isMoveCompleteY()) {
    updateDisplay();
    delay(1);
  }
  setSpeedY(4000);
  moveTicksY(-50);
  while (!isMoveCompleteY()) {
    updateDisplay();
    delay(1);
  }
}

void program8() {
  moveBoth(40, 15, 2500, 0);
  while (!isBothMoveComplete()) {
    updateDisplay();
    delay(1);
  }
}

void program9() {
  while (1) {
    // Empty loop
  }
}

void program10() {
  float centerX = getPositionX();
  float centerY = getPositionY();
  drawEllipse((int)centerX, (int)centerY, 75, 30, 4000, 36);
  s_up();
}

void program11() {
  float centerX = getPositionX();
  float centerY = getPositionY();
  int radius = 50;
  float speed = 8000;
  drawCircle((int)centerX, (int)centerY, radius, speed, 2000);
  s_up();
}

void loop() {
  s_up();
  switch (selectedProgram) {
    case 1: program1(); break;
    case 2: program2(); break;
    case 3: program3(); break;
    case 4: program4(); break;
    case 5: program5(); break;
    case 6: program6(); break;
    case 7: program7(); break;
    case 8: program8(); break;
    case 9: program9(); break;
    case 10: program10(); break;
    case 11: program11(); break;
  }
  delay(5000);
}