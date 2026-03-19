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

float side = 0;
float r = 0;

union FloatUnion {
  float value;
  uint8_t bytes[4];
};

uint8_t crc8(uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}

float circleX0 = 0; 
float circleY0 = 0; 
float circleR = 0;  

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
void calibration();
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

void sendX(float x);
int receiveY(float &y1, float &y2);
void findCircle();
void analyzeResults(float xVals[], float yVals[][2], int counts[], int numSamples);

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  
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

  
  display.clearDisplay();
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
  display.println(r, 1);
  display.setCursor(0, 32);
  display.print("Y: ");
  display.println(side, 1);
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
    moveBoth(dx_start, dy_start, speed/2, 1);
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

  moveBoth(dx, dy, speed, 0);

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

void calibration(){
  moveBoth(999, 999, 20000, 0);
  while(!isBothMoveComplete()){
    delay(1);
    if (sensor() < 30){
      stopBoth();
      break;
    }
  }
  delay(100);
  moveBoth(2, 0, 3000, 0);
  while(!isBothMoveComplete()){
    delay(1);
  }
  moveBoth(999, 0, 2000, 0);
  while(!isBothMoveComplete()){
    delay(1);
    if (sensor() > 70){
      stopBoth();
      break;
    }
  }
  delay(100);
  currentPositionX = 0;
  moveBoth(10, 0, 3000, 0);
  while(!isBothMoveComplete()){
    delay(1);
  }
  moveBoth(0, -999, 20000, 0);
  while(!isBothMoveComplete()){
    delay(1);
    if (sensor() < 30){
      stopBoth();
      break;
    }
  }
  currentPositionY = 0;
  // moveBoth(70, 80, 20000, 0);
  delay(1000);
}

void calibrationX(){
  moveBoth(999, 999, 20000, 0);
  while(!isBothMoveComplete()){
    delay(1);
    if (sensor() < 30){
      stopBoth();
      break;
    }
  }
  delay(100);
  moveBoth(2, 0, 3000, 0);
  while(!isBothMoveComplete()){
    delay(1);
  }
  moveBoth(999, 0, 2000, 0);
  while(!isBothMoveComplete()){
    delay(1);
    if (sensor() > 70){
      stopBoth();
      break;
    }
  }
  delay(100);
  currentPositionX = 0;
}

void moveSin(float startX_mm, float startY_mm, float width_mm, float amplitude_mm, int periods, float speed) {
  long startX_steps = (long)(startX_mm * koef_x);
  long startY_steps = (long)(startY_mm * koef_y);
  int segments = 1000;
  
  s_up();
  long dx_start = startX_steps - currentPositionX;
  long dy_start = startY_steps - currentPositionY;
  if (dx_start != 0 || dy_start != 0) {
    moveBoth(dx_start, dy_start, speed, 1);
    while (!isBothMoveComplete()) delay(1);
  }
  
  s_down();
  
  long prevX = startX_steps;
  long prevY = startY_steps;
  
  for (int i = 1; i <= segments; i++) {
    float t = (float)i / segments;
    float x_mm = startX_mm + t * width_mm;
    float y_mm = startY_mm + amplitude_mm * sin(2 * PI * periods * t);
    
    long x_steps = (long)(x_mm * koef_x);
    long y_steps = (long)(y_mm * koef_y);
    
    long dx = x_steps - prevX;
    long dy = y_steps - prevY;
    
    moveBoth(dx, dy, speed, 1);
    while (!isBothMoveComplete()) delay(1);
    
    prevX = x_steps;
    prevY = y_steps;
  }
  
  s_up();
}

void moveParabola(float vertexX_mm, float vertexY_mm, float a, float width_mm, float speed) {
  float x_left = vertexX_mm - width_mm / 2.0;
  float x_right = vertexX_mm + width_mm / 2.0;
  
  long leftX_steps = (long)(x_left * koef_x);
  long leftY_steps = (long)((a * (x_left - vertexX_mm) * (x_left - vertexX_mm) + vertexY_mm) * koef_y);
  
  int segments = 1000;
  
  s_up();
  long dx_start = leftX_steps - currentPositionX;
  long dy_start = leftY_steps - currentPositionY;
  if (dx_start != 0 || dy_start != 0) {
    moveBoth(dx_start, dy_start, speed, 1);
    while (!isBothMoveComplete()) delay(1);
  }
  
  s_down();
  
  long prevX = leftX_steps;
  long prevY = leftY_steps;
  
  for (int i = 1; i <= segments; i++) {
    float t = (float)i / segments;
    float x_mm = x_left + t * width_mm;
    float y_mm = a * (x_mm - vertexX_mm) * (x_mm - vertexX_mm) + vertexY_mm;
    
    long x_steps = (long)(x_mm * koef_x);
    long y_steps = (long)(y_mm * koef_y);
    
    long dx = x_steps - prevX;
    long dy = y_steps - prevY;
    
    moveBoth(dx, dy, speed, 1);
    while (!isBothMoveComplete()) delay(1);
    
    prevX = x_steps;
    prevY = y_steps;
  }
  
  s_up();
}

void drawTriangle(float cx, float cy, float side, float angle, float speed) {
  float height = side * sqrt(3) / 2.0;
  
  float Ax_local = 0.0;
  float Ay_local = height * 2.0 / 3.0;
  float Bx_local = -side / 2.0;
  float By_local = -height / 3.0;
  float Cx_local = side / 2.0;
  float Cy_local = -height / 3.0;
  
  float rad = angle * PI / 180.0;
  float cosA = cos(rad);
  float sinA = sin(rad);
  
  float Ax = cx + Ax_local * cosA - Ay_local * sinA;
  float Ay = cy + Ax_local * sinA + Ay_local * cosA;
  float Bx = cx + Bx_local * cosA - By_local * sinA;
  float By = cy + Bx_local * sinA + By_local * cosA;
  float Cx = cx + Cx_local * cosA - Cy_local * sinA;
  float Cy = cy + Cx_local * sinA + Cy_local * cosA;
  
  s_up();
  moveToPoint(Ax, Ay, speed);
  s_down();
  moveToPoint(Bx, By, speed);
  moveToPoint(Cx, Cy, speed);
  moveToPoint(Ax, Ay, speed);
  s_up();
  
  float ABx = (Ax + Bx) / 2.0;
  float ABy = (Ay + By) / 2.0;
  float BCx = (Bx + Cx) / 2.0;
  float BCy = (By + Cy) / 2.0;
  float CAx = (Cx + Ax) / 2.0;
  float CAy = (Cy + Ay) / 2.0;
  
  s_up();
  moveToPoint(Cx, Cy, speed);
  s_down();
  moveToPoint(ABx, ABy, speed);
  s_up();
  
  moveToPoint(Ax, Ay, speed);
  s_down();
  moveToPoint(BCx, BCy, speed);
  s_up();
  
  moveToPoint(Bx, By, speed);
  s_down();
  moveToPoint(CAx, CAy, speed);
  s_up();
}

void mark(){
  moveBoth(15, -15, 10000, 0);
  while(!isBothMoveComplete()){
    delay(1);
  }
  s_down(); 
  moveBoth(-30, 30, 10000, 0);
  while(!isBothMoveComplete()){
    delay(1);
  }
  s_up(); 
  moveBoth(0, -30, 10000, 0); 
  while(!isBothMoveComplete()){
    delay(1);
  }
  s_down();
  moveBoth(30, 30, 10000, 0); 
  while(!isBothMoveComplete()){
    delay(1);
  }


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

void sendX(float x) {
  uint8_t packet[7];
  
  packet[0] = 0xAA;
  packet[1] = 0xBB;
  
  FloatUnion fu;
  fu.value = x;
  
  packet[2] = fu.bytes[0];
  packet[3] = fu.bytes[1];
  packet[4] = fu.bytes[2];
  packet[5] = fu.bytes[3];
  
  packet[6] = crc8(&packet[2], 4);
  
  Serial3.write(packet, 7);
}

int receiveY(float &y1, float &y2) {
  const int TIMEOUT = 100;
  unsigned long startTime = millis();
  
  // Ждем заголовок 0xFF
  while (millis() - startTime < TIMEOUT) {
    if (Serial3.available() && Serial3.read() == 0xFF) {
      break;
    }
  }
  if (millis() - startTime >= TIMEOUT) return -2;
  
  // Ждем длину данных
  startTime = millis();
  while (millis() - startTime < TIMEOUT) {
    if (Serial3.available()) {
      uint8_t dataLen = Serial3.read();
      
      if (dataLen == 0) {
        while (millis() - startTime < TIMEOUT) {
          if (Serial3.available()) {
            Serial3.read(); // CRC
            return 0;
          }
        }
      } 
      else if (dataLen == 4) {
        FloatUnion fu;
        for (int i = 0; i < 4; i++) {
          startTime = millis();
          while (millis() - startTime < TIMEOUT) {
            if (Serial3.available()) {
              fu.bytes[i] = Serial3.read();
              break;
            }
          }
        }
        
        startTime = millis();
        while (millis() - startTime < TIMEOUT) {
          if (Serial3.available()) {
            uint8_t crc = Serial3.read();
            if (crc == crc8(fu.bytes, 4)) {
              y1 = fu.value;
              return 1;
            } else {
              return -1;
            }
          }
        }
      }
      else if (dataLen == 8) {
        FloatUnion fu1, fu2;
        
        for (int i = 0; i < 4; i++) {
          startTime = millis();
          while (millis() - startTime < TIMEOUT) {
            if (Serial3.available()) {
              fu1.bytes[i] = Serial3.read();
              break;
            }
          }
        }
        
        for (int i = 0; i < 4; i++) {
          startTime = millis();
          while (millis() - startTime < TIMEOUT) {
            if (Serial3.available()) {
              fu2.bytes[i] = Serial3.read();
              break;
            }
          }
        }
        
        startTime = millis();
        while (millis() - startTime < TIMEOUT) {
          if (Serial3.available()) {
            uint8_t crc = Serial3.read();
            uint8_t data[8];
            memcpy(data, fu1.bytes, 4);
            memcpy(data + 4, fu2.bytes, 4);
            
            if (crc == crc8(data, 8)) {
              y1 = fu1.value;
              y2 = fu2.value;
              return 2;
            } else {
              return -1;
            }
          }
        }
      }
    }
  }
  
  return -2;
}

void findCircle() {
  const int NUM_SAMPLES = 50;
  float xValues[NUM_SAMPLES];
  float yIntersections[NUM_SAMPLES][2];
  int intersectionCount[NUM_SAMPLES];
  int sampleIndex = 0;
  
  Serial.println("Searching for circle...");
  
  // Сканируем X от -8 до 8 с шагом 0.3
  for (float x = -8.0; x <= 8.0 && sampleIndex < NUM_SAMPLES; x += 0.3) {
    
    sendX(x);
    delay(50);
    
    float y1, y2;
    int result = receiveY(y1, y2);
    
    if (result >= 0) {
      xValues[sampleIndex] = x;
      
      if (result == 0) {
        intersectionCount[sampleIndex] = 0;
        sampleIndex++;
      }
      else if (result == 1) {
        yIntersections[sampleIndex][0] = y1;
        intersectionCount[sampleIndex] = 1;
        sampleIndex++;
      }
      else if (result == 2) {
        // Сортируем Y, чтобы y1 был меньше y2
        if (y1 < y2) {
          yIntersections[sampleIndex][0] = y1;
          yIntersections[sampleIndex][1] = y2;
        } else {
          yIntersections[sampleIndex][0] = y2;
          yIntersections[sampleIndex][1] = y1;
        }
        intersectionCount[sampleIndex] = 2;
        sampleIndex++;
      }
    }
    
    delay(50);
  }
  
  Serial.print("Collected "); Serial.print(sampleIndex); Serial.println(" samples");
  
  analyzeResults(xValues, yIntersections, intersectionCount, sampleIndex);
}

void analyzeResults(float xVals[], float yVals[][2], int counts[], int numSamples) {
  
  // Сначала соберем все точки с двумя пересечениями
  int twoPointsCount = 0;
  float twoPointsX[50], twoPointsY1[50], twoPointsY2[50];
  
  for (int i = 0; i < numSamples; i++) {
    if (counts[i] == 2) {
      twoPointsX[twoPointsCount] = xVals[i];
      twoPointsY1[twoPointsCount] = yVals[i][0];
      twoPointsY2[twoPointsCount] = yVals[i][1];
      twoPointsCount++;
    }
  }
  
  Serial.print("Two-intersection points: "); Serial.println(twoPointsCount);
  
  // Если есть точки с двумя пересечениями
  if (twoPointsCount >= 3) {
    
    // Метод 1: Используем свойство, что среднее арифметическое Y1 и Y2 = Y0
    float sumY0 = 0;
    for (int i = 0; i < twoPointsCount; i++) {
      sumY0 += (twoPointsY1[i] + twoPointsY2[i]) / 2.0;
    }
    circleY0 = sumY0 / twoPointsCount;
    
    Serial.print("Estimated Y0: "); Serial.println(circleY0);
    
    // Метод 2: Для каждого X, разность Y1 и Y2 дает информацию о R и X0
    // (y1 - y2)/2 = sqrt(R^2 - (x - X0)^2)
    
    // Создаем массив для хранения предполагаемых X0 от каждой пары точек
    float x0Candidates[100];
    int candidateCount = 0;
    
    for (int i = 0; i < twoPointsCount; i++) {
      for (int j = i + 1; j < twoPointsCount; j++) {
        float x1 = twoPointsX[i];
        float x2 = twoPointsX[j];
        float dy1 = (twoPointsY1[i] - twoPointsY2[i]) / 2.0;
        float dy2 = (twoPointsY1[j] - twoPointsY2[j]) / 2.0;
        
        // Из уравнений:
        // R^2 = (x1 - X0)^2 + dy1^2
        // R^2 = (x2 - X0)^2 + dy2^2
        // Приравниваем: (x1 - X0)^2 + dy1^2 = (x2 - X0)^2 + dy2^2
        // (x1^2 - 2x1*X0 + X0^2) + dy1^2 = (x2^2 - 2x2*X0 + X0^2) + dy2^2
        // x1^2 - 2x1*X0 + dy1^2 = x2^2 - 2x2*X0 + dy2^2
        // -2x1*X0 + 2x2*X0 = x2^2 - x1^2 + dy2^2 - dy1^2
        // 2X0(x2 - x1) = (x2^2 - x1^2) + (dy2^2 - dy1^2)
        // X0 = [(x2^2 - x1^2) + (dy2^2 - dy1^2)] / [2(x2 - x1)]
        
        float numerator = (x2*x2 - x1*x1) + (dy2*dy2 - dy1*dy1);
        float denominator = 2.0 * (x2 - x1);
        
        if (abs(denominator) > 0.001) {
          float x0 = numerator / denominator;
          
          // Проверяем, что x0 в разумных пределах
          if (x0 >= -10.0 && x0 <= 10.0) {
            x0Candidates[candidateCount++] = x0;
          }
        }
      }
    }
    
    Serial.print("X0 candidates: "); Serial.println(candidateCount);
    
    // Находим наиболее часто встречающееся значение X0
    if (candidateCount > 0) {
      // Сортируем кандидатов
      for (int i = 0; i < candidateCount - 1; i++) {
        for (int j = i + 1; j < candidateCount; j++) {
          if (x0Candidates[i] > x0Candidates[j]) {
            float temp = x0Candidates[i];
            x0Candidates[i] = x0Candidates[j];
            x0Candidates[j] = temp;
          }
        }
      }
      
      // Группируем близкие значения
      float bestX0 = 0;
      int maxFrequency = 0;
      
      int startIdx = 0;
      for (int i = 0; i < candidateCount; i++) {
        if (i == candidateCount - 1 || x0Candidates[i+1] - x0Candidates[i] > 0.5) {
          int frequency = i - startIdx + 1;
          if (frequency > maxFrequency) {
            maxFrequency = frequency;
            // Среднее значение в этой группе
            float sum = 0;
            for (int k = startIdx; k <= i; k++) {
              sum += x0Candidates[k];
            }
            bestX0 = sum / frequency;
          }
          startIdx = i + 1;
        }
      }
      
      circleX0 = bestX0;
      Serial.print("Selected X0: "); Serial.println(circleX0);
      
      // Теперь вычисляем R как среднее по всем точкам
      float sumR = 0;
      int countR = 0;
      
      for (int i = 0; i < twoPointsCount; i++) {
        float x = twoPointsX[i];
        float dx = x - circleX0;
        
        // Вычисляем R из этой точки
        float r1 = sqrt(dx*dx + (twoPointsY1[i] - circleY0)*(twoPointsY1[i] - circleY0));
        float r2 = sqrt(dx*dx + (twoPointsY2[i] - circleY0)*(twoPointsY2[i] - circleY0));
        
        sumR += (r1 + r2) / 2.0;
        countR++;
      }
      
      circleR = sumR / countR;
      
      Serial.print("Calculated R: "); Serial.println(circleR);
      return;
    }
  }
  
  // Если не получилось с двумя пересечениями, используем точки касания
  int touchCount = 0;
  float touchX[50], touchY[50];
  
  for (int i = 0; i < numSamples; i++) {
    if (counts[i] == 1) {
      touchX[touchCount] = xVals[i];
      touchY[touchCount] = yVals[i][0];
      touchCount++;
    }
  }
  
  Serial.print("Touch points: "); Serial.println(touchCount);
  
  if (touchCount >= 2) {
    // Для точек касания: Y0 = Y точки касания
    float sumY = 0;
    for (int i = 0; i < touchCount; i++) {
      sumY += touchY[i];
    }
    circleY0 = sumY / touchCount;
    
    // X0 - среднее всех X
    float sumX = 0;
    for (int i = 0; i < touchCount; i++) {
      sumX += touchX[i];
    }
    circleX0 = sumX / touchCount;
    
    // Радиус - среднее расстояние от X0
    float sumR = 0;
    for (int i = 0; i < touchCount; i++) {
      sumR += abs(touchX[i] - circleX0);
    }
    circleR = sumR / touchCount;
    
    Serial.print("From touch: X0="); Serial.print(circleX0);
    Serial.print(", Y0="); Serial.print(circleY0);
    Serial.print(", R="); Serial.println(circleR);
    return;
  }
  
  // Если ничего не нашлось
  circleX0 = 0;
  circleY0 = 0;
  circleR = 5;
  Serial.println("Using default values");
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

int posArray[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
bool binaryNum[] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 
int oldNum = 0;
int sensorIndications[] = {0, 0, 0, 0, 0, 0};
int sensorPos1 = 0;
int sensorPos2 = 0;
int pointsKordsX[] = {106, 106, 106, 76, 76, 76, 46, 46, 46};
int pointsKordsY[] = {50, 80, 110, 50, 80, 110, 50, 80, 110};
int sumPoints = 0;

void program6() {
  moveBoth(12, 30, 15000, 0);
  while(!isBothMoveComplete()){
    delay(1);
  }
  // delay(2000);
  for(int a = 0; a <= 2; ++a){
    moveBoth(20, 0, 4000, 0);
    while(!isBothMoveComplete()){
      delay(1);
    }
    // delay(500);
    sensorIndications[a] = sensor() < 50 ? 1 : 0;
    moveBoth(10, 0, 4000, 0);
    while(!isBothMoveComplete()){
      delay(1);
    }
    // delay(500);

    sensorIndications[a] += sensor() < 50 ? 2 : 0;
    Serial.println(sensorIndications[a]);
}

moveBoth(27, -9, 15000, 0);
  while(!isBothMoveComplete()){
    delay(1);
  }
  // delay(2000);
  for(int a = 3; a <= 5; ++a){
    moveBoth(-10, 29, 15000, 0);
    while(!isBothMoveComplete()){
      delay(1);
    }
    // delay(500);
    sensorIndications[a] = sensor() < 50 ? 1 : 0;
    moveBoth(10, 0, 4000, 0);
    while(!isBothMoveComplete()){
      delay(1);
    }
    // delay(500);

    sensorIndications[a] += sensor() < 50 ? 2 : 0;
    Serial.println(sensorIndications[a]);
}
Serial.println();
for(int a = 0; a <= 5; ++a){
  Serial.print(sensorIndications[a]);
}
delay(5000);


  for (int i = 0; i <= 512; ++i){
    oldNum = i;
    for (int n = 0; n <= 8; ++n){
      binaryNum[n] = oldNum % 2;
      oldNum = floor(oldNum/2);
      Serial.print(binaryNum[n]);
    }
    Serial.println();
    // // delay(1000);
    if (sensorIndications[0] == (binaryNum[6] + binaryNum[7] + binaryNum[8])){      
      if (sensorIndications[1] == (binaryNum[3] + binaryNum[4] + binaryNum[5])){
        if (sensorIndications[2] == (binaryNum[0] + binaryNum[1] + binaryNum[2])){
          if (sensorIndications[3] == (binaryNum[0] + binaryNum[3] + binaryNum[6])){
            if (sensorIndications[4] == (binaryNum[1] + binaryNum[4] + binaryNum[7])){
              if (sensorIndications[5] == (binaryNum[2] + binaryNum[5] + binaryNum[8])){
                delay(1000);
                for(int i = 0; i <= 8; ++i){
                  if (binaryNum[i] == 1){
                  moveToPoint(pointsKordsX[i]-51, pointsKordsY[i], 5000);
                  drawCircle(getPositionX(), getPositionY(), 3, 10000, 100);
                  delay(500);
                }
              }
              break;
              delay(1000000);
              }
            }
          }
        }
      }
    }
  }

}

void program7() {
   drawTriangle(0, 0, 80.0, 0.0, 7000);
}

bool st = 0;
int repetitions = 0;
int centrX[] = {0, 0};
int centrY[] = {0, 0};
void program8() {
  calibrationX();
  moveToPoint(15, 10, 7000);
  while (!isBothMoveComplete()) delay(1);

  float centersX[5];
  float centersY[5];
  int found = 0;
  int repetitions = 0;

  while (found < 5) {
    int st = 0;
    int posX = 0, posY = 0, sum = 0;
    int curX = 0;
    int i;
    
    for (i = repetitions; i <= 16; i++) {
      moveBoth(210, 0, 4000, 0);
      while (!isBothMoveComplete()) {
        if (sensor() < 50 && abs(getPositionX() - (float)curX) >= 2.0) {
          curX = (int)getPositionX();
          posX += curX;
          posY += (int)getPositionY();
          sum += 1;
          st = 1;
          digitalWrite(led, HIGH);
          updateDisplay();
        }
        delay(1);
        digitalWrite(led, LOW);
      }
      delay(100);

      moveBoth(-210, 9.75, 4500, 0);
      while (!isBothMoveComplete()) delay(1);
      delay(200);
      curX = 0;

      if (st == 0 && sum > 0) {
        centersX[found] = (float)posX / sum - 51.0;
        centersY[found] = (float)posY / sum;
        found++;
        repetitions = i + 1;
        break;
      }
      st = 0;
    }
    
    if (i > 16) break;
  }

  delay(1000);

  s_down();
  for (int n = 0; n < found; n++) {
    moveToPoint(centersX[n], centersY[n], 8000);
    while (!isBothMoveComplete()) delay(1);
    drawCircle((int)getPositionX(), (int)getPositionY(), 4, 4000, 100);
  }
  s_up();
}

void program9() {
  findCircle();
  
  Serial.println(circleX0);
  Serial.println(circleY0);
  delay(100000);
  moveBoth(circleX0*10 - 51, circleY0*10, 4000, 0);
  while(!isBothMoveComplete()){
    delay(1);
  }
  
  s_down();
  delay(1000);
  drawCircle(getPositionX(), getPositionY(), circleR*10, 4000, 200);
  s_up();
  
  while(1) {
    delay(1000);
  }
}

int firstPos = 0;
int secondPos = 0;
int kordsX[] = {105, 105, 105, 63, 63, 63, 21, 21, 21};
int kordsY[] = {35, 80, 125, 35, 80, 125, 35, 80, 125};
int points[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int targetPoints[] = {0, 0, 0};
int counter = 0;
// int sum = 0;

void program10() {
  calibrationX();
  for (int i = 0; i <= 2; ++i){
    moveToPoint(kordsX[i], kordsY[i], 10000);
    points[i] = sensor() > 50 ? 0 : 1;
    Serial.print(points[i]);
  }
  for (int i = 3; i <= 5; ++i){
    moveToPoint(kordsX[i], kordsY[i], 10000);
    points[i] = sensor() > 50 ? 0 : 1;
    Serial.print(points[i]);
  }
  for (int i = 6; i <= 8; ++i){
    moveToPoint(kordsX[i], kordsY[i], 10000);
    points[i] = sensor() > 50 ? 0 : 1;
    Serial.print(points[i]);
  }
  Serial.println();

  if ((points[0] + points[1] + points[2]) == 2){
    sum = points[1] + points[2] * 2;
    targetPoints[counter] = 3 - sum;
    counter += 1;
    sum = 0;
  }

  if ((points[3] + points[4] + points[5]) == 2){
    sum = points[3] * 3 - points[4] * 4 + points[5] * 5;
    targetPoints[counter] = 12 - sum;
    counter += 1;
    sum = 0;
  }
  
  if ((points[6] + points[7] + points[8]) == 2){
    sum = points[6] * 6 - points[7] * 7 + points[8] * 8;
    targetPoints[counter] = 21 - sum;
    counter += 1;
    sum = 0;
  }

  if ((points[0] + points[3] + points[6]) == 2){
    sum = points[3] * 3 + points[6] * 6;
    targetPoints[counter] = 9 - sum;
    counter += 1;
    sum = 0;
  }

  if ((points[1] + points[4] + points[7]) == 2){
    sum = points[1] * 1 - points[7] * 7 + points[4] * 4;
    targetPoints[counter] = 12 - sum;
    counter += 1;
    sum = 0;
  }

  if ((points[2] + points[5] + points[8]) == 2){
    sum = points[2] * 2 - points[5] * 5 + points[8] * 8;
    targetPoints[counter] = 15 - sum;
    counter += 1;
    sum = 0;
  }

  if ((points[0] + points[4] + points[8]) == 2){
    sum = points[4] * 4 + points[8] * 8;
    targetPoints[counter] = 12 - sum;
    counter += 1;
    sum = 0;
  }

  if ((points[2] + points[4] + points[6]) == 2){
    sum = points[2] * 2 - points[4] * 4 + points[6] * 6;
    targetPoints[counter] = 12 - sum;
    counter += 1;
    sum = 0;
  }
  
  for (int i = 0; i <= counter - 1; ++i){
    moveToPoint(kordsX[targetPoints[i]] - 51, kordsY[targetPoints[i]], 10000);
    // moveToPoint(-15 + getPositionX(), 15 + getPositionY(), 6000);
    // s_down();
    // moveToPoint(30 + getPositionX(), -30 + getPositionY(), 6000);
    // s_up();
    // moveToPoint(getPositionX(), 30 + getPositionY(), 6000);
    // s_down();
    // moveToPoint(30 + getPositionX(), 30 + getPositionY(), 6000);
    mark();
    s_up();
  }
}

void program11() {
  while(1){
    Serial.println(sensor());
  }
}

void loop() {
  selectedProgram = selectProgram();
  updateDisplay();
  s_up();
  // calibration();
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