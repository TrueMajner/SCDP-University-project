#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <LiquidCrystal_I2C.h>

// Class for detecting jumps
class JumpDetector {
public:
  JumpDetector(int ledPinX, int ledPinY, int ledPinZ, float jumpThreshold, int debounceDelay, int samplingInterval, int bufferDuration)
    : ledPinX(ledPinX),
      ledPinY(ledPinY),
      ledPinZ(ledPinZ),
      jumpThreshold(jumpThreshold),
      debounceDelay(debounceDelay),
      samplingInterval(samplingInterval),
      bufferSize(bufferDuration / samplingInterval),
      bufferIndex(0),
      lastJumpTime(0),
      verticalAxis("Undefined") {
    accel = new Adafruit_ADXL345_Unified(12345);
    accelXBuffer = new float[bufferSize]();
    accelYBuffer = new float[bufferSize]();
    accelZBuffer = new float[bufferSize]();
  }

  ~JumpDetector() {
    delete accel;
    delete[] accelXBuffer;
    delete[] accelYBuffer;
    delete[] accelZBuffer;
  }

  // Initialization of the sensor and LEDs
  void begin() {
    pinMode(ledPinX, OUTPUT);
    pinMode(ledPinY, OUTPUT);
    pinMode(ledPinZ, OUTPUT);

    digitalWrite(ledPinX, LOW);
    digitalWrite(ledPinY, LOW);
    digitalWrite(ledPinZ, LOW);

    if (!accel->begin()) {
      Serial.println("Failed to initialize ADXL345. Check the connection.");
      while (1)
        ;
    }

    accel->setRange(ADXL345_RANGE_16_G);
    Serial.println("ADXL345 is ready!");
  }

  // Detect if a jump has occurred
  bool checkForJump() {
    sensors_event_t event;
    accel->getEvent(&event);

    // Store current accelerations in the buffer
    accelXBuffer[bufferIndex] = abs(event.acceleration.x / 9.81);
    accelYBuffer[bufferIndex] = abs(event.acceleration.y / 9.81);
    accelZBuffer[bufferIndex] = abs(event.acceleration.z / 9.81);
    bufferIndex = (bufferIndex + 1) % bufferSize;

    // Calculate average acceleration
    float avgX = calculateAverage(accelXBuffer);
    float avgY = calculateAverage(accelYBuffer);
    float avgZ = calculateAverage(accelZBuffer);

    // Determine the current vertical axis
    determineVerticalAxis(avgX, avgY, avgZ);
    float vAcc = 0.0;

    if (verticalAxis == "X") {
      vAcc = abs(event.acceleration.x / 9.81);
    } else if (verticalAxis == "Y") {
      vAcc = abs(event.acceleration.y / 9.81);
    } else if (verticalAxis == "Z") {
      vAcc = abs(event.acceleration.z / 9.81);
    }

    // Check if the vertical acceleration exceeds the jump threshold
    if (vAcc > jumpThreshold && millis() - lastJumpTime > debounceDelay) {
      lastJumpTime = millis();
      indicateJump();
      return true;
    }

    return false;
  }

  // Light up the LED corresponding to the vertical axis
  void indicateVerticalAxis() {
    digitalWrite(ledPinX, LOW);
    digitalWrite(ledPinY, LOW);
    digitalWrite(ledPinZ, LOW);

    if (verticalAxis == "X") {
      digitalWrite(ledPinX, HIGH);
    } else if (verticalAxis == "Y") {
      digitalWrite(ledPinY, HIGH);
    } else if (verticalAxis == "Z") {
      digitalWrite(ledPinZ, HIGH);
    }
  }

private:
  Adafruit_ADXL345_Unified* accel;
  int ledPinX, ledPinY, ledPinZ;
  float jumpThreshold;
  int debounceDelay;
  int samplingInterval;
  int bufferSize;
  int bufferIndex;
  unsigned long lastJumpTime;
  String verticalAxis;
  float verticalAcceleration;
  float* accelXBuffer;
  float* accelYBuffer;
  float* accelZBuffer;

  // Calculate the average from a buffer
  float calculateAverage(float* buffer) {
    float sum = 0;
    for (int i = 0; i < bufferSize; i++) {
      sum += buffer[i];
    }
    return sum / bufferSize;
  }

  // Determine the vertical axis by finding the axis closest to Earth's gravity
  void determineVerticalAxis(float avgX, float avgY, float avgZ) {
    float diffX = abs(avgX - 1.0);
    float diffY = abs(avgY - 1.0);
    float diffZ = abs(avgZ - 1.0);

    if (diffX < diffY && diffX < diffZ) {
      verticalAxis = "X";
      verticalAcceleration = avgX;
    } else if (diffY < diffX && diffY < diffZ) {
      verticalAxis = "Y";
      verticalAcceleration = avgY;
    } else {
      verticalAxis = "Z";
      verticalAcceleration = avgZ;
    }
  }

  // Log jump detection
  void indicateJump() {
    Serial.println("Jump detected.");
  }
};

// Global JumpDetector object
JumpDetector jumpDetector(8, 9, 10, 1.5, 300, 50, 1000);

// Class for joystick handling
class Joystick {
  public:
  Joystick(int vrxPin, int vryPin, int centerX = 512, int centerY = 512, int deadZone = 100) 
      : vrxPin(vrxPin), vryPin(vryPin), centerX(centerX), centerY(centerY), deadZone(deadZone) {}

  // Method to get joystick deviation and determine direction
  String getDirection() {
    // Read analog values from the X and Y axes
    int xValue = analogRead(vrxPin);
    int yValue = analogRead(vryPin);

    // Calculate deviations from the center position
    int xDelta = xValue - centerX;
    int yDelta = yValue - centerY;

    // Check if the joystick is in the dead zone (no significant movement)
    if (abs(xDelta) < deadZone && abs(yDelta) < deadZone) {
      return "Center";
    }

    if (xValue > 670) return "Down";
    if (yValue > 670) return "Left";
    if (yValue < 30) return "Right";
    if (xValue < 30) return "Up";

    return "Center"; // Default case (should not reach here)
  }

  private: int vrxPin; // Analog pin for X-axis
  int vryPin; // Analog pin for Y-axis
  int centerX; // Center value for X-axis (default is 512)
  int centerY; // Center value for Y-axis (default is 512)
  int deadZone; // Tolerance for minor movements (default is 100)
};

// Example: Create a Joystick object with pins A0 and A1
Joystick joystick(A0, A1, 512, 512, 50); // Adjust center and dead zone as needed

// Class to manage LCD operations with custom symbols and text handling
class LCDManager {
public:
  // Constructor: accepts a reference to a LiquidCrystal_I2C object
  LCDManager(LiquidCrystal_I2C& lcdObj) : lcd(lcdObj) {}

  // Initialize the display and register custom characters
  void begin() {
    lcd.init();         // Initialize the LCD
    lcd.backlight();    // Turn on the backlight
    // Register custom characters
    lcd.createChar(0, dino);
    lcd.createChar(1, cactus);
    lcd.createChar(2, bird);
    lcd.createChar(3, arrowUp);
    lcd.createChar(4, arrowDown);
    lcd.createChar(5, arrowLeft);
    lcd.createChar(6, arrowRight);
    clear();            // Clear the screen initially
  }

  // Method to draw a custom symbol at a specific position (x, y)
  void drawSymbol(int x, int y, const String& symbolName) {
    lcd.setCursor(x, y);
    if (symbolName == "dino") {
      lcd.write(byte(0));
    } else if (symbolName == "cactus") {
      lcd.write(byte(1));
    } else if (symbolName == "bird") {
      lcd.write(byte(2));
    } else if (symbolName == "arrowUp") {
      lcd.write(byte(3));
    } else if (symbolName == "arrowDown") {
      lcd.write(byte(4));
    } else if (symbolName == "arrowLeft") {
      lcd.write(byte(5));
    } else if (symbolName == "arrowRight") {
      lcd.write(byte(6));
    } else {
      lcd.print(" ");
    }
  }

  // Method to draw a string at a specified position (x, y)
  void drawString(int x, int y, const String& text) {
    lcd.setCursor(x, y);
    lcd.print(text);
  }

  // Method to clear the display
  void clear() {
    lcd.clear();
  }

private:
  LiquidCrystal_I2C& lcd; // Reference to the LCD object

  byte dino[8] = {
    0x06,
    0x07,
    0x07,
    0x06,
    0x1E,
    0x1E,
    0x12,
    0x12
  };

  byte cactus[8] = {
    B00100,
    B00100,
    B11111,
    B00100,
    B11111,
    B00100,
    B11111,
    B00100
  };

  byte bird[8] = {
    0x00,
    0x0D,
    0x1F,
    0x0E,
    0x04,
    0x0E,
    0x00,
    0x00
  };

  // Кастомные символы стрелок
  byte arrowUp[8] = {
    0x00,
    0x04,
    0x0E,
    0x1F,
    0x04,
    0x04,
    0x04,
    0x00
  };

  byte arrowDown[8] = {
    0x00,
    0x04,
    0x04,
    0x04,
    0x1F,
    0x0E,
    0x04,
    0x00
  };

  byte arrowLeft[8] = {
    0x00,
    0x04,
    0x0C,
    0x1C,
    0x0C,
    0x04,
    0x04,
    0x00
  };

  byte arrowRight[8] = {
    0x00,
    0x10,
    0x18,
    0x1C,
    0x18,
    0x10,
    0x10,
    0x00
  };
};

// Create the LCD object with I2C address and size (16x2)
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Create the LCDManager object to control the display
LCDManager lcdManager(lcd);

class Dino {
public:
  // Constructor to initialize the Dino object
  Dino()
    : score(0), 
      maxScore(0), 
      arrowDirection("Up"), 
      updateRate(10),
      gameStartTime(0),
      lastUpdateTime(0),
      jumpingState(0),
      tick(0),
      inputAllowed(true),
      gameOverState(0) {
    resetCacti();
    resetBirds();
  }

  // Method to handle the game over scenario
  void gameOver() {
    setGameOver(true);
    lcdManager.drawString(0, 0, "Game over...");
    lcdManager.drawString(0, 1, "S" + String(score) + " Smax" + String(maxScore));
  }

  // Method to set the game over state
  void setGameOver(bool state) {
    gameOverState = state ? 100 : 0;
  }

  // Method to update the game state: move obstacles, increase score, check collisions
  void next() {
    tick++; // Increment the game tick
    if (gameOverState == 1) reset(); // Reset the game if it's over
    if (gameOverState > 0) gameOverState--; // Decrease game over state countdown

    // Update the game only at the specified rate (updateRate)
    if (tick % updateRate != 0 || isGameOver()) return;
    
    // Clear the screen
    lcdManager.clear();

    //If jumping, decrease the jumping state
    if(isJumping()) setJumpingState(getJumpingState() - 1);

    // Change update rate when score hits certain milestones
    if(score % 100 == 0 && updateRate > 2) updateRate -= 1; 

    // Update positions of obstacles and check for collisions
    for (int i = 0; i < 5; i++) {
      // Collision detection: if the dino is not jumping and hits a cactus, or is jumping and hits a bird, game over
      if((cacti[i] == score && !isJumping()) || (birds[i] == score && isJumping())) {
        lcdManager.clear();
        gameOver();
        return;
      }

      // Reset cactus position if it passed the dino
      if(cacti[i] < score - 1) {
        cacti[i] = getGoodPosition();
      }

      // Draw cactus if within range of the dino's screen
      if(cacti[i] >= score - 1 && cacti[i] < score + 12) {
        lcdManager.drawSymbol(cacti[i] - score + 1, 1, "cactus");
      }

      // Reset bird position if it passed the dino
      if(birds[i] < score - 1) {
        birds[i] = getGoodPosition();
      }
      
      // Draw bird if within range of the dino's screen
      if(birds[i] >= score - 1 && birds[i] < score + 12) {
        lcdManager.drawSymbol(birds[i] - score + 1, 0, "bird");
      }
    }

    // Display elapsed time and game status
    lcdManager.drawString(13, 0, String(getElapsedSeconds()) + "S");
    lcdManager.drawSymbol(13, 1, "arrow" + String(getArrowDirection()));
    lcdManager.drawString(14, 1, String(getJumpingState()));

    // Draw the dino symbol, depending on the jumping state
    lcdManager.drawSymbol(1, isJumping() ? 0 : 1, "dino");

    lastUpdateTime = millis(); // Update the last update time
    score++; // Increment score
    
    // Update the max score if the current score exceeds it
    if (score > maxScore) {
      maxScore = score;
    }
  }

  // Method to calculate the elapsed time in seconds since the game has started
  unsigned long getElapsedSeconds() const {
    return (millis() - gameStartTime) / 1000; // Time difference in seconds
  }

  // Method to start a jump action: toggle jumping state and randomize the arrow direction
  void jump() {
    if(!isInputAllowed()) return;
    setInputAllowed(false);
    setRandomArrowDirection();
    if(!isJumping()) setJumpingState(5);
    else setJumpingState(0);
  }

  // Method to allow or disallow further input (e.g., jumping... but there's only 1 action - jumping... so... it's only disabling jumping and I should wrote not further input but jumping... or not?) 
  void setInputAllowed(bool state) {
    inputAllowed = state;
  }

  //I DO NOT WANT TO COMMENT ANYMORE!!!
  void setJumpingState(int state) {
    jumpingState = state;
  }

  void setRandomArrowDirection() {
  // LEMME GO PLEASEEE
  const String directions[] = {"Left", "Up", "Right", "Down"};
  
  // Choose a random index from the directions array
  int randomIndex = random(0, 4); // Generates a random number between 0 and 3
  
  // Set arrowDirection to the chosen value
  arrowDirection = directions[randomIndex];
  }

  // Reset the game state
  void reset() {
    score = 0;
    arrowDirection = "Up";
    resetCacti();
    resetBirds();
    updateRate = 10;
    gameStartTime = millis();
    Serial.println("Game reset!");
  }

  // Accessor methods for debugging or game logic
  int getScore() const { return score; }
  int getMaxScore() const { return maxScore; }
  String getArrowDirection() const { return arrowDirection; }
  const int* getCactiPositions() const { return cacti; }
  const int* getBirdPositions() const { return birds; }
  int getUpdateRate() const { return updateRate; }
  bool isJumping() const { return jumpingState != 0; }
  int getJumpingState() const {return jumpingState;}
  bool isInputAllowed() const {return inputAllowed;}
  bool isGameOver() const {return gameOverState != 0;}

private:
  int score;                // Current game score
  int maxScore;             // Maximum score achieved
  String arrowDirection;    // Direction of arrow input (e.g., "Up", "Down"...)
  int cacti[5];             // Array to store cactus positions
  int birds[5];             // Array to store bird positions
  int updateRate;           // Update rate
  unsigned long gameStartTime; //no more comments
  unsigned long lastUpdateTime; //you can guess what does this variable store
  int jumpingState; //Somethink like jumping energy (height, but there's only 2 lines so...)
  int tick; //knock knock, who's there? - Open up
  bool inputAllowed; //Impossible to guess
  int gameOverState; //hahahhahahahah you lose! Noob!

  int getGoodPosition() {
    int pos;
    while(true) {
      bool good = true;
      pos = random(score + 10, score + 100);
      for (int i = 0; i < 5; i++) { 
        int currentPos1 = cacti[i];
        int currentPos2 = birds[i];
        if(abs(currentPos1 - pos) < 4 || abs(currentPos2 - pos) < 4) good = false;
      }
      if(good) break;
    }
    return pos;
  }

  // Reset cacti positions
  void resetCacti() {
    for (int i = 0; i < 5; i++) {
      cacti[i] = getGoodPosition(); // Random initial positions
    }
  }

  // Reset bird positions
  void resetBirds() {
    for (int i = 0; i < 5; i++) {
      birds[i] = getGoodPosition();  // Random initial positions
    }
  }
};

Dino dino;

void setup() {
  Serial.begin(9600);
  jumpDetector.begin();
  lcdManager.begin();
  lcdManager.drawString(0, 0, "Hello Dino!");
  lcdManager.drawSymbol(0, 1, "dino");
  lcdManager.drawSymbol(2, 1, "cactus");
  lcdManager.drawSymbol(4, 1, "bird");
  lcdManager.drawSymbol(5, 1, "arrowUp");
  lcdManager.drawSymbol(6, 1, "arrowDown");
  lcdManager.drawSymbol(7, 1, "arrowLeft");
  lcdManager.drawSymbol(8, 1, "arrowRight");

}

bool valueChanged = true;
int lastValue = LOW;

bool isButtonPressed(int buttonPin) {
  int currentValue = digitalRead(buttonPin);
  if (currentValue == HIGH && valueChanged) {
    valueChanged = false;
    lastValue = currentValue;
    return true;
  }

  if (currentValue != lastValue) {
    valueChanged = true;
    lastValue = currentValue;
  }

  return false;
}

void loop() {
  if (isButtonPressed(2)) {
    dino.jump();
  }

  if(digitalRead(2) != lastValue) valueChanged = true;

  if (jumpDetector.checkForJump()) {
    dino.jump();
  }

  if(joystick.getDirection() == "Center") dino.setInputAllowed(true);

  if(joystick.getDirection() == dino.getArrowDirection()) {
     dino.jump();
     dino.setInputAllowed(false);
  }

  jumpDetector.indicateVerticalAxis();
  dino.next();
  delay(50);
}
