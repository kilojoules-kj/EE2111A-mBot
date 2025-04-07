// EE2111A Project Code
#include "MeMCore.h"

// Motor objects
MBotDCMotor motorLeft(M1); // facing front of car, M1 M2 on the right hand 
MBotDCMotor motorRight(M2); // M2 is north/"above" of M1

void stopMotor() { motorLeft.stop(); motorRight.stop(); }
void moveForward(int speed) { motorLeft.run(-speed); motorRight.run(speed); } // -50 and 50 generate very low torque and will likely cause motor to stall.
void moveBackward(int speed) { motorLeft.run(speed); motorRight.run(-speed); }
void turnLeft(int speed) { motorLeft.run(speed); motorRight.run(speed); }
void turnRight(int speed) { motorLeft.run(-speed); motorRight.run(-speed); }
void turnLeft90() { motorLeft.run(200); motorRight.run(200); delay(420); stopMotor(); }
void turnRight90() { motorLeft.run(-200); motorRight.run(-200); delay(420); stopMotor(); }



// Need a new data structure to store the stupid coord
struct Position {
  int forwardSteps;  // net forward steps (forward - backward)
  int turnSteps;     // net left turns (left - right)
  bool originSet;    // whether origin has been set
};


// other variables declaration
MeRGBLed ledFrontLeft(PORT_7); // PORT_7 where it's connected to. Front LED
//MeLightSensor lightSensor(PORT_6);
MeLineFollower lineFinder(PORT_3);
MeUltrasonicSensor ultraSensor(PORT_1);
MeIR remote_ir_receiver;

// Pressure Sensor
#define pressureSensorPin A0


// Status pin - this is the receiver to the transmitter that is soldered
const int irLEDPin = 10;


// My State class
class State {
  protected:
    const char* _name;
    const MeBuzzer buzzer;
    const int constantSpeed = 100;
    const int constantTurnSpeed = 240;
    static int cornersSaved;
    static Position savedCorners[4];
    static Position currentPosition;
    


  public:
    void setLEDcolour(int R, int G, int B) { 
      ledFrontLeft.setColorAt(0, R,G,B); // 0 is right side led, 1 is left hand left, bot is facing away from you
      ledFrontLeft.show();
    } // called to display status with LED

    int readLineFinder() {
      return(lineFinder.readSensors());
    }

    bool isObstacle() {
      if (ultraSensor.distanceCm() <= 8) { return true; }
      return false;
    }

    bool isIRLed() {
      if (digitalRead(irLEDPin) == HIGH) { 
        return false; //Off
      } else {
        return true; // ON
      }
    }

    bool isPressured() {
      int pressureStatus = analogRead(pressureSensorPin)-1000;
      Serial.println(pressureStatus);
      
      if (pressureStatus > 10) { // 0 is OFF, max force can go higher but 50 was the best we could do
        Serial.println(F("Pressure above threshold"));
        return true;
      }
      return false;
    }

    virtual void handleObstacle() {
      stopMotor();
      setLEDcolour(255, 0, 0); // Red when avoiding
      
      // Small backward movement
      moveBackward(constantSpeed);
      delay(200);
      stopMotor();
      
      // Turn 90° right (or left if preferred)
      turnRight90();
      
      // Small forward movement to clear obstacle
      moveForward(constantSpeed);
      delay(500);
      stopMotor();
      
      // Turn back to original orientation
      turnLeft90();
      resetLED();
    }

    virtual void remoteIR(MeIR ir) {
      
      static unsigned long lastIRTime = 0;
      static uint32_t lastIRValue = 0;

      static unsigned long lastButtonCTime = 0;
      static unsigned long lastButtonSettingTime = 0;
      
      const unsigned long repeatDelay = 200;
      const unsigned long buttonDelay = 500;
      uint32_t value = 0; // for the ir remote
      bool isMoving = false;
    
      if(ir.decode()) {
          value = ir.value;
          value = value >> 16 & 0xff;
          lastIRValue = value;
          lastIRTime = millis();
      } else {
          if (millis() - lastIRTime < repeatDelay && lastIRValue != 0) {
              value = lastIRValue;
          } else {
              if (millis() - lastIRTime > repeatDelay && lastIRTime != 0) {
                  stopMotor();
                  lastIRValue = 0;
                  lastIRTime = 0;
              }
              return;
          }
      }

      switch(value) {
        case IR_BUTTON_C: 
           if (millis() - lastButtonCTime > buttonDelay) {
            lastButtonCTime = millis();
            
            toggleState();
          }
          break;

        default: break;
      }
    }

    virtual int getCornersCount() const { return cornersSaved; }
    virtual void resetLED() {} // the base led colour
    virtual void playTone() {}  // play tone
    virtual void initialise() {}  // entering state
    virtual void exit() {stopMotor();}   // exiting state
    virtual void update() { remoteIR(remote_ir_receiver); } // called each loop iteration
};

class PauseState : public State {    
  private:
    int forwardCount = 0;
    int backwardCount = 0;
    int leftCount = 0;
    int rightCount = 0;

  public:
    PauseState() { 
      _name = "PauseState"; 
    }
    
    void initialise() override {
      Serial.println(F("initialise PauseState behaviour"));
      setLEDcolour(150,150,0); // Yellow
      stopMotor();
      playTone();
    }

    void playTone() override {
      // Pause tone 
      buzzer.tone(392, 100); // G4
      delay(100); 
      buzzer.tone(392, 100); // G4
      delay(100);
      buzzer.noTone();
    }

    void updatePosition(int direction) {
      switch(direction) {
        case 1:
          moveForward(constantSpeed);
          forwardCount++;
          delay(200);
          break;
        case 2:
          moveBackward(constantSpeed);
          backwardCount++;
          delay(200);
          break;
        case 3:
          turnLeft(constantSpeed);
          leftCount++;
          delay(200);
          break;
        case 4:
          turnRight(constantSpeed);
          rightCount++;
          delay(200);
          break;
      }
      stopMotor();
    }

    void markCorner() {
      if (cornersSaved >= 4) {
        Serial.println(F("Maximum 4 corners already marked"));
        return;
      }
      
      if (!currentPosition.originSet) {
          // First corner - set as origin
          currentPosition.forwardSteps = 0;
          currentPosition.turnSteps = 0;
          currentPosition.originSet = true;
          Serial.println(F("Origin set as first corner"));
      } else {
          // Subsequent corners
          currentPosition.forwardSteps = forwardCount - backwardCount;
          currentPosition.turnSteps = leftCount - rightCount;
      }
      
      savedCorners[cornersSaved] = currentPosition;
      cornersSaved++;
      
      buzzer.tone(659, 200); // Play E5 tone when position is saved
      delay(200);
      buzzer.noTone();

      Serial.print(F("Corner "));
      Serial.print(getCornersCount());
      Serial.print(F(" marked: Forward steps="));
      Serial.print(currentPosition.forwardSteps);
      Serial.print(F(", Turn steps="));
      Serial.println(currentPosition.turnSteps);

      // Reset counts for next movement
      forwardCount = 0;
      backwardCount = 0;
      leftCount = 0;
      rightCount = 0;
    }

    void remoteIR(MeIR ir) override {
      
      static unsigned long lastIRTime = 0;
      static uint32_t lastIRValue = 0;
      static unsigned long lastButtonATime = 0;
      static unsigned long lastButtonBTime = 0;
      static unsigned long lastButtonCTime = 0;
      static unsigned long lastButtonSettingTime = 0;
      
      const unsigned long repeatDelay = 200;
      const unsigned long buttonDelay = 500;
      uint32_t value = 0; // for the ir remote
      bool isMoving = false;
    
      if(ir.decode()) {
          value = ir.value;
          value = value >> 16 & 0xff;
          lastIRValue = value;
          lastIRTime = millis();
      } else {
          if (millis() - lastIRTime < repeatDelay && lastIRValue != 0) {
              value = lastIRValue;
          } else {
              if (millis() - lastIRTime > repeatDelay && lastIRTime != 0) {
                  stopMotor();
                  lastIRValue = 0;
                  lastIRTime = 0;
              }
              return;
          }
      }

      switch(value) {
        case IR_BUTTON_A:
          if (millis() - lastButtonATime > buttonDelay) {
            lastButtonATime = millis();

            Serial.print(F("Checking how many corners marked: "));
            Serial.println(cornersSaved);
            toggleState();
          }
          break;

        case IR_BUTTON_SETTING:
          if (millis() - lastButtonSettingTime > buttonDelay) {
            lastButtonSettingTime = millis();
            markCorner();
          }
          break;

        case IR_BUTTON_B: 
          if (millis() - lastButtonBTime > buttonDelay) {
            lastButtonBTime = millis();
            
            currentPosition = {0,0, false};
            cornersSaved = 0;
            memset(savedCorners, 0, sizeof(savedCorners));

            buzzer.tone(659, 200); // Play E5 tone when position is saved
            delay(200);
            buzzer.noTone();
          }
          break;
        case IR_BUTTON_C: 
           if (millis() - lastButtonCTime > buttonDelay) {
            lastButtonCTime = millis();
            
            toggleState();
          }
          break;
        // Forward-1, Backward-2, Left-3, Right-4
        case IR_BUTTON_LEFT:
          if (cornersSaved < 4) {
            updatePosition(3); // Pure rotation
          }
          break;
        case IR_BUTTON_RIGHT:
          if (cornersSaved < 4) {
            updatePosition(4); // Pure rotation
          }
          break;
        case IR_BUTTON_UP:
          if (cornersSaved < 4) {
            updatePosition(1); // Move forward only
          }
          break;
        case IR_BUTTON_DOWN:
          if (cornersSaved < 4) {
            updatePosition(2); // Move forward only
          }
          break;

        default: break;
      }
    }

    void update() override {
      stopMotor();
      remoteIR(remote_ir_receiver); 
    }
};

class NavigateOrigin : public State {
  private:
    int currentCornerIndex;
    int currentFacing; // 0=initial, 1=right, 2=back, 3=left
    bool navigating;
    unsigned long lastMoveTime;
    const unsigned long moveDelay = 500;

  public:
    NavigateOrigin() { 
      _name = "NavigateOrigin"; 
      currentCornerIndex = 0;
      currentFacing = 0;
      navigating = false;
      lastMoveTime = 0;
    }
    
    void initialise() override {
      Serial.println(F("Starting navigation to origin"));
      setLEDcolour(0,0,200);
      
      if (getCornersCount() < 4) {
        toggleState();
        return;
      }
      
      currentCornerIndex = getCornersCount() - 1;
      currentFacing = 0; // Start facing original direction
      navigating = true;
      lastMoveTime = millis();
    }

    void executeReverseMovement() {
      if (currentCornerIndex < 0) {
        navigating = false;
        setLEDcolour(0, 255, 0);
        buzzer.tone(784, 200);
        delay(200);
        buzzer.tone(1047, 400);
        Serial.println(F("Origin reached!"));
        toggleState();
        return;
      }
      
      Position corner = savedCorners[currentCornerIndex];
      Serial.print(F("Reversing corner "));
      Serial.print(currentCornerIndex);
      Serial.print(F(" F="));
      Serial.print(corner.forwardSteps);
      Serial.print(F(" T="));
      Serial.println(corner.turnSteps);

      // 1. First undo the forward movement
      if (corner.forwardSteps != 0) {
        int steps = abs(corner.forwardSteps);
        Serial.print(F("Moving "));
        Serial.print(steps);
        Serial.println(corner.forwardSteps > 0 ? " backward" : " forward");
        
        for (int i = 0; i < steps; i++) {
          if (corner.forwardSteps > 0) {
            moveBackward(constantSpeed);
          } else {
            moveForward(constantSpeed);
          }
          delay(200);
          stopMotor();
          delay(100);
        }
      }

      // 2. Then undo the turns (important - reverse order!)
      if (corner.turnSteps != 0) {
        int turns = abs(corner.turnSteps);
        Serial.print(F("Turning "));
        Serial.print(turns);
        Serial.println(corner.turnSteps > 0 ? " right" : " left");
        
        for (int i = 0; i < turns; i++) {
          if (corner.turnSteps > 0) {
            turnRight(constantSpeed);
            currentFacing = (currentFacing + 3) % 4; // Update facing
          } else {
            turnLeft(constantSpeed);
            currentFacing = (currentFacing + 1) % 4;
          }
          delay(200);
          stopMotor();
          delay(100);
        }
      }

      currentCornerIndex--;
      lastMoveTime = millis();
    }

    void update() override {
      remoteIR(remote_ir_receiver); 

      if (!navigating) return;
      if (millis() - lastMoveTime > moveDelay) {
        executeReverseMovement();
      }
    }
};

class SnakeState : public State {
  private:
    const char* _name;
    int passesCompleted;
    bool movingForward;
    int stepsInCurrentPass;
    int xSteps, ySteps;
    unsigned long lastMoveTime;
    const int stepSize = 6;
    bool avoidingObstacle;
    int obstacleRetries;
    
  public:
    SnakeState() : 
      _name("SnakeState"),
      passesCompleted(0),
      movingForward(true),
      stepsInCurrentPass(0),
      avoidingObstacle(false),
      obstacleRetries(0) {}
    
    void initialise() override {
      Serial.println(F("Starting snake coverage"));
      resetLED();
      xSteps = abs(savedCorners[1].forwardSteps);
      ySteps = abs(savedCorners[2].turnSteps);
      Serial.print(F("Area dimensions: "));
      Serial.print(xSteps);
      Serial.print(F("x"));
      Serial.println(ySteps);
    }

    void resetLED() {
      setLEDcolour(0, 255, 255); // Cyan
    }

    void handleObstacle() override {
      stopMotor();
      setLEDcolour(255, 0, 0); // Red when avoiding
      
      // Small backward movement
      moveBackward(constantSpeed);
      delay(200);
      stopMotor();
      
      // Turn 90° right (or left if preferred)
      turnRight90();
      
      // Small forward movement to clear obstacle
      moveForward(constantSpeed);
      delay(300);
      stopMotor();
      
      // Turn back to original orientation
      turnLeft90();
      
      obstacleRetries++;
      if (obstacleRetries > 3) {
        // Can't get past after 3 tries - skip this segment
        stepsInCurrentPass = xSteps;
        Serial.println(F("Obstacle blocking - skipping segment"));
      }
      
      avoidingObstacle = false;
      resetLED();
    }

    void executeSnakeStep() {
      if (passesCompleted >= ySteps * 2) {
        setLEDcolour(0, 255, 0);
        buzzer.tone(1047, 400);
        Serial.println(F("Coverage complete!"));
  

        toggleState();
        return;
      }

      // Obstacle detection during movement
      if (isObstacle() && stepsInCurrentPass < xSteps) {
        avoidingObstacle = true;
        handleObstacle();
        return;
      }

      if (stepsInCurrentPass < xSteps) {
        if (isPressured()) {
          toggleStateDust(); // when pressure sensor, DustState
          return;
        }

        moveForward(constantSpeed);
        delay(200);
        stopMotor();
        stepsInCurrentPass++;
        obstacleRetries = 0; // Reset retry counter on successful move
        lastMoveTime = millis();
        return;
      }

      // U-turn logic
      Serial.println(F("Executing U-turn"));
      if (passesCompleted % 2 == 0) {
        
        turnRight90();
        moveForward(constantSpeed);
        delay(300);
        turnRight90();
      } else {
        
        turnLeft90();
        moveForward(constantSpeed);
        delay(300);
        turnLeft90();
      }
      passesCompleted++;
      stepsInCurrentPass = 0;
      movingForward = !movingForward;
      lastMoveTime = millis();

      stopMotor();
    }

    void update() override { 
      remoteIR(remote_ir_receiver); 

      if (millis() - lastMoveTime > 300 && !avoidingObstacle) {
        executeSnakeStep();
      }
    }
};

class StartState : public State {
  public:
    StartState() { _name = "StartState"; }
    
    void initialise() override {
      Serial.println(F("initialise StartState behaviour"));
      resetLED();
      playTone();
    }

    void playTone() override {
      // Start tone
      buzzer.tone(523, 200); // C5
      delay(200);
      buzzer.tone(659, 200); // E5
      delay(200);
      buzzer.tone(784, 200); // G5
      delay(200);
      buzzer.tone(1047, 400); // C6
      delay(400);
      buzzer.noTone();
    }

    void resetLED() {
      setLEDcolour(0,150,0); // Green
    }

    void randomisedRuns() {
      int lineLightSensorState =  readLineFinder();

      switch(lineLightSensorState) {
        case S1_IN_S2_IN:
          Serial.println(F("Sensor 1 and 2 are inside of black line"));
          stopMotor();
          moveBackward(constantSpeed);
          delay(300);

            switch (random(2)) {
              case 0: turnLeft(constantTurnSpeed); delay(200); break;
              case 1: turnRight(constantTurnSpeed); delay(200); break;
              default: break;
            }
          break;
        case S1_IN_S2_OUT:
          Serial.println(F("Sensor 2 is outside of black line"));
          moveBackward(constantSpeed);
          turnRight(constantTurnSpeed);
          break;
        case S1_OUT_S2_IN:
          Serial.println(F("Sensor 1 is outside of black line"));
          moveBackward(constantSpeed);
          turnLeft(constantTurnSpeed);
          break;
        case S1_OUT_S2_OUT:
          Serial.println(F("Sensor 1 and 2 are outside of black line"));
          if (isObstacle()) {
            stopMotor();

            handleObstacle();
            // switch (random(2)) {
            //   case 0: turnLeft(constantTurnSpeed); delay(200); break;
            //   case 1: turnRight(constantTurnSpeed); delay(200); break;
            //   default: break;
            // }
          } else {
            moveForward(constantSpeed);
          }

        default: break;
      }
    }

    void update() override {
      remoteIR(remote_ir_receiver); 

      if (isIRLed()) {
        toggleStateHold();
      }

      if (isPressured()) {
        toggleStateDust();
        return;
      }

      randomisedRuns();      
    }
};

class EndState : public State {
  public:
    EndState() { _name = "EndState"; }
    
    void initialise() override {
      Serial.println(F("initialise EndState behaviour"));
      setLEDcolour(200,0,0); //Red
      stopMotor(); // not needed, just in case
      playTone();
    }

    void playTone() override {
      buzzer.tone(1047, 200); // C6
      delay(200);
      buzzer.tone(784, 200); // G5
      delay(200);
      buzzer.tone(659, 200); // E5
      delay(200);
      buzzer.tone(523, 400); // C5
      delay(400);
      buzzer.noTone();
    }

    void remoteIR(MeIR ir) {
      static unsigned long lastIRTime = 0;
      static uint32_t lastIRValue = 0;
      static unsigned long lastButtonCTime = 0;
      const unsigned long repeatDelay = 200;
      const unsigned long buttonDelay = 500;
      uint32_t value = 0; // for the ir remote
    
      if(ir.decode()) {
          value = ir.value;
          value = value >> 16 & 0xff;
          lastIRValue = value;
          lastIRTime = millis();
      } else {
          if (millis() - lastIRTime < repeatDelay && lastIRValue != 0) {
              value = lastIRValue;
          } else {
              if (millis() - lastIRTime > repeatDelay && lastIRTime != 0) {
                  stopMotor();
                  lastIRValue = 0;
                  lastIRTime = 0;
              }
              return;
          }
      }

      switch(value) {
        case IR_BUTTON_C: 
          if (millis() - lastButtonCTime > buttonDelay) {
            lastButtonCTime = millis();
            toggleState();
          }
          break;
      
        default: break;
      }
    }

    void update() override {
      stopMotor();
      remoteIR(remote_ir_receiver);
    }
};

class DustState : public State {
  public:
    DustState() { _name = "DustState"; }
  
    void initialise() override {
      Serial.println(F("initialise EndState behaviour"));
      setLEDcolour(150,150,0); // Yellow
      stopMotor(); // not needed, just in case
      playTone();
    }

    void playTone() override {
      buzzer.tone(1047, 200); // C6
      delay(100);
      buzzer.noTone();
    }

    void update() override {
      remoteIR(remote_ir_receiver); 
      playTone();
    }
};


// my State var
State* currentState = nullptr; // so we initialise the var first... needs to be dynamic type kinda...
int State::cornersSaved = 0; // static vars must be declared once
Position State::savedCorners[4];
Position State::currentPosition = {0, 0, false};

PauseState pauseState;
NavigateOrigin navigateOrigin;
SnakeState snakeState;
StartState startState;
EndState endState;
DustState dustState;


// Button
#define buttonSwitch A7 // 1023 is OFF, 0 is ON
const unsigned long HOLD_DURATION = 3000;  // Hold duration in milliseconds

bool isButtonPressed() {
  const int buttonDebounce = 50; // arbitrary value of 100
  int buttonStatus = analogRead(buttonSwitch); // 0 is ON, !0 is OFF
  if (buttonStatus <= buttonDebounce) { 
    return true;
  }
  return false;
}

int checkButton() {
  static bool buttonActive = false;
  static unsigned long buttonPressTime = 0;
  
  if (isButtonPressed()) {
    if (!buttonActive) {
      buttonPressTime = millis();
      buttonActive = true;
    } else if (millis() - buttonPressTime >= HOLD_DURATION) {
      buttonActive = false;
      return 2;
    }
  } else if (buttonActive) {
    buttonActive = false;
    if (millis() - buttonPressTime >= HOLD_DURATION) {
      return 2; // Long hold (shouldn't reach here but just in case)
    } else if (millis() - buttonPressTime >= 20) { return 1; } // Short press
  }
  
  return 0; // No action
}


// ======================================== Main Program Code ========================================


void setup() {
  pinMode(buttonSwitch, INPUT); // button
  
  pinMode(pressureSensorPin, INPUT); // pressure sensor
  pinMode(irLEDPin, INPUT); // Data pin 10 if irStatusPin is 10 on Port 2

  remote_ir_receiver.begin();
  Serial.begin(9600);

  // Start in pause state
  currentState = &pauseState;
  currentState->initialise();  
}

void loop() {
  int buttonResult = checkButton();

  if (buttonResult == 1) {
    // Short press - toggle state
    toggleState();
  } else if (buttonResult == 2) {
    // Long hold - special action
    toggleStateHold();
  }

  if (currentState) { // if currentState exists, not Null
    currentState->update();
  }
}

void toggleState() {
  if (currentState == &pauseState) {
    currentState->exit();
    if (currentState->getCornersCount() >= 4) {
      currentState = &navigateOrigin;
    } else {
      // go into start state
      currentState = &startState;
    }
    currentState->initialise();
  } else if (currentState == &navigateOrigin) {
    currentState->exit();
    currentState = &snakeState;
    currentState->initialise();
  } else if (currentState == &snakeState) {
    currentState->exit();
    currentState = &pauseState;
    currentState->initialise();
  } else if (currentState == &startState) {
    currentState->exit();
    currentState = &pauseState;
    currentState->initialise();
  } else if (currentState == &endState) {
    currentState->exit();
    currentState = &pauseState;
    currentState->initialise();
  } else if (currentState == &dustState) {
    currentState->exit();
    currentState = &startState;
    currentState->initialise();
  }
}

void toggleStateHold() {
  if (currentState == &dustState) {
    currentState->exit();
    currentState = &startState;
    currentState->initialise();
  }

  currentState->exit();
  currentState = &endState;
  currentState->initialise();
}

void toggleStateDust() {
  currentState->exit();
  currentState = &dustState;
  currentState->initialise();
}
