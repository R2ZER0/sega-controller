#define LED (13)

class SegaController {
  public:
    enum SegaButtons {
      B_A = 0,
      B_START,
      B_UP,
      B_DOWN,
      B_LEFT,
      B_RIGHT,
      B_B,
      B_C,
      NUM_BUTTONS
    };

    enum SegaPins {
      PIN1_INPUT1 = 0,
      PIN2_INPUT2,
      PIN3_INPUT3,
      PIN4_INPUT4,
      // PIN5 = Vcc
      PIN6_INPUT5,
      PIN7_SELECT,
      // PIN8 = Gnd
      PIN9_INPUT6,
      NUM_SEGAPINS
    };
    
  private:  
    int pinmap[NUM_SEGAPINS];
    int pinstates[NUM_SEGAPINS];
    bool buttons[NUM_BUTTONS];
    bool buttons_prev[NUM_BUTTONS];

    int p(SegaPins pin) {
      return this->pinstates[pin];
    }

    void setb(SegaButtons b, bool v) {
      this->buttons[b] = v;
    }

    void readPins(int selectstate)
    {
      digitalWrite(this->pinmap[PIN7_SELECT], selectstate);
      for(int i = 0; i < NUM_SEGAPINS; ++i) {
        if(i == PIN7_SELECT) { continue; }
        this->pinstates[i] = digitalRead(this->pinmap[i]);
      }
    }

  public:
    void setPinMap(SegaPins pin, int realpin) {
      this->pinmap[pin] = realpin;
    }

    void initPins() {
      pinMode(this->pinmap[PIN1_INPUT1], INPUT);
      pinMode(this->pinmap[PIN2_INPUT2], INPUT);
      pinMode(this->pinmap[PIN3_INPUT3], INPUT);
      pinMode(this->pinmap[PIN4_INPUT4], INPUT);
      pinMode(this->pinmap[PIN6_INPUT5], INPUT);
      pinMode(this->pinmap[PIN9_INPUT6], INPUT);
      pinMode(this->pinmap[PIN7_SELECT], OUTPUT);
    }

    bool checkControllerExists() {
      this->readPins(LOW);
      return (!p(PIN3_INPUT3) && !p(PIN4_INPUT4)) &&
             ( p(PIN1_INPUT1) ||  p(PIN2_INPUT2));
    }

    void poll() {
      for(int i = 0; i < NUM_BUTTONS; ++i) {
        this->buttons_prev[i] = this->buttons[i];
      }
      
      this->readPins(LOW);
      setb(B_A,     !p(PIN6_INPUT5));
      setb(B_START, !p(PIN9_INPUT6));

      Serial.printf("LOW %d %d %d %d %d %d ", p(PIN1_INPUT1), p(PIN2_INPUT2), p(PIN3_INPUT3), p(PIN4_INPUT4), p(PIN6_INPUT5), p(PIN9_INPUT6));

      this->readPins(HIGH);
      setb(B_UP,    !p(PIN1_INPUT1));
      setb(B_DOWN,  !p(PIN2_INPUT2));
      setb(B_LEFT,  !p(PIN3_INPUT3));
      setb(B_RIGHT, !p(PIN4_INPUT4));
      setb(B_B,     !p(PIN6_INPUT5));
      setb(B_C,     !p(PIN9_INPUT6));
      
      Serial.printf("HIGH %d %d %d %d %d %d\n", p(PIN1_INPUT1), p(PIN2_INPUT2), p(PIN3_INPUT3), p(PIN4_INPUT4), p(PIN6_INPUT5), p(PIN9_INPUT6));
    }

    bool buttonStateChanged(SegaButtons b) {
      return this->buttons[b] != this->buttons_prev[b];
    }

    bool isButtonPressed(SegaButtons b) {
      return this->buttons[b];
    }
};

SegaController segac;

int getHatPosition(SegaController* c)
{
#define bp(x) c->isButtonPressed(x)
       if( bp(SegaController::B_UP)    && bp(SegaController::B_RIGHT) ) { return 45;  }
  else if( bp(SegaController::B_RIGHT) && bp(SegaController::B_DOWN)  ) { return 135; }
  else if( bp(SegaController::B_DOWN)  && bp(SegaController::B_LEFT)  ) { return 225; }
  else if( bp(SegaController::B_LEFT)  && bp(SegaController::B_UP)    ) { return 315; }
  else if( bp(SegaController::B_UP)                                   ) { return 0;   }
  else if( bp(SegaController::B_RIGHT)                                ) { return 90;  }
  else if( bp(SegaController::B_DOWN)                                 ) { return 180; }
  else if( bp(SegaController::B_LEFT)                                 ) { return 270; }
  else                                                                  { return -1;  }
#undef bp
}

void setup() {
  // Setup pin configuration
  pinMode(LED, OUTPUT);
  segac.setPinMap(SegaController::PIN1_INPUT1, 1);
  segac.setPinMap(SegaController::PIN2_INPUT2, 2);
  segac.setPinMap(SegaController::PIN3_INPUT3, 3);
  segac.setPinMap(SegaController::PIN4_INPUT4, 4);
  segac.setPinMap(SegaController::PIN6_INPUT5, 6);
  segac.setPinMap(SegaController::PIN7_SELECT, 7);
  segac.setPinMap(SegaController::PIN9_INPUT6, 9);
  segac.initPins();
  delayMicroseconds(10); // Give time for the pins to initialise

  // Manually send Joystick USB packets
  Joystick.useManualSend(true);
}

void loop() {
  if(!segac.checkControllerExists()) {
    delay(100);
    return;
  }

  
  bool joystickUpdate = false;
  segac.poll();
  
  if( segac.buttonStateChanged(SegaController::B_UP) ||
      segac.buttonStateChanged(SegaController::B_RIGHT) ||
      segac.buttonStateChanged(SegaController::B_DOWN) ||
      segac.buttonStateChanged(SegaController::B_LEFT)
      ) {
        Joystick.hat(getHatPosition(&segac));
        joystickUpdate = true;
      }

  static SegaController::SegaButtons pushbuttons[] = {
    SegaController::B_A,
    SegaController::B_B,
    SegaController::B_C,
    SegaController::B_START
  };

  for(unsigned int i = 0; i < sizeof(pushbuttons)/sizeof(pushbuttons[0]); ++i) {
    if(segac.buttonStateChanged(pushbuttons[i])) {
      Joystick.button(i+1, segac.isButtonPressed(pushbuttons[i]));
      joystickUpdate = true;
    }
  }

  if(joystickUpdate) {
    Joystick.send_now();
    digitalWrite(LED, HIGH);
  }
  delay(5);
  digitalWrite(LED, LOW);
}
