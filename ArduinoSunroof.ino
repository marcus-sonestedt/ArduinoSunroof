const auto GoOutPin = 2;
const auto GoInPin = 4;

const auto LimOutPin = 6;
const auto LimInPin = 7;

const auto MotorOutPlusPin = 8;
const auto MotorInPlusPin = 9;
const auto MotorOutNegPin = 10;
const auto MotorInNegPin = 11;

const auto ExtLedPin = 3;
const auto LedPin = 13;

void setPwmFrequency(int pin, int divider);

void setup() {
  // put your setup code here, to run once:

  pinMode(GoOutPin, INPUT_PULLUP);
  pinMode(GoInPin, INPUT_PULLUP);

  pinMode(MotorOutPlusPin, OUTPUT);
  pinMode(MotorInPlusPin, OUTPUT);
  pinMode(MotorOutNegPin, OUTPUT);
  pinMode(MotorInNegPin, OUTPUT);

  setPwmFrequency(MotorOutNegPin, 1);
  setPwmFrequency(MotorInNegPin, 1);

  pinMode(LedPin, OUTPUT);
  pinMode(ExtLedPin, OUTPUT);

  Serial.begin(115200);
}

bool prevOut = LOW;
bool prevIn = LOW;
auto blinkHz = 0.1f;

class DebouncedInput
{
 public:
  DebouncedInput(int pin) : _pin(pin), _lastTime(millis()) {}

  operator bool()
  {
    auto value = digitalRead(_pin) == HIGH;

    if (value != _lastValue) {
      _lastValue = value;
      _lastTime = millis();      
    } else if (millis() - _lastTime > 50)  {
      _actualValue = _lastValue;      
    }

    return _actualValue;
  }

  int _pin;
  int _lastTime;
  bool _lastValue = false;
  bool _actualValue = false;
};

DebouncedInput goInDebouncer(GoInPin), goOutDebouncer(GoOutPin);

int pwm = 0;
int targetPwm = 0;

void loop() {
  // put your main code here, to run repeatedly:

  auto goOut = !goInDebouncer;
  auto goIn = !goOutDebouncer;

  auto limitOut = digitalRead(LimOutPin) == HIGH;
  auto limitIn = digitalRead(LimInPin) == HIGH;

  goOut &= !limitOut;
  goIn &= !limitIn;

  if (prevOut != goOut || prevIn != goIn) {
    if (goOut) {       
      blinkHz = 1.0f;
    } else if (goIn) {
      blinkHz = 3.0f;
    } else {
      blinkHz = 0.1f;
    }
    
    prevOut = goOut;
    prevIn = goIn;
  }

  if (limitIn || limitOut)
    blinkHz = 10.0f;

  if (goOut == goIn) {
    targetPwm = 0;  
  } else if (goOut) {
    targetPwm = 255;
  } else if (goIn) {
    targetPwm = -255;
  }

  int actualPwm = abs(pow(pwm, 3) / pow(256, 2));

  if (pwm == 0)
  {
    digitalWrite(MotorOutPlusPin, LOW);
    digitalWrite(MotorInPlusPin, LOW);
    digitalWrite(MotorOutNegPin, LOW);
    digitalWrite(MotorInNegPin, LOW);
  } 
  else if (pwm > 0) 
  {
    digitalWrite(MotorOutPlusPin, HIGH);
    digitalWrite(MotorInPlusPin, LOW);
    digitalWrite(MotorOutNegPin, LOW);
    analogWrite(MotorInNegPin, actualPwm);     
  } 
  else if (pwm < 0)
  {
    digitalWrite(MotorOutPlusPin, LOW);
    digitalWrite(MotorInPlusPin, HIGH);
    analogWrite(MotorOutNegPin, actualPwm);
    digitalWrite(MotorInNegPin, LOW);         
  }

  if (pwm != targetPwm)
  {
    if (pwm < targetPwm)
      pwm++;
    else
      pwm--;
  }

  auto blinkValue = int((cos(micros() * 1e-6f * blinkHz * 3.14f * 2) + 1) * 128);
  //digitalWrite(LedPin, blinkValue > 200 ? HIGH : LOW);
  analogWrite(ExtLedPin, blinkValue * blinkValue / (blinkHz > 1 ? 255.0f : 128.0f));

  delay(10);
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = (TCCR0B & 0b11111000) | mode;
    } else {
      TCCR1B = (TCCR1B & 0b11111000) | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = (TCCR2B & 0b11111000) | mode;
  }
}
