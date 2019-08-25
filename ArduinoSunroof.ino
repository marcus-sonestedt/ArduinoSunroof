const auto GoOutPin = 2;
const auto GoInPin = 4;

const auto LimOutPin = 5;
const auto LimInPin = 6;

const auto MotorOutPlusPin = 8;
const auto MotorInPlusPin = 9;
const auto MotorOutNegPin = 10;
const auto MotorInNegPin = 11;

const auto ExtLedPin = 3;
const auto LedPin = 13;


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

void setPwmFrequency(int pin, int divider);

// state
DebouncedInput goInDebouncer(GoInPin), goOutDebouncer(GoOutPin);
int pwm = 0;
int targetPwm = 0;
auto blinkHz = 0.1f;

void setup() {
  pinMode(GoOutPin, INPUT_PULLUP);
  pinMode(GoInPin, INPUT_PULLUP);

  pinMode(LimitOutPin, INPUT);
  pinMode(LimitInPin, INPUT);

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

void loop() {
  auto goOut = !goInDebouncer;
  auto goIn = !goOutDebouncer;

  auto limitOut = digitalRead(LimOutPin) == HIGH;
  auto limitIn = digitalRead(LimInPin) == HIGH;

  if ((limitIn || limitOut) && (goIn || goOut)) {
    blinkHz = 10.0f;
  } else if (goOut) {       
    blinkHz = 1.0f;
  } else if (goIn) {
    blinkHz = 3.0f;
  } else {
    blinkHz = 0.2f;
  } 
  
  goOut &= !limitOut;
  goIn &= !limitIn;
 
  if (goOut == goIn) {
    targetPwm = 0;  
  } else if (goOut) {
    targetPwm = 255;
  } else if (goIn) {
    targetPwm = -255;
  }

  if (pwm != targetPwm)
  {
    auto pwmDelta = (limitIn || limitOut) ? 5 : 1;
    pwm += (pwm < targetPwm) ? pwmDelta : -pwmDelta;
  }

  if (abs(pwm) < 16 && targetPwm == 0)
    pwm = 0;


  auto actualPwm = abs(pow(pwm, 3) / pow(256, 2));

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

  auto blinkValue = int((cos(micros() * 1e-6f * blinkHz * 3.14f * 2) + 1) * (blinkHz < 1 ? 25 : 100)) + 25;
  digitalWrite(LedPin, blinkValue > 200 ? HIGH : LOW);
  analogWrite(ExtLedPin, blinkValue * blinkValue / (256.0f));

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
