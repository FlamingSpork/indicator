#include <Wire.h> 
#include <SparkFunSX1509.h>
SX1509 io0, io1;

bool state;
int val;
int i;
String s;

#define LED_0  9  // io0 pin 9
#define LED_10 8
#define LED_15 19 // io1 pin 3
#define LED_18 7
#define LED_25 6
#define LED_40 5
#define LED_50 4
#define LED_55 3
#define LED_60 2
#define BEEPER 20 // io1 pin 4
#define STOP   1
#define YARD_10 0
#define MAN_RL  11
#define UMAN_RL 12 // blank underneath MAN/RL
#define BYPASS  15
#define W_40    14
#define LW_40   13 // blank left of W/40
#define G_ATO   12
#define Y_MAN   16 // yellow MAN of MAN/ATO; io1 pin 0
#define LOC     17 // _probably_ LOC of LOC/REMOTE (burnt out)
#define REMOTE  18 // REMOTE of LOC/REMOTE
#define B_DEPART 21
#define ATO_MAN 22

// arduino PWM pin
#define AMMETER 5

// lookup table based on the regression we found on the PWM values with a bit of manual adjustment
const int speedometer_values[] = {24, 25, 26, 26, 27, 28, 29, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 45, 46, 47, 48, 50, 51, 53, 54, 56, 57, 59, 61, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 85, 87, 89, 92, 94, 97, 100, 103, 106, 109, 112, 115, 118, 121, 125, 128, 132, 136, 139, 143, 147, 152, 156, 160, 165, 169, 174, 179, 184, 189, 195, 200, 206, 212, 218, 224};

const int n_pins = 20;
const int pins[] = {
  STOP, LED_0, LED_10, LED_15, LED_18, LED_25, LED_40, LED_50, LED_55, LED_60, YARD_10, MAN_RL, UMAN_RL, BYPASS, LW_40, W_40, G_ATO, Y_MAN, LOC, REMOTE
};

const int n_ipins = 2;
const int ipins[] = { B_DEPART, ATO_MAN };

bool output_states[n_pins];

int mph_to_pwm(int mph) {
  if(mph > 80) {
    return speedometer_values[80]; // clamp to max
  }
  if(mph < 0) {
    return 0;
  }
  return speedometer_values[mph];
}

void io_write(uint8_t pin, uint8_t highLow) {
  if(pin > 15) {
    io1.digitalWrite(pin - 16, highLow);
  }else {
    io0.digitalWrite(pin, highLow);
  }
}

uint8_t io_read(uint8_t pin) {
  return (pin > 15 ? io1 : io0).digitalRead(pin % 16);
}

void io_mode(uint8_t pin, uint8_t inOut) {
  if(pin > 15) {
    io1.pinMode(pin - 16, inOut);
  }else {
    io0.pinMode(pin, inOut);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(AMMETER, OUTPUT);
  analogWrite(AMMETER, 0);
  val = 0;

  Wire.begin();
  Serial.print("io0: ");
  Serial.println(io0.begin(0x3E));
  Serial.print("io1: ");
  Serial.println(io1.begin(0x3F));

  for (int i=0; i<n_pins; i++) {
    io_mode(pins[i], OUTPUT);
    io_write(pins[i], LOW);
    output_states[i] = 0;
  }

  for (int i=0; i<n_ipins; i++) {
    io_mode(ipins[i], INPUT);
  }

}

void speedometer_sweep() {
  io_write(LED_0, HIGH);
  for(i=0; i<80; i++) {
    analogWrite(AMMETER, mph_to_pwm(i));
    if(i > 10) {
      io_write(LED_10, HIGH);
    }
    if(i > 15) {
      io_write(LED_15, HIGH);
    }
    if (i > 18) {
      io_write(LED_18, HIGH);
    }
    if (i > 25) {
      io_write(LED_25, HIGH);
    }
    if (i > 40) {
      io_write(LED_40, HIGH);
    }
    if (i > 50) {
      io_write(LED_50, HIGH);
    }
    if (i > 55) {
      io_write(LED_55, HIGH);
    }
    if (i > 60) {
      io_write(LED_60, HIGH);
    }
    delay(300);
  }
  //io.digitalWrite(BEEPER, HIGH);
  delay(50);
  io_write(BEEPER, LOW); //very important to have it on only for a short time
  delay(1000);

  io_write(LED_0,  LOW);
  io_write(LED_10, LOW);
  io_write(LED_15, LOW);
  io_write(LED_18, LOW);
  io_write(LED_25, LOW);
  io_write(LED_40, LOW);
  io_write(LED_50, LOW);
  io_write(LED_55, LOW);
  io_write(LED_60, LOW);
  analogWrite(AMMETER, 0);
  delay(2500);
}

void lamp_test() {
  for(int i = 0; i < n_pins; ++i) {
    io_write(pins[i], HIGH);
  }
  delay(5000);
  for(int i = 0; i < n_pins; ++i) {
    io_write(pins[i], LOW);
  }
}

void serial_lamp_test() {
  s = Serial.readStringUntil('\n');
  if(s == "all") { return lamp_test(); }
  if(s == "inp") { return inputs_test(); }
  if(s == "speed") { return speedometer_sweep(); }
  
  if(s.toInt() > 0) {
    io_write(s.toInt(), HIGH);
    delay(5000);
    io_write(s.toInt(), LOW);
  }
}

void inputs_test() {
  bool states[] = { false, false, false, false };
  int n_ipins = 4;

  Serial.write("start input test\n");
  for(int i = 0; i < 3000; ++i) {
    delay(5);
    for(int i = 0; i < n_ipins; ++i) {
      bool before = states[i];
      bool after = (io_read(ipins[i]) == HIGH);
      if(before != after) { Serial.write("pin "); Serial.write(String(i).c_str()); Serial.write(after ? " high" : " low"); Serial.write("\n"); }
      states[i] = after;
    }
  }
  Serial.write("end input test\n");
}

void interface() {
  int cmd;
  int arg;
  int resp = '.';
  if(Serial.available() >= 3) {
    cmd = Serial.read();
    arg = Serial.read();
    Serial.read();

    if(cmd == '+') {
      io_write(pins[arg], HIGH);
      output_states[arg] = 1;
    }
    else if(cmd == '-') {
      io_write(pins[arg], LOW);
      output_states[arg] = 0;
    }
    else if(cmd == '/') {
      analogWrite(AMMETER, arg);
    }
    else if(cmd == '!') {
      while(1) serial_lamp_test();
    }
    else if(cmd == '?') {
      if(io_read(ipins[arg]) == LOW) {
        resp = '1';
      }
      else {
        resp = '0';
      }
    }
    else if(cmd == '=') {
      resp = output_states[arg] ? '1':'0';
    }
    else {
      resp = '?';
    }
    
    Serial.write(resp);
  }
}

void loop() {
  interface();
}