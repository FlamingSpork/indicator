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
#define LOC     17 // _probably_ LOC of LOC/REMOTE
#define REMOTE  18 // _probably_ REMOTE of LOC/REMOTE
#define B_DEPART 21 // ????
#define ATO_MAN 22  // ???? very wrong

// arduino PWM pin
#define AMMETER 5

// lookup table based on the regression we found on the PWM values with a bit of manual adjustment
const int speedometer_values[] = {24, 25, 26, 26, 27, 28, 29, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 45, 46, 47, 48, 50, 51, 53, 54, 56, 57, 59, 61, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 85, 87, 89, 92, 94, 97, 100, 103, 106, 109, 112, 115, 118, 121, 125, 128, 132, 136, 139, 143, 147, 152, 156, 160, 165, 169, 174, 179, 184, 186, 190, 194, 198, 203, 207, 212};

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
  analogWrite(AMMETER, 50);
  val = 0;

  Wire.begin();
  Serial.print("io0: ");
  Serial.println(io0.begin(0x3E));
  Serial.print("io1: ");
  Serial.println(io1.begin(0x3F));

  io_mode(LED_0,  OUTPUT);
  io_mode(LED_10, OUTPUT);
  io_mode(LED_15, OUTPUT);
  io_mode(LED_18, OUTPUT);
  io_mode(LED_25, OUTPUT);
  io_mode(LED_40, OUTPUT);
  io_mode(LED_50, OUTPUT);
  io_mode(LED_55, OUTPUT);
  io_mode(LED_60, OUTPUT);
  io_mode(BEEPER, OUTPUT);
  io_mode(STOP, OUTPUT);
  io_mode(YARD_10, OUTPUT);
  io_mode(MAN_RL, OUTPUT);
  io_mode(UMAN_RL, OUTPUT);
  io_mode(BYPASS, OUTPUT);
  io_mode(W_40, OUTPUT);
  io_mode(LW_40, OUTPUT);
  io_mode(G_ATO, OUTPUT);
  io_mode(Y_MAN, OUTPUT);
  io_mode(LOC, OUTPUT);
  io_mode(REMOTE, OUTPUT);
  io_mode(B_DEPART, INPUT);
  io_mode(ATO_MAN, INPUT);

  io_write(LED_0,  LOW);
  io_write(LED_10, LOW);
  io_write(LED_15, LOW);
  io_write(LED_18, LOW);
  io_write(LED_25, LOW);
  io_write(LED_40, LOW);
  io_write(LED_50, LOW);
  io_write(LED_55, LOW);
  io_write(LED_60, LOW);
  io_write(BEEPER, LOW);
  io_write(STOP, LOW);
  io_write(YARD_10, LOW);
  io_write(MAN_RL, LOW);
  io_write(UMAN_RL, LOW);
  io_write(BYPASS, LOW);
  io_write(W_40, LOW);
  io_write(LW_40, LOW);
  io_write(G_ATO, LOW);
  io_write(Y_MAN, LOW);
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

void loop() {
  // put your main code here, to run repeatedly:
  /*if(digitalRead(4) == HIGH) {
    delay(30);
    state = !state;
    digitalWrite(2, state? HIGH:LOW);
    digitalWrite(3, state? LOW:HIGH);
    while(digitalRead(4) == HIGH) {}
  }
  delay(30);*/

  /*
  io.digitalWrite(LED_0,  HIGH);
  io.digitalWrite(LED_10, HIGH);
  io.digitalWrite(LED_15, HIGH);
  io.digitalWrite(LED_18, HIGH);
  io.digitalWrite(LED_25, HIGH);
  io.digitalWrite(LED_40, HIGH);
  io.digitalWrite(LED_50, HIGH);
  io.digitalWrite(LED_55, HIGH);
  io.digitalWrite(LED_60, HIGH);
  */

  s = Serial.readStringUntil('\n');
  if(s.toInt() > 0) {
    io_write(s.toInt(), HIGH);
    delay(5000);
    io_write(s.toInt(), LOW);
  }
/*  for(i = 20; i < 180;) {
    analogWrite(AMMETER, i);
    delay(500);
    if(i > 120) { i += 4; }
    else if(i > 60) { i += 2; }
    else { i += 1; }
  }*/

  //speedometer_sweep();
}