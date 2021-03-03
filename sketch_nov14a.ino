#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Rotary.h>
#include <PID_v1.h>

#define TACHO 3            // tacho motorun arkasındakı sesor ucunun bırı gnd bırı 3 nolu pını 
#define DETECT 2           // alternatıf akım 0 noktasını tespıt edebılmek ıcın 2 nolu pıne baglandı 
#define GATE A3            // Triyak tetik ucu 
#define RANGE1 9           // 1 devir ayarı için pin girişi 
#define RANGE2 10          // 2 devir ayarı için pin girişi 
#define BUTTON 16          // rottary encoder button pini 
#define RELAY 5            // role cıkısı 
#define PULSE 2            // triak modeline göre tetikleme uygulanır her 1 deger 16 mikro saniyedir 
#define TACHOPULSES 8      // tako jenaratörünün 1 turda verdiği pals degeridir 

unsigned int RPM;                   // rpm veri tutucu 
unsigned int count;                 // tacho sayaci 
unsigned int lastcount = 0;         // eski ve yeni tacho sayaci 
unsigned long lastcounttime = 0;
unsigned long lastflash;
unsigned long lastpiddelay = 0;
unsigned long previousMillis = 0;
unsigned long lastDebounceTime = 0;

const int sampleRate = 1;           // PID döngümüzün ne kadar hızlı olduğunu belirleyen değişken
const int rpmcorrection = 86;       // istenen RPM'ye eşit gerçek RPM'ye sahip olmak için bu parametreyi eklemek zorunda kaldım
const int lcdinterval = 2000;       // milisaniye cinsinden lcd yenileme aralığı
const int protection = 2000;        // gerçek devir, değer tarafından istenen değeri aştığında koruma devreye girer
const int debounceDelay = 50;       // geri tepme süresi; çıktı titriyorsa artırın
const int minoutputlimit = 80;      // PID çıkışı sınırı
const int maxoutputlimit = 540;     // PID çıkışı sınırı
const int mindimminglimit = 80;     // triyak tetikleme önceki en kısa gecikme
const int maxdimminglimit = 520;    // 60Hz için 520 olacak
const int minrpmR1 = 300;           // 1 aralığının min RPM'i
const int maxrpmR1 = 1500;          // 1 aralığının max RPM'i
const int minrpmR2 = 1500;          // 2 aralığının min RPM'i
const int maxrpmR2 = 6500;          // 2 aralığının max RPM'i
const int risetime = 100;           // Mikrosaniye cinsinden RPM yükselme zamanı gecikmesi (yükselme süresi x RPM)


int dimming = 540;                  
int counterR1;                      
int counterR2;                      
int desiredRPM;
int tempcounter = 100;

byte range;
byte lastRangeState = 0;
byte relayState = LOW;              
byte buttonState;                  
byte lastButtonState = HIGH;        

bool loopflag = false;              
bool startflag = false;             
bool runflag = false;               

double Setpoint, Input, Output;      
double sKp = 0.1, sKi = 0.2, sKd = 0; 
double rKp = 0.25, rKi = 1, rKd = 0;  

LiquidCrystal_I2C lcd(0x3F, 16, 2);   
Rotary r = Rotary(15, 14);           
PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT); 

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.home();
  lcd.print("YILMAZ");
  pinMode(BUTTON, INPUT);             
  pinMode(RELAY, OUTPUT);            
  pinMode(DETECT, INPUT);            
  pinMode(GATE, OUTPUT);              
  pinMode(TACHO, INPUT);             
  pinMode(RANGE1, INPUT);             
  pinMode(RANGE2, INPUT);             
  digitalWrite(BUTTON, HIGH);         
  digitalWrite(RANGE1, HIGH);         
  digitalWrite(RANGE2, HIGH);         
  digitalWrite(RELAY, relayState);    

  counterR1 = minrpmR1;               
  counterR2 = minrpmR2;              
  Input = 200;                        
  Setpoint = 200;                     

 
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minoutputlimit, maxoutputlimit);
  myPID.SetSampleTime(sampleRate);    


  OCR1A = 100;                        
  TIMSK1 = 0x03;                      
  TCCR1A = 0x00;                      
  TCCR1B = 0x00;                      

 
  attachInterrupt(0, zeroCrossingInterrupt, RISING);
  attachInterrupt(1, tacho, FALLING);



//  lcd.init();      
//  lcd.backlight();  

  // check the RPM range state at startup and display it
  int rangeOne = digitalRead(RANGE1);
  int rangeTwo = digitalRead(RANGE2);

  if (rangeOne == 1 && rangeTwo == 1) {
    range = 0;
    range0();
  }
  if (rangeOne == 0 && rangeTwo == 1) {
    range = 1;
    RPMrange1();
  }
  if (rangeOne == 1 && rangeTwo == 0) {
    range = 2;
    RPMrange2();
  }
}

// 0 geçis noktası
void zeroCrossingInterrupt() { // zero cross detect
  TCCR1B = 0x04;               // start timer with divide by 256 input
  TCNT1 = 0;                   // reset timer - count from zero
  OCR1A = dimming;             // set the compare register brightness desired.
}


ISR(TIMER1_COMPA_vect) {       // comparator match
  if (startflag == true) {     // flag for start up delay
    digitalWrite(GATE, HIGH);  // set TRIAC gate to high
    TCNT1 = 65536 - PULSE;     // trigger pulse width
  }
}

ISR(TIMER1_OVF_vect) {         // timer1 overflow
  digitalWrite(GATE, LOW);     // turn off TRIAC gate
  TCCR1B = 0x00;               // disable timer stops unintended triggers
}

// RPM counting routine
void tacho() {
  count++;
  unsigned long tachotime = micros() - lastflash;
  float time_in_sec  = ((float)tachotime + rpmcorrection) / 1000000;
  float prerpm = 60 / time_in_sec;
  RPM = prerpm / TACHOPULSES;
  lastflash = micros();
}

void loop() {

  // check the RPM range switch state
  int rangeOne = digitalRead(RANGE1);
  int rangeTwo = digitalRead(RANGE2);

  if (rangeOne == 1 && rangeTwo == 1) {
    range = 0;
  }
  if (rangeOne == 0 && rangeTwo == 1) {
    range = 1;
    desiredRPM = counterR1;
  }
  if (rangeOne == 1 && rangeTwo == 0) {
    range = 2;
    desiredRPM = counterR2;
  }

  // check the RPM range switch state changes
  if (range != lastRangeState) {
    if (range == 0) {
      range0();
      runflag = false;
      startflag = false;            
      delay (300);                  
      digitalWrite(RELAY, LOW);
      relayState = LOW;
    }
    if (range == 1) {
      RPMrange1();
    }
    if (range == 2) {
      RPMrange2();
    }
    if (relayState == LOW && range != 0) {
      motorStateStop();
    }
  }
  lastRangeState = range;

  // check the start / stop button state
  if (range != 0) {
    int reading = digitalRead(BUTTON); // read the state of the switch into a local variable:
    if (reading != lastButtonState) {  // If the switch changed, due to noise or pressing
      lastDebounceTime = millis();     // reset the debouncing timer
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {     // if the button state has changed:
        buttonState = reading;
        if (buttonState == LOW) {       // only toggle the relay if the new button state is LOW
          relayState = !relayState;
          if (relayState == HIGH) {
            loopflag = true;
            digitalWrite(RELAY, relayState); // set the Relay:
            delay (300);                     // delay to prevent sparks on relay contacts
            startflag = true;                // flag to start motor
          }
          if (relayState == LOW) {
            Setpoint = 200;
            Input = 200;
            runflag = false;
            startflag = false;
            delay (300);                     // delay to prevent sparks on relay contacts
            digitalWrite(RELAY, relayState); // set the Relay:
            motorStateStop();
          }
        }
      }
    }
    lastButtonState = reading;            // save the reading. Next time through the loop, it'll be the lastButtonState:
  }

  //rotarry encoder process
  unsigned char result = r.process();
  if (range == 1 && result == DIR_CW) {
    if (counterR1 >= 500)
    {
      counterR1 += 50;
    }
    else counterR1 += 20;
    if (counterR1 >= maxrpmR1) {
      counterR1 = maxrpmR1;
    }
    RPMrange1();
  }

  else if (range == 1 && result == DIR_CCW) {
    if (counterR1 <= 500)
    {
      counterR1 -= 20;
    }
    else counterR1 -= 50;
    if (counterR1 <= minrpmR1) {
      counterR1 = minrpmR1;
    }
    RPMrange1();
  }

  if (range == 2 && result == DIR_CW) {
    counterR2 += 100;
    if (counterR2 >= maxrpmR2) {
      counterR2 = maxrpmR2;
    }
    RPMrange2();
  }
  else if (range == 2 && result == DIR_CCW) {
    counterR2 -= 100;
    if (counterR2 <= minrpmR2) {
      counterR2 = minrpmR2;
    }
    RPMrange2();
  }

  //soft start
  if (loopflag == true) {
    myPID.SetTunings(sKp, sKi, sKd);        // Set the PID gain constants and start
    int i = (desiredRPM - tempcounter);
    for (int j = 1; j <= i; j++) {
      Input = RPM;
      Setpoint = tempcounter;
      myPID.Compute();
      dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // inverse the output
      dimming = constrain(dimming, mindimminglimit, maxdimminglimit);     // check that dimming is in 20-625 range
      tempcounter++;
      delayMicroseconds (risetime);
    }
    if (tempcounter >= desiredRPM) {
      lastcounttime = millis();
      lastpiddelay = millis();
      loopflag = false;
      runflag = true;
      tempcounter = 100;
    }
  }

  // normal motor running state
  if (relayState == HIGH && loopflag == false) {
    unsigned long piddelay = millis();

    if ((piddelay - lastpiddelay) > 1000) {     // delay to switch PID values. Prevents hard start
      myPID.SetTunings(rKp, rKi, rKd);          // Set the PID gain constants and start
      lastpiddelay = millis();
    }

    Input = RPM;
    Setpoint = desiredRPM;
    myPID.Compute();
    dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // reverse the output
    dimming = constrain(dimming, mindimminglimit, maxdimminglimit);     // check that dimming is in 20-625 range
  }
  // diagnose a fault and turn on protection

  unsigned long counttime = millis();
  if (counttime - lastcounttime >= 1000) {
    if (count == 0 && relayState == HIGH && runflag == true) {
      startflag = false;            // flag to turn off triac before relay turns off
      delay (300);                  // delay to prevent sparks on relay contacts
      digitalWrite(RELAY, LOW);
      relayState = LOW;
      stuckerror();
    }
    lastcount = count;
    count = 0;
    lastcounttime = millis();
  }

  //reset rpm after motor stops
  if (count == 0 && relayState == LOW) {
    RPM = 0;
  }

  // protection against high rpm. i e triac damage
  if (relayState == HIGH && RPM > desiredRPM + protection) {
    startflag = false;            // flag to turn off triac before relay turns off
    delay (300);                  // delay to prevent sparks on relay contacts
    digitalWrite(RELAY, LOW);
    relayState = LOW;
    exceederror();
  }
  // real RPM display
  if (relayState == HIGH && range != 0) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= lcdinterval) {
      previousMillis = currentMillis;
      int rpmdisplay = RPM;
      lcd.setCursor(0, 1);
      lcd.print("MTR RPM :   ");
      if (rpmdisplay >= 1000) {
        lcd.setCursor(12, 1);
        lcd.print(rpmdisplay);
      }
      else if (RPM < 1000) {
        lcd.setCursor(12, 1);
        lcd.print(" ");
        lcd.setCursor(13, 1);
        lcd.print(rpmdisplay);
      }
    }
  }
}

void range0() {
  lcd.setCursor(0, 0);
  lcd.print("  DEVIR  SECIN  ");
  lcd.setCursor(0, 1);
  lcd.print("  1. YADA 2. :) ");
}

void motorStateStop() {
  lcd.setCursor(0, 1);
  lcd.print (" BASLATA BASIN! ");

}

void RPMrange1() {
  lcd.setCursor(0, 0);
  lcd.print("1. DEVIR  : ");
  if (counterR1 >= 1000) {
    lcd.setCursor(12, 0);
    lcd.print(counterR1);
  }
  else {
    lcd.setCursor(12, 0);
    lcd.print(" ");
    lcd.setCursor(13, 0);
    lcd.print(counterR1);
  }
}

void RPMrange2() {
  lcd.setCursor(0, 0);
  lcd.print("2. DEVIR  : ");
  lcd.setCursor(12, 0);
  lcd.print(counterR2);
}

void exceederror() {
  lcd.clear();
  while (1) {
    lcd.setCursor(5, 0);
    lcd.print("DURDUR!");
    lcd.setCursor(2, 1);
    lcd.print("TRIAK HASARI");
  }
}

void stuckerror() {
  lcd.clear();
  while (1) {
    lcd.setCursor(5, 0);
    lcd.print("DURDUR!");
    lcd.setCursor(2, 1);
    lcd.print("MOTOR SKSMS!");
  }
}
