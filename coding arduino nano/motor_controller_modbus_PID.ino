#include <ModbusRtu.h>
#include <SoftwareSerial.h>
//#include  <TimerOne.h>     

#define MAX485_DE 8  // Pin untuk DE
#define MAX485_RE_NEG 7  // Pin untuk RE

#define ID   1 // Slave ID
SoftwareSerial mySerial(5, 6);  // RX, TX
Modbus slave(ID, mySerial, 0);

uint16_t au16data[10]; // Array untuk data


#include <RBDdimmer.h>//

//#define USE_SERIAL  SerialUSB //Serial for boards whith USB serial port
#define USE_SERIAL  Serial
#define outputPin  12 
#define zerocross  2 // for boards with CHANGEBLE input pins

//dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
dimmerLamp dimmer(outputPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero

int outVal = 0;

int alpha_buff;
int alpha;
int count;

unsigned long timer;
unsigned long timer_prev;

unsigned long serial_timer;
unsigned long serial_timer_prev;

unsigned long rpm_timer;
unsigned long rpm_timer_prev;

float counter;
int sensor_state;
int sensor_state_prev;
float rpm_filtered;

float sp;
float kp;
float ki;
float kd;

float pid;
float e;
float e_prev;
float p;
float i;
float i_prev;
float d;
float d_prev;

int status;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float p_control(float kp, float error) {
    float p_control_val = kp*error;
    return p_control_val;
}


float i_control(float ki, float error, float error_prev, float T, float i_control_prev) {
    float i_control_val = ki * T/2 * (error + error_prev) + i_control_prev;
    
    if (i_control_val > 1000){
      i_control_val = 1000;
    } 
    if (i_control_val < 0) {
      i_control_val = 0;
    }
    
    return i_control_val;
}

float d_control(float kd, float error, float error_prev, float T, float d_control_prev){
    float d_control_val;
    
    if (abs(error - error_prev) > 0.5){
      d_control_val = kd * 2/T * (error - error_prev) - d_control_prev;
    } else {
      d_control_val = 0;
    }
    
    if (d_control_val > 1000){
      d_control_val = 1000;
    } 
    if (d_control_val < 0) {
      d_control_val = 0;
    }


    return d_control_val;
}

// Fungsi untuk mengatur pin sebelum pengiriman
void preTransmission() {
  digitalWrite(MAX485_RE_NEG, HIGH);  // Disable receiving
  digitalWrite(MAX485_DE, HIGH);      // Enable transmission
  delay(5); // Tambahkan sedikit delay
}

// Fungsi untuk mengatur pin setelah pengiriman
void postTransmission() {
  digitalWrite(MAX485_RE_NEG, LOW);   // Enable receiving
  digitalWrite(MAX485_DE, LOW);       // Disable transmission
  delay(5); // Tambahkan sedikit delay setelah transmission
}

void setup() {

  dimmer.begin(NORMAL_MODE, ON); 
  
  pinMode(3, INPUT);
  // Inisialisasi pin
  pinMode(MAX485_RE_NEG, OUTPUT);

  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, LOW);  // Set to receive mode
  digitalWrite(MAX485_DE, LOW);      // Set to receive mode

  mySerial.begin(38400);  // Inisialisasi SoftwareSerial
  Serial.begin(9600);    // Inisialisasi Serial
  slave.start();         // Memulai slave
}



void loop() {

  timer = millis() - timer_prev;

  if (timer > 50){
  
  // Panggil preTransmission sebelum polling data
  preTransmission();

  // Polling data dari master
  slave.poll(au16data, 10); // Poll data, ambil 10 register

  // Panggil postTransmission setelah polling
  postTransmission();

  sp = float(float(au16data[0]));
  kp = float(float(au16data[2])/100);
  ki = float(float(au16data[3])/100);
  kd = float(float(au16data[4])/100);
  status = au16data[5];


  if (alpha_buff < 35){
    dimmer.setState(OFF);
  } else {
    dimmer.setState(ON);
    dimmer.setPower(alpha_buff);  
  }
   
  timer_prev = millis();
  }

  sensor_state = digitalRead(3);
  if (sensor_state != sensor_state_prev){
    counter++;
  }

  rpm_timer = millis() - rpm_timer_prev;
  if (rpm_timer > 50){
    e = sp - rpm_filtered;
    rpm_filtered = (0.95 * rpm_filtered) + (0.05 * counter * 10);
    
    
    if(status == 1){

    p = float(p_control(kp, e));
    i = float(i_control(ki, e, e_prev, 0.05, i_prev));
    d = float(d_control(kd, e, e_prev, 0.05, d_prev));

    pid = float(p + i + d);

    if (pid > 1000){
      pid = 1000;
    } 
    if (pid < 0) {
      pid = 0;
    }
    alpha_buff = mapFloat(pid, 0, 1000, 35, 45);

    }

    if (status == 0){
      p = 0;
      i = 0;
      d = 0;
      pid = 0;
      alpha_buff = 0;
    }

    e_prev = e;
    i_prev = i;
    d_prev = d;
    counter = 0;
    rpm_timer_prev = millis();

  }

  serial_timer = millis() - serial_timer_prev;
  
  if (serial_timer > 500){
    au16data[1] = int(rpm_filtered);
    au16data[6] = int(p);
    au16data[7] = int(i);
    au16data[8] = int(d);
    au16data[9] = int(pid);

    Serial.print(sp);
    Serial.print(",");
    Serial.println(rpm_filtered);
    /*
    Serial.print("setpoint : ");
    Serial.print(sp);
    
    Serial.print(" p : ");
    Serial.print(p);

    Serial.print(" i : ");
    Serial.print(i);

    Serial.print(" d : ");
    Serial.print(d);
    
    Serial.print(" pid : ");
    Serial.print(pid);

    Serial.print(" pv : ");
    Serial.print(rpm_filtered);

    Serial.print(" stat : ");
    Serial.print(status);

    Serial.println();
    */
    
    serial_timer_prev = millis();
  }
    


  sensor_state_prev = sensor_state;

  //delay(50);  // Delay sebelum loop berikutnya
}
