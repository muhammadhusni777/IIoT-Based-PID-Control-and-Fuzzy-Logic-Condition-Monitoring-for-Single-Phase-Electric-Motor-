#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Fuzzy.h>
#include <SoftwareSerial.h>

// Objek Fuzzy
Fuzzy *fuzzy = new Fuzzy();

// Set fuzzy untuk power
FuzzySet *lowPower = new FuzzySet(-10, 0, 400, 500);    // Power rendah
FuzzySet *mediumPower = new FuzzySet(290, 600, 700, 800);  // Power sedang
FuzzySet *highPower = new FuzzySet(600, 800, 1000, 1200);  // Power tinggi

// Set fuzzy untuk RPM
FuzzySet *lowRPM = new FuzzySet(-10, 0, 50, 75);   // RPM rendah
FuzzySet *mediumRPM = new FuzzySet(50, 75, 125, 300);  // RPM sedang
FuzzySet *highRPM = new FuzzySet(125, 200, 400, 400);  // RPM tinggi

// Set fuzzy untuk kualitas dinamo
FuzzySet *poor = new FuzzySet(0, 20, 20, 40);        // 
FuzzySet *average = new FuzzySet(30, 50, 50, 70);    // 
FuzzySet *good = new FuzzySet(60, 80, 100, 120);      // 


float quality;

SoftwareSerial mySerial(D5, D6); 
#include <ModbusMaster.h>
ModbusMaster node;

#define MAX485_DE 16
#define MAX485_RE_NEG 5

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

// Replace the next variables with your SSID/Password combination
const char* ssid = "Wifihusni12";//"HASAN_wifi"; // "Syergie Indo Prima"
const char* password = "12345678";//"kulonprogo";//"cahayaharapanhati";
const char* mqtt_server = "192.168.253.142";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;


// LED Pin
const int led1_pin = 2;
const int led2_pin = 4;
const int led3_pin = 5;
const int pwm_pin = 23; 

// button pin
const int button1_pin = 18;
const int button2_pin = 19;

int button1_state;
int button2_state;
int pwm_value;

int analog;
int power;

int status;
float sp;
float kp;
float ki;
float kd;
int pid;
int status_streaming;

int sensor;

void setup() {
  mySerial.begin(38400);

    // Fuzzy Input untuk power
  FuzzyInput *power = new FuzzyInput(1);
  power->addFuzzySet(lowPower);
  power->addFuzzySet(mediumPower);
  power->addFuzzySet(highPower);
  fuzzy->addFuzzyInput(power);

  // Fuzzy Input untuk RPM
  FuzzyInput *rpm = new FuzzyInput(2);
  rpm->addFuzzySet(lowRPM);
  rpm->addFuzzySet(mediumRPM);
  rpm->addFuzzySet(highRPM);
  fuzzy->addFuzzyInput(rpm);

  // Fuzzy Output untuk kualitas dinamo
  FuzzyOutput *quality = new FuzzyOutput(1);
  quality->addFuzzySet(poor);
  quality->addFuzzySet(average);
  quality->addFuzzySet(good);
  fuzzy->addFuzzyOutput(quality);

  // Aturan Fuzzy untuk menilai kualitas dinamo

  // Aturan 1: Jika power rendah dan RPM rendah, maka kualitas dinamo baik
  FuzzyRuleAntecedent *lowPowerAndLowRPM = new FuzzyRuleAntecedent();
  lowPowerAndLowRPM->joinWithAND(lowPower, lowRPM);
  FuzzyRuleConsequent *goodQuality = new FuzzyRuleConsequent();
  goodQuality->addOutput(good);
  FuzzyRule *rule1 = new FuzzyRule(1, lowPowerAndLowRPM, goodQuality);
  fuzzy->addFuzzyRule(rule1);

  // Aturan 2: Jika power sedang dan RPM sedang, maka kualitas dinamo baik
  FuzzyRuleAntecedent *mediumPowerAndMediumRPM = new FuzzyRuleAntecedent();
  mediumPowerAndMediumRPM->joinWithAND(mediumPower, mediumRPM);
  FuzzyRuleConsequent *goodQuality2 = new FuzzyRuleConsequent();
  goodQuality2->addOutput(good);
  FuzzyRule *rule2 = new FuzzyRule(2, mediumPowerAndMediumRPM, goodQuality2);
  fuzzy->addFuzzyRule(rule2);

  // Aturan 3: Jika power tinggi dan RPM tinggi, maka kualitas dinamo baik
  FuzzyRuleAntecedent *highPowerAndHighRPM = new FuzzyRuleAntecedent();
  highPowerAndHighRPM->joinWithAND(highPower, highRPM);
  FuzzyRuleConsequent *goodQuality3 = new FuzzyRuleConsequent();
  goodQuality3->addOutput(good);
  FuzzyRule *rule3 = new FuzzyRule(3, highPowerAndHighRPM, goodQuality3);
  fuzzy->addFuzzyRule(rule3);

  // Aturan 4: Jika power sedang dan RPM rendah, maka kualitas dinamo cukup
  FuzzyRuleAntecedent *mediumPowerAndLowRPM = new FuzzyRuleAntecedent();
  mediumPowerAndLowRPM->joinWithAND(mediumPower, lowRPM);
  FuzzyRuleConsequent *averageQuality = new FuzzyRuleConsequent();
  averageQuality->addOutput(average);
  FuzzyRule *rule4 = new FuzzyRule(4, mediumPowerAndLowRPM, averageQuality);
  fuzzy->addFuzzyRule(rule4);

  // Aturan 5: Jika power tinggi dan RPM sedang, maka kualitas dinamo cukup
  FuzzyRuleAntecedent *highPowerAndMediumRPM = new FuzzyRuleAntecedent();
  highPowerAndMediumRPM->joinWithAND(highPower, mediumRPM);
  FuzzyRuleConsequent *averageQuality2 = new FuzzyRuleConsequent();
  averageQuality2->addOutput(average);
  FuzzyRule *rule5 = new FuzzyRule(5, highPowerAndMediumRPM, averageQuality2);
  fuzzy->addFuzzyRule(rule5);

  
  // Aturan 6: Jika power tinggi dan RPM rendah, maka kualitas dinamo jelek
  FuzzyRuleAntecedent *highPowerAndLowRPM2 = new FuzzyRuleAntecedent();
  highPowerAndLowRPM2->joinWithAND(highPower, lowRPM);
  FuzzyRuleConsequent *poorQuality2 = new FuzzyRuleConsequent();
  poorQuality2->addOutput(poor);
  FuzzyRule *rule6 = new FuzzyRule(6, highPowerAndLowRPM2, poorQuality2);
  fuzzy->addFuzzyRule(rule6);

  //power kecil speed medium
  FuzzyRuleAntecedent *lowPowerORmediumRPM = new FuzzyRuleAntecedent();
  lowPowerAndLowRPM->joinWithOR(lowPower, mediumRPM);
  FuzzyRuleConsequent *goodQuality4 = new FuzzyRuleConsequent();
  goodQuality->addOutput(good);
  FuzzyRule *rule7 = new FuzzyRule(7, lowPowerORmediumRPM, goodQuality4);
  fuzzy->addFuzzyRule(rule7);


  FuzzyRuleAntecedent *mediumPowerORhighRPM = new FuzzyRuleAntecedent();
  lowPowerAndLowRPM->joinWithOR(mediumPower, highRPM);
  FuzzyRuleConsequent *goodQuality5 = new FuzzyRuleConsequent();
  goodQuality->addOutput(good);
  FuzzyRule *rule8 = new FuzzyRule(8, lowPowerORmediumRPM, goodQuality5);
  fuzzy->addFuzzyRule(rule8);
  
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);
  pinMode(led3_pin, OUTPUT);

  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button2_pin, INPUT_PULLUP);

  node.begin(1, mySerial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  //Serial.print("Message arrived on topic: ");
  //Serial.print(topic);
  //Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  //Serial.println();

  if (String(topic) == "power") {
    
    
     power = messageTemp.toInt();
      node.writeSingleRegister(0,power); 
     Serial.print("Changing output to ");
     Serial.println(power);
  }

  if (String(topic) == "sp") {
    sp = messageTemp.toFloat();
    uint8_t result;
    Serial.print("Changing output to ");
    
      node.writeSingleRegister(0, int(sp));
  
      if (result == node.ku8MBSuccess) {
        Serial.println("Coil written successfully");
      } else {
        Serial.print("Error: ");
        Serial.println(result, HEX);
      }
  }
  
  if (String(topic) == "kp") {
    kp = messageTemp.toFloat();
    uint8_t result;
    Serial.print("Changing output to ");
    
      node.writeSingleRegister(2, int(kp*100));
  
      if (result == node.ku8MBSuccess) {
        Serial.println("Coil written successfully");
      } else {
        Serial.print("Error: ");
        Serial.println(result, HEX);
      }
  }

  if (String(topic) == "ki") {
    ki = messageTemp.toFloat();
    uint8_t result;
    Serial.print("Changing output to ");
    
      node.writeSingleRegister(3, int(ki*100));
  
      if (result == node.ku8MBSuccess) {
        Serial.println("Coil written successfully");
      } else {
        Serial.print("Error: ");
        Serial.println(result, HEX);
      }
  }

    if (String(topic) == "kd") {
    kd = messageTemp.toFloat();
    uint8_t result;
    Serial.print("Changing output to ");
    
      node.writeSingleRegister(4, int(kd*100));
  
      if (result == node.ku8MBSuccess) {
        Serial.println("Coil written successfully");
      } else {
        Serial.print("Error: ");
        Serial.println(result, HEX);
      }
  }


  if (String(topic) == "status") {
    uint8_t result;
    //Serial.print("Changing output to ");
    
    if (messageTemp == "true"){
      status_streaming = 1;
    } else {
      status_streaming = 0;
    }
    
  
      if (result == node.ku8MBSuccess) {
        //Serial.println("Coil written successfully");
      } else {
        //Serial.print("Error: ");
        //Serial.println(result, HEX);
      }
  }

}



void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("motormotormotor")) {
      Serial.println("connected");
      client.subscribe("status");

      client.subscribe("sp");
      client.subscribe("kp");
      client.subscribe("ki");
      client.subscribe("kd");
      client.subscribe("status");

    } else {
      node.writeSingleCoil(5, int(0));
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

static char sensor_send[8];
static char pid_send[8];
static char quality_send[8];

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

 int result;
 uint16_t data[10]; // Array to store the read data
  long now = millis();
  if (now - lastMsg > 500) {

  fuzzy->setInput(1, pid);
  fuzzy->setInput(2, sensor);
  fuzzy->fuzzify();

  quality = fuzzy->defuzzify(1);

  // Read holding registers starting from address 0, read 10 registers
  result = node.readHoldingRegisters(0, 10);
 
  // Check if the read operation was successful
  if (result == node.ku8MBSuccess) {
    // Print each register value
    for (int i = 0; i < 10; i++) {
      data[i] = node.getResponseBuffer(i); // Get the value of each register
      if (i == 0){
          sp = int(data[i]);
          //Serial.print("power : ");
          //Serial.println(data[i]);
      }
      
      if (i == 1){
          sensor = int(data[i]);
          
      }
      

      if (i == 9){
        pid = int(data[i]);
        
      }

      
      
      /*
      Serial.print("Register ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(data[i]);
      */
      }
    Serial.println("sucess to read holding registers!");
    digitalWrite(led1_pin, LOW);
  } else {
    Serial.println("Failed to read holding registers!");
    digitalWrite(led1_pin, HIGH);
  }
  

  
    Serial.print("sensor : ");
    Serial.print(sensor);
    Serial.print("     ");
    Serial.print(" sp : ");
    Serial.print(sp);
    Serial.print(" pid : ");
    Serial.print(pid);
    Serial.print("     ");
    Serial.println();
    
    
        
    if (status_streaming == 1){
        node.writeSingleRegister(5, int(1));
    } else {
        node.writeSingleRegister(5, int(0));
    }


    //kirim data  
    char analog_send[8];
    dtostrf(sensor, 1, 2, sensor_send);
    dtostrf(pid, 1, 2, pid_send);
    dtostrf(quality, 1, 2, quality_send);
    
    client.publish("sensor", sensor_send);
    client.publish("pid", pid_send);
    client.publish("quality", quality_send);
    lastMsg = now;

  }
}
