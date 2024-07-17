
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <WiFiS3.h>
#include <ThingSpeak.h>

// SSID + pass 
char ssid[] = "nome do Wifi";  
char password[] = "password do wifi";  

// ThingSpeak settings
unsigned long myChannelNumber = 2595837;
const char* myWriteAPIKey = "TLR4G8FKTIXNFWZO";


int status = WL_IDLE_STATUS;
WiFiClient client;  
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();


float vector_x, vector_y, vector_z, vector_sum;

int fallDetected = 0; 
int maybefallDetected = 0;
const float fallThreshold = 30;  // > is maybefallDetected

//upadte time variables ThingSpeak
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 300000;   //5 min interval for updating

// Variables for inactivity detection
const float inactivityThreshold = 16;  // threshold for inactivity 
unsigned long inactiveStartTime = 0;
const unsigned long inactivityTimeout = 60000;  //1 min inactivity

// Variables to store previous accelerometer values
float prev_vector_x = 0.0, prev_vector_y = 0.0, prev_vector_z = 0.0;


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    Serial.println("waiting for serial connection"); // waits until Serial connection is established
  }
  delay(1000);
  Serial.println("Starting Setup");
  
  //connect to Wifi
  WiFi.begin(ssid,password);

  // check WiFi module
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
  }
  Serial.println("WiFi connected");

  delay(1000);

  //initialize ThingSpeak
  ThingSpeak.begin(client);

  //initialize ADXL345
  if (!accel.begin()) {
    Serial.println("No ADXL345 detected");
    while (1);
  }
  Serial.println("ADXL345 initialized");
  delay(1000);
  accel.setRange(ADXL345_RANGE_16_G); 
  accel.setDataRate(ADXL345_DATARATE_100_HZ); //fs = 100 Hz

  sensors_event_t event;
  accel.getEvent(&event);

  prev_vector_x = event.acceleration.x;
  prev_vector_y = event.acceleration.y;
  prev_vector_z = event.acceleration.z;
}



void loop() {
  
  // Read accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);
  
  vector_x = event.acceleration.x;
  vector_y = event.acceleration.y;
  vector_z = event.acceleration.z;
  vector_sum = sqrt(sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z));
  Serial.print("X: "); Serial.print(vector_x);
  Serial.print(", Y: "); Serial.print(vector_y);
  Serial.print(", Z: "); Serial.print(vector_z);
  Serial.print(", Sum: "); Serial.println(vector_sum);


  if (vector_sum > fallThreshold) {
    maybefallDetected = 1;
    Serial.println("maybe Fall Detected");

    //start inactivity timer
    inactiveStartTime = millis();
  }
  
  //check inactivity after fall detection
  if (maybefallDetected){
    //if vector_sum exceeds the inactivity threshold -> reset the inactivity timer
    if (vector_sum >= inactivityThreshold){
      inactiveStartTime = millis();    //reset inactivity interval
      Serial.println("new fall");
    }

    //check if vector_sum is below inactivity threshold for too long
    Serial.println("inactiveStartTime" + String(inactiveStartTime));

    if (millis() - inactiveStartTime >= inactivityTimeout){
      Serial.println("inativo há mt tempo - maybe fall true");

      if (abs(vector_x - prev_vector_x) > 6 || abs(vector_y - prev_vector_y) > 6 || abs(vector_z - prev_vector_z) > 6){
        fallDetected = 1;    //fall TRUE
        Serial.println("FALL DETECTED");
        maybefallDetected = 0;   //fall FALSE
        inactiveStartTime = 0; 
 
      } else{
        maybefallDetected = 0;
      }
    }
  }
  
    
  //update Thingspeak every 5 min (updateInterval -> can vary)
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;

    // Send data to ThingSpeak
    ThingSpeak.setField(1, fallDetected);  //sens the variable fallDetected (1-> fall; 0-> no fall)

    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    Serial.println("sent to THINGSPEAK");
    
    //reset fall detection status (after updating Thingspeak)
    fallDetected = 0;

  }
}













