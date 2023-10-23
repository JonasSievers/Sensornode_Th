/*

 */

#include <LwIP.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <STM32Ethernet.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <BH1750.h>


// Enter an IP address for your controller below.
// The IP address will be dependent on your local network:

IPAddress ip(172,22,167,221);  // local sensor 1 is 172.22.167.221 server adress is 172.22.167.220
IPAddress server(172,22,167,220); // Raspberry Pi Server

byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
EthernetClient ethClient;
PubSubClient client(ethClient);
Adafruit_BMP280 bmp; // use I2C interface
BH1750 GY30(0x23);


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // start the Ethernet connection and the server:
  Ethernet.begin(ip);
  //Serial.print(bmp.begin());
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  if (GY30.begin()) {
    Serial.println(F("GY30 Advanced begin"));
  } else {
    Serial.println(F("Error initialising GY30"));
  }
  Serial.print("SK1 is at: ");
  Serial.println(Ethernet.localIP());





  client.setServer(server, 1883);
  client.setCallback(callback);

  Ethernet.begin(mac, ip);
  // Allow the hardware to sort itself out
  delay(1500);
}





void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("tSk1","hello world");
      // ... and resubscribe
      client.subscribe("inTopicYannick");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

float arrayTemperature[15] = {0};
float arrayLight[15] = {0};
unsigned long lasTime =0;
unsigned int arrayIndex = 0;
float averageTemperature=0;
float averageLight=0;
char msg_out[20];
void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if (uint32_t(millis()-lasTime)>1000){
  
    lasTime = millis();

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    arrayTemperature[arrayIndex] = bmp.readTemperature();

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    float lux = GY30.readLightLevel();
    arrayLight[arrayIndex] =lux;
    arrayIndex ++;
    client.publish("tSk1","iteration");
    
    Serial.print(F("Light = "));
    Serial.print(lux);
    Serial.println(" lux");

    int sound = analogRead(A1);
    // Print the sensor value to the serial monitor
    Serial.print("Analog Value Sound: ");
    Serial.println(sound);


    if (arrayIndex>=15){
      averageTemperature = 0;
      averageLight = 0;
      arrayIndex = 0;
      for(int i=0; i<15; i++){
        averageTemperature += arrayTemperature[i];
        averageLight += arrayLight[i];
      }
      averageTemperature = averageTemperature/15;
      averageLight = averageLight/15;
      Serial.print(F("----- AVERAGE Temperature = "));
      Serial.print(averageTemperature);
      Serial.println(" *C");
      Serial.print(F("----- AVERAGE Light = "));
      Serial.print(averageLight);
      Serial.println(" lux");
      
      // sprintf(msg_out, "%f",averageTemperature);
      dtostrf(averageTemperature, 9, 6, msg_out);
      client.publish("tSk1","average Temp ist:");
      client.publish("tSk1",msg_out);

    }

    }

}