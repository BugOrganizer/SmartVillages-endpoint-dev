#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ---------CREDENTIIALS-------- //
#define ssid "ls"
#define wifiPass ""
#define mqttServer "mqtt.eclipse.org"
#define mqttUsername "KangLanis"
#define mqttPassword "LanisJayaSelamanya"
#define mqttPort 1883

// -----------PIN-------------- //
#define sensor_pin_flow_1 D1
#define sensor_pin_flow_2 D2
#define sensor_pin_rain_1 A0

// ---------- TOPIC ------------//
#define mqttTopic "smartvilages/river/endpointUjung"

// ---------- DEVICE ID --------//
#define deviceID "7b5d6a80-13ff-11eb-9168-92374923234"
#define deviceType "ESP8266"

// FlowSensors Structure
struct FlowSensors
{
    float calibrationFactor{4.5};
    volatile float flowRate{0};
    volatile uint32_t pulse{0};
    volatile uint32_t mililiters{0};
    unsigned long totalMilliLitres{0};
    unsigned long prevTime{0};

    
};

// rainSensor reading parameters
uint32 sensorReading{0};
uint16 sensorReadRange{0};

const int jsonCapacity = 313; // Calculated for esp8266 // AVR = 217

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
StaticJsonDocument<jsonCapacity> rawData;
FlowSensors sensors_flow_1;
FlowSensors sensors_flow_2;

JsonObject sensorsData = rawData.to<JsonObject>();
JsonArray flowSensorsArray = sensorsData.createNestedArray("flowSensors");
JsonArray rainSensorsArray = sensorsData.createNestedArray("rainSensors");
JsonObject flowSensorData_1 = flowSensorsArray.createNestedObject();
JsonObject flowSensorData_2 = flowSensorsArray.createNestedObject();
JsonObject rainSensorsData_1 = rainSensorsArray.createNestedObject();

String rawPayload{};

// Put interrupt routine to ram
void ICACHE_RAM_ATTR pulseCounter1() { sensors_flow_1.pulse++; };
void ICACHE_RAM_ATTR pulseCounter2() { sensors_flow_2.pulse++; };

void connect_wifi()
{
    Serial.println("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, wifiPass);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print(wifiClient.localIP());
}

void connect_mqtt()

{
    mqttClient.setClient(wifiClient);
    mqttClient.setServer(mqttServer, mqttPort);
    while (!mqttClient.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect(deviceID))
        {
            Serial.println("Connected");
            mqttClient.subscribe(mqttTopic);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

FlowSensors flowUpdate (FlowSensors flow) {

    flow.flowRate = ((1000.0 / (millis() - flow.prevTime)) * flow.pulse)/ flow.calibrationFactor;
    flow.prevTime = millis();
    flow.mililiters = (flow.flowRate / 60) * 1000;
    flow.totalMilliLitres += flow.mililiters;
    
    return flow;
}


void setup()
{
    Serial.begin(115200);
    sensorsData["deviceID"] = deviceID;
    sensorsData["deviceType"] = deviceType;

    pinMode(sensor_pin_flow_1, INPUT);
    pinMode(sensor_pin_flow_2, INPUT);
    pinMode(sensor_pin_rain_1, INPUT);

    digitalWrite(sensor_pin_flow_1, HIGH);
    digitalWrite(sensor_pin_flow_2, HIGH);

    connect_wifi();
    connect_mqtt();

    attachInterrupt(digitalPinToInterrupt(sensor_pin_flow_1), pulseCounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(sensor_pin_flow_2), pulseCounter2, RISING);
}

void loop()
{
    if (!mqttClient.connected())
    {
        connect_mqtt();
    }

    while (WiFi.status() == WL_CONNECTED && mqttClient.connected())
    {
        if((millis() - sensors_flow_1.prevTime) > 1000){
            
            //FlowSensor 1
            detachInterrupt(digitalPinToInterrupt(sensor_pin_flow_1));
            sensors_flow_1 = flowUpdate(sensors_flow_1);
            flowSensorData_1["flow"] = sensors_flow_1.mililiters;
            flowSensorData_1["flowTotal"] = sensors_flow_1.totalMilliLitres;
            sensors_flow_1.pulse = 0;
            attachInterrupt(digitalPinToInterrupt(sensor_pin_flow_1));
            
            //FlowSensor 2
            detachInterrupt(digitalPinToInterrupt(sensor_pin_flow_2));
            sensors_flow_2 = flowUpdate(sensors_flow_2);
            flowSensorData_2["flow"] = sensors_flow_2.mililiters;
            flowSensorData_2["flowTotal"] = sensors_flow_2.totalMilliLitres;
            sensors_flow_2.pulse = 0;
            attachInterrupt(digitalPinToInterrupt(sensor_pin_flow_2));

            sensorReading = analogRead(sensor_pin_rain_1);
            sensorReadRange = map(sensorReading, 0, 1024, 0, 10);
            rainSensorsData_1["waterResistance"] = sensorReadRange;

            serializeJson(rawData, rawPayload);

            mqttClient.publish(mqttTopic, rawPayload.c_str(), false);

            Serial.println(rawPayload); rawPayload = "";

            delay(2000);
            
        }


    }
    
}
