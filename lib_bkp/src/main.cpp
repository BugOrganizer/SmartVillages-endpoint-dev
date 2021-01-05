#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <creds.h>

#define flowPin_1 D2
#define flowPin_2 D3
#define rainPin_1 A0

#define BAUDRATE 115200

struct FlowSensors
{
    float calFactor{4.5};
    volatile float fRate;
    volatile uint32_t pulse;
    volatile uint32_t mLiters;
    unsigned long mLitersTotal;
    unsigned long preTime;
};

struct RainSensor
{
    uint32 sensorReading;
    uint16 sensorReadRange;
};

const int jsonCapacity = 313; // Calculated for esp8266 // AVR = 217

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
StaticJsonDocument<jsonCapacity> rawData;

JsonObject sensObj = rawData.to<JsonObject>();
JsonArray fSensArr = sensObj.createNestedArray("flowSensors");
JsonArray rSensArr = sensObj.createNestedArray("rainSensors");
JsonObject fSensDat_1 = fSensArr.createNestedObject();
JsonObject fSensDat_2 = fSensArr.createNestedObject();
JsonObject rSensDat_1 = rSensArr.createNestedObject();

FlowSensors sensors_flow_1;
FlowSensors sensors_flow_2;
RainSensor  sensors_rain_1;

String rawPayload{};

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

void readFlowSens(FlowSensors *f, JsonObject &j)
{

    f->fRate = ((1000.0 / (millis() - f->preTime)) * f->pulse) / f->calFactor;
    f->preTime = millis();
    f->mLiters = (f->fRate/60) * 1000;
    f->mLitersTotal += f->mLiters;
    j["flow"] = f->mLiters;
    j["flowTotal"] = f->mLitersTotal;
    f->pulse = 0;

}

void readRainSens(RainSensor *r, JsonObject &j) {

    r->sensorReading = analogRead(rainPin_1);
    r->sensorReadRange = map(r->sensorReading, 0, 1024, 0, 10);
    j["waterResistance"] = r->sensorReadRange;

}

void setup()
{

    Serial.begin(BAUDRATE);

    sensObj["deviceID"] = deviceID;
    sensObj["deviceType"] = deviceType;

    pinMode(flowPin_1, INPUT);
    pinMode(flowPin_2, INPUT);
    pinMode(rainPin_1, INPUT);

    digitalWrite(flowPin_1, HIGH);
    digitalWrite(flowPin_2, HIGH);

    connect_wifi();
    connect_mqtt();

    attachInterrupt(digitalPinToInterrupt(flowPin_1), pulseCounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(flowPin_2), pulseCounter2, RISING);

}

void loop()
{
    if (!mqttClient.connected())
    {
        connect_mqtt();
    }

    while (WiFi.status() == WL_CONNECTED && mqttClient.connected())
    {

        detachInterrupt(digitalPinToInterrupt(flowPin_1));
        readFlowSens(&sensors_flow_1,fSensDat_1);
        
        detachInterrupt(digitalPinToInterrupt(flowPin_2));
        readFlowSens(&sensors_flow_2,fSensDat_2);

        readRainSens(&sensors_rain_1,rSensDat_1);

        serializeJson(rawData, rawPayload);
        mqttClient.publish(mqttTopic, rawPayload.c_str(), false);
        

        Serial.println(rawPayload);
        rawPayload = "";

        attachInterrupt(digitalPinToInterrupt(flowPin_1), pulseCounter1, RISING);
        attachInterrupt(digitalPinToInterrupt(flowPin_2), pulseCounter2, RISING);
        delay(2000);

    }

}