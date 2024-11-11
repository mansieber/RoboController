#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NewPingESP8266.h>

#define DEBUG

#define PORT_BROKER 1883
#define PIN_ENA_L 4            // D2 on NodeMCU 
#define PIN_ENA_R 5            // D1 on NodeMCU 
#define PIN_DIR_L 14           // D5 on NodeMCU
#define PIN_DIR_R 12           // D6 on NodeMCU
#define PIN_ECHO 13            // D7 on NodeMCU
#define PIN_TRIG 15            // D8 on NodeMCU

#define TURNING_SPEED 30.0
#define MAX_SPEED 100.0
#define MAX_DEFLECTION 100.0
#define MAX_DISTANCE 200        // Maximum distance in cm
#define WAIT_TIME 10            // Time in ms defines the wait duration in loop
#define SONAR_TIME 1000         // Time in ms defines the duration between two sonar measurements

enum turningMode { TURNING_LEFT = -1, TURNING_OFF, TURNING_RIGHT };
enum drivingMode { MANUAL = 0, AUTO };
enum side { LEFT, RIGHT };

const char* ssid = "MiMaWireless";
const char* passwd = "landofmordor5967";
const char* broker = "192.168.178.31";
const char* clientName = "robi";
const char* publishedTopic = "mimamisc/roboData";
const char* subscribedTopic = "mimamisc/roboControl";
const char* keySpeed = "Speed";
const char* keyDeflection = "Deflection";
const char* keyTurn = "Turn";
const char* keyMode = "Mode";
const char* keyDistance = "Distance";

unsigned int speedRight = 0;
unsigned int speedLeft = 0;
float deflection = 0.0;
float speed = 0.0;
enum turningMode turning = TURNING_OFF;
enum drivingMode mode = MANUAL;

WiFiClient espClient;
PubSubClient client(espClient);
NewPingESP8266 sonar(PIN_TRIG, PIN_ECHO, MAX_DISTANCE);

void sendDistanceMessage(unsigned int);
void setup_wifi();
void reconnect();
void callback(char*, byte*, unsigned int);
void setSpeedAndDirectionOfMotors();
unsigned int speedValue(float);
int sonarTimer = 0;

void setup() {
    pinMode(PIN_ENA_R, OUTPUT);     // Initialize pin PIN_ENA_R as an output
    pinMode(PIN_ENA_L, OUTPUT);     // Initialize pin PIN_ENA_L as an output
    pinMode(PIN_DIR_R, OUTPUT);     // Initialize pin PIN_DIR_R as an output
    pinMode(PIN_DIR_L, OUTPUT);     // Initialize pin PIN_DIR_L as an output
    setSpeedAndDirectionOfMotors();
#ifdef DEBUG    
    Serial.begin(115200);
    while ( ! Serial );
    Serial.println();
    Serial.println("Set up Wifi ... ");
#endif
    setup_wifi();
    client.setServer(broker, PORT_BROKER);
    client.setCallback(callback);
}

// the loop function runs over and over again forever
void loop() {
        if ( ! client.connected() ) {
        reconnect();
    }
    client.loop();
    delay(WAIT_TIME);
    if ( ++sonarTimer >= ( SONAR_TIME / WAIT_TIME ) ) {
        unsigned int distance = sonar.ping_cm();
        sendDistanceMessage(distance);
#ifdef DEBUG    
        Serial.print("Distance: ");
        Serial.println(distance);
#endif
        sonarTimer = 0;
    }
}

/*
 * Method sends MQTT message with distance (to object ahead) data.
 */
void sendDistanceMessage(unsigned int distance) {
    StaticJsonDocument<512> msgBuffer;
    String payload;
    msgBuffer["Distance"] = distance;
    int n = serializeJson(msgBuffer, payload);
#ifdef DEBUG    
    Serial.print("JSON object serialized, n = ");
    Serial.println(n);
#endif
    client.publish(publishedTopic, payload.c_str());
}

/*
 * Method sets up the WLAN connection.
 */
void setup_wifi() {
    delay(10);
#ifdef DEBUG    
    Serial.print("Connecting to ");
    Serial.println(ssid);
#endif    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, passwd);
    while ( WiFi.status() != WL_CONNECTED ) {
        delay(500);
#ifdef DEBUG        
        Serial.print(".");
#endif        
    }
#ifdef DEBUG    
    Serial.print(" connected, ");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
#endif
}

void reconnect() {
    while ( ! client.connected() ) {
#ifdef DEBUG    
        Serial.print("Attempting MQTT connection ... ");
#endif
        if ( client.connect(clientName) ) {
#ifdef DEBUG    
            Serial.println("connected");
#endif
            client.subscribe(subscribedTopic);
        } else {
#ifdef DEBUG    
            Serial.print("failed: ");
            Serial.print(client.state());
            Serial.println(" retry in 5 sec ...");
#endif
            delay(5000);
        }
    }
}

/*
 * Method is called if MQTT message was received
 */
void callback(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<512> msgBuffer;
#ifdef DEBUG            
    Serial.print("Message on topic ");
    Serial.print(topic);
    Serial.print(": ");
    for ( unsigned int i = 0; i < length; i++ ) {
        Serial.print((char) payload[i]);
    }
#endif
    DeserializationError error = deserializeJson(msgBuffer, payload);
    if ( error ) {
#ifdef DEBUG            
        Serial.print(" deserialization failed: ");
        Serial.println(error.f_str());
#endif
        return;
    } else {
#ifdef DEBUG            
        Serial.println(" deserialized");
#endif
        if ( msgBuffer.containsKey(keySpeed) ) {
            String sSpeed = msgBuffer[keySpeed];
            speed = sSpeed.toFloat();
#ifdef DEBUG            
            Serial.print("key: ");
            Serial.print(keySpeed);
            Serial.print(" value: ");
            Serial.println(speed);
#endif
            turning = TURNING_OFF;
            setSpeedAndDirectionOfMotors();
        } else if ( msgBuffer.containsKey(keyDeflection) ) {
            String sDeflection = msgBuffer[keyDeflection];
            deflection = sDeflection.toFloat();
#ifdef DEBUG            
            Serial.print("key: ");
            Serial.print(keyDeflection);
            Serial.print(" value: ");
            Serial.println(deflection);
#endif
            setSpeedAndDirectionOfMotors();
        } else if ( msgBuffer.containsKey(keyTurn) ) {
            String sTurningMode = msgBuffer[keyTurn];
            turning = ( sTurningMode.compareTo("Left") == 0 ) ? TURNING_LEFT : 
                      ( ( sTurningMode.compareTo("Right") == 0 ) ? TURNING_RIGHT : TURNING_OFF );
#ifdef DEBUG            
            Serial.print("key: ");
            Serial.print(keyTurn);
            Serial.print(" value: ");
            Serial.println(turning);
#endif
            setSpeedAndDirectionOfMotors();
        } else if ( msgBuffer.containsKey(keyMode) ) {
            String sMode = msgBuffer[keyMode];
            mode = ( sMode.compareTo("Auto") == 0 ) ? AUTO : MANUAL;
#ifdef DEBUG            
            Serial.print("key: ");
            Serial.print(keyMode);
            Serial.print(" value: ");
            Serial.println(mode);
#endif
            
        } else {    
#ifdef DEBUG            
            Serial.println(" unknown key!");
#endif
            return;
        }
    }
}

/*
 * Set the speed and turning direction of both motors.
 * Full behavior is defined in this function.
 */
void setSpeedAndDirectionOfMotors() {
    float vr, vl;
    boolean dr, dl;
    if ( turning == TURNING_LEFT || turning == TURNING_RIGHT ) {   // turning
        vr = vl = TURNING_SPEED;
        dl = turning == TURNING_RIGHT;
        dr = turning == TURNING_LEFT;
    } else {                                                       // no turning
        boolean moveForward = true;
        if ( deflection > MAX_DEFLECTION ) {
            deflection = MAX_DEFLECTION;
        } else if ( deflection < - MAX_DEFLECTION ) {
            deflection = - MAX_DEFLECTION;
        }
        if ( speed < 0.0 ) {
            moveForward = false;
            speed = - speed;
        }
        if ( speed > MAX_SPEED ) {
            speed = MAX_SPEED;
        }
#ifdef DEBUG
        Serial.printf("Filtered speed and deflection: speed = %6.1f, deflection = %6.1f\n", speed, deflection);
#endif        
        vr = ( 1.0 - deflection / MAX_DEFLECTION ) * speed;
        vl = ( 1.0 + deflection / MAX_DEFLECTION ) * speed;
#ifdef DEBUG
        Serial.printf("Preliminary motor speeds: vr = %6.1f, vl = %6.1f\n", vr, vl);
#endif        
        if ( vr > 100.0 ) {
            vl = ( 100.0 + deflection ) / ( 100.0 - deflection ) * ( vr = 100.0 );
        } else if ( vl > 100.0 ) {
            vr = ( 100.0 - deflection ) / ( 100.0 + deflection ) * ( vl = 100.0 );
        }
        dr = dl = moveForward;
    }
#ifdef DEBUG            
    Serial.print("New motor speeds and directions: vr = ");
    Serial.print(vr);
    Serial.print(", vl = ");
    Serial.print(vl);
    Serial.print(", dr = ");
    Serial.print(dr ? "forward" : "bckward");
    Serial.print(", dl = ");
    Serial.println(dl ? "forward" : "bckward");
#endif
    analogWrite(PIN_ENA_R, speedValue(vr));
    analogWrite(PIN_ENA_L, speedValue(vl));
    digitalWrite(PIN_DIR_R, dr ? HIGH : LOW);
    digitalWrite(PIN_DIR_L, dl > 0 ? HIGH : LOW);
}

/*
 * Calculate the speed value transmitted via the L298N interface.
 * Because motors will not turn if the voltage is too low, a transfer 
 * function with a lower cut off value is used to calculate the 
 * voltage.
 */
unsigned int speedValue(float speed) {
    // Motor transfer function: 0 -> 0, 20 -> 60, 40 -> 70, ..., 100 -> 100
    unsigned int correctedSpeed = ( speed < 10.0 ? 0 : ( (unsigned int) ( ( 50.0f + 0.5f * speed ) * 255.0f / 100.0f ) ) );
#ifdef DEBUG    
    Serial.print("Corrected speed = ");
    Serial.println(correctedSpeed);
#endif
    return correctedSpeed;
}

