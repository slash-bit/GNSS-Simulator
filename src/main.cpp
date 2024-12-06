#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include <time.h>
#include "credentials.h"

#define JOYSTICK // Enable joystick code

#ifdef JOYSTICK
#define JOYSTICK_X_PIN 0 // ADC pin for X-axis
#define JOYSTICK_Y_PIN 1 // ADC pin for Y-axis
#define JOYSTICK_NEUTRAL 2048 // Apsprox. mid-point of ADC (for 12-bit ADC)
#define ALTITUDE_RATE 0.1 // Rate of change in altitude per unit deviation
#define COURSE_RATE 0.1 // Rate of change in course per unit deviation

int joystickXValue = 0;
int joystickYValue = 0;
float currentCourse = 90.0; // Starting course in degrees (east)
#else
#define JOYSTICK_X_PIN 0
#define JOYSTICK_Y_PIN 1
#endif

//define ssid and password from wifi.h file located in the include folder
// const char* ssid = SSID;
// const char* password = PASSWORD;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;
time_t fixTime;


// Function prototypes 
void updatePosition(int courseDeg); 
String generateGPGGASentence(float lat, float lon, float alt, const char* timeStr);
String generateGNGSASentence1();
String generateGNGSASentence2();
String generateGNRMCSentence(float lat, float lon, float speed, const char* timeStr);
String sentence;
String calculateChecksum(String sentence);
void handleUbloxQuery();

const char* ssid = "arse-5G";
const char* password = "Privet2999";

// Define two Serial devices mapped to the two internal UARTs
// HardwareSerial MySerial0(0);
HardwareSerial MySerial1(1);

const unsigned long interval = 1000; // Interval in milliseconds for updates
unsigned long previousMillis = 0;

float currentLat = 51.8727;
float currentLon = 0.125;
float speed_kmh = 30.0; // Speed in km/h
int courseDeg = 90; // Direction in degrees
float altitude = 100.0; // Altitude in meters

int baudRate = 9600; // Default baud rate
bool ubloxQuery = false;

AsyncWebServer server(80);

void setup() {
    Serial.begin(115200);

    // Configure MySerial0 on pins TX=6 and RX=7 (-1, -1 means use the default)
    // MySerial0.begin(9600, SERIAL_8N1, -1, -1);
    // MySerial0.print("MySerial0");

    // And configure MySerial1 on pins RX=D9, TX=D10
    MySerial1.begin(9600, SERIAL_8N1, 9, 10);
    MySerial1.print("MySerial1");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

        // Initialize and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }

      // Configure ADC pins for joystick
    #ifdef JOYSTICK
    pinMode(JOYSTICK_X_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK_Y_PIN, INPUT_PULLUP);
    #endif

    // Set up the web server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = "<html><body>"
                      "<h1>GPS Simulation Parameters</h1>"
                      "<form action='/set_parameters' method='POST'>"
                      "Latitude: <input type='text' name='lat' value='" + String(currentLat) + "'><br>"
                      "Longitude: <input type='text' name='lon' value='" + String(currentLon) + "'><br>"
                      "Speed (km/h): <input type='text' name='speed' value='" + String(speed_kmh) + "'><br>"
                      "Baud Rate: <input type='text' name='baud' value='" + String(baudRate) + "'><br>"
                      "<input type='submit' value='Set Parameters'>"
                      "</form></body></html>";
        request->send(200, "text/html", html);
    });

    server.on("/set_parameters", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("lat", true) && request->hasParam("lon", true) && request->hasParam("speed", true) && request->hasParam("baud", true)) {
            currentLat = request->getParam("lat", true)->value().toFloat();
            currentLon = request->getParam("lon", true)->value().toFloat();
            speed_kmh = request->getParam("speed", true)->value().toFloat();
            baudRate = request->getParam("baud", true)->value().toInt();
            // Update the baud rate of MySerial1
            MySerial1.updateBaudRate(baudRate);
        }
        request->send(200, "text/html", "Parameters updated! <a href='/'>Go Back</a>");
    });

    // Set up the web server for settings
    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = "<html><body>"
                      "<h1>GPS Simulator Settings</h1>"
                      "<form action='/set_joystick' method='POST'>"
                      "<h3>Joystick Settings:</h3>"
                      "<input type='checkbox' name='enable_joystick' value='1' checked> Enable Joystick<br>"
                      "Joystick X Pin: <input type='text' name='joystick_x_pin' value='" + String(JOYSTICK_X_PIN) + "'><br>"
                      "Joystick Y Pin: <input type='text' name='joystick_y_pin' value='" + String(JOYSTICK_Y_PIN) + "'><br>"
                      "<input type='submit' value='Save Settings'>"
                      "</form></body></html>";
        request->send(200, "text/html", html);
    });

    server.on("/set_joystick", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("enable_joystick", true)) {
            #define JOYSTICK
        } else {
            #undef JOYSTICK
        }
        request->send(200, "text/html", "Settings updated! <a href='/settings'>Go Back</a>");
    });

    server.begin();
    Serial.println("Server started");
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        updatePosition(courseDeg);

        fixTime = time(nullptr);  // Get current time in seconds
        struct tm * ptm = gmtime(&fixTime); // Convert to tm struct for UTC
        char timeStr[11];
        strftime(timeStr, sizeof(timeStr), "%H%M%S.00", ptm); // Format time as HHMMSS.00

	      String gnrmcSentence = generateGNRMCSentence(currentLat, currentLon, speed_kmh, timeStr);
		    String gpggaSentence = generateGPGGASentence(currentLat, currentLon, altitude, timeStr);
		    String gngsaSentence1 = generateGNGSASentence1();
		    String gngsaSentence2 = generateGNGSASentence2();
        
        Serial.println(gnrmcSentence);
        MySerial1.println(gnrmcSentence);

        Serial.println(gpggaSentence);
        MySerial1.println(gpggaSentence);
        
        Serial.println(gngsaSentence1);
        MySerial1.println(gngsaSentence1);

        Serial.println(gngsaSentence2);
        MySerial1.println(gngsaSentence2);
        

        // Toggle the PPS pin
        // digitalWrite(ppsPin, HIGH);
        // delay(10); // Small pulse duration
        // digitalWrite(ppsPin, LOW);

       
        // Handle u-blox queries
        if (!ubloxQuery) {
            handleUbloxQuery();
        }
    }
}

void updatePosition(int courseDeg) {
    float distance_km = (speed_kmh / 3600.0) * (interval / 1000.0);
    float deltaLon = distance_km / (111.32 * cos(radians(currentLat)));
    currentLon += deltaLon;

    #ifdef JOYSTICK
    // Read joystick values
    joystickXValue = analogRead(JOYSTICK_X_PIN);
    joystickYValue = analogRead(JOYSTICK_Y_PIN);

    // Calculate changes based on joystick position
    float altitudeChange = (joystickXValue - JOYSTICK_NEUTRAL) * ALTITUDE_RATE;
    float courseChange = (joystickYValue - JOYSTICK_NEUTRAL) * COURSE_RATE;

    altitude += altitudeChange;
    currentCourse += courseChange;

    // Ensure course is within 0 to 360 degrees
    if (currentCourse >= 360) currentCourse -= 360;
    if (currentCourse < 0) currentCourse += 360;

    // Debug output
    Serial.print("Joystick X: ");
    Serial.print(joystickXValue);
    Serial.print(" Joystick Y: ");
    Serial.println(joystickYValue);
    Serial.print("Altitude: ");
    Serial.println(altitude);
    Serial.print("Course: ");
    Serial.println(currentCourse);
    #endif
}


String generateGPGGASentence(float lat, float lon, float alt, const char* timeStr) {
    char nmea[120];
    int lat_deg = (int)lat;
    float lat_min = (lat - lat_deg) * 60.0;
    char lat_dir = (lat >= 0) ? 'N' : 'S';

    int lon_deg = (int)lon;
    float lon_min = (lon - lon_deg) * 60.0;
    char lon_dir = (lon >= 0) ? 'E' : 'W';

    snprintf(nmea, sizeof(nmea), "$GNGGA,%s,%02d%07.5f,%c,%03d%07.5f,%c,1,08,1.33,%.1f,M,45.6,M,,",
             timeStr, lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir, alt);

    String sentence = String(nmea);
    sentence += "*" + calculateChecksum(sentence);
    return sentence;
}

String generateGNGSASentence1() {
    char nmea[80];
    snprintf(nmea, sizeof(nmea), "$GNGSA,A,3,20,30,05,11,13,14,22,,,,,,3.03,2.01,2.28");
    String sentence = String(nmea);
    sentence += "*" + calculateChecksum(sentence);
    return sentence;
}

String generateGNGSASentence2() {
    char nmea[80];
    snprintf(nmea, sizeof(nmea), "$GNGSA,A,3,78,,,,,,,,,,,,3.03,2.01,2.28");
    String sentence = String(nmea);
    sentence += "*" + calculateChecksum(sentence);
    return sentence;
}

String generateGNRMCSentence(float lat, float lon, float speed, const char* timeStr) {
    char nmea[120];
    int lat_deg = (int)lat;
    float lat_min = (lat - lat_deg) * 60.0;
    char lat_dir = (lat >= 0) ? 'N' : 'S';
    
    int lon_deg = (int)lon;
    float lon_min = (lon - lon_deg) * 60.0;
    char lon_dir = (lon >= 0) ? 'E' : 'W';

    float speed_knots = speed * 0.539957; // Convert km/h to knots
    float course = courseDeg; // Course over ground in degrees (east)

    // Create date string
    char dateStr[7];
    struct tm * ptm = gmtime(&fixTime);
    strftime(dateStr, sizeof(dateStr), "%d%m%y", ptm); // Format date as DDMMYY

    snprintf(nmea, sizeof(nmea), "$GNRMC,%s,A,%02d%07.5f,%c,%03d%07.5f,%c,%.3f,%.1f,%s,,,A",
             timeStr, lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir, speed_knots, course, dateStr);

    String sentence = String(nmea);
    sentence += "*" + calculateChecksum(sentence);
    return sentence;
}

void handleUbloxQuery() {
    if (MySerial1.available()) {
        String query = MySerial1.readString();
        Serial.print("Received u-blox query: ");
        Serial.println(query);

        if (query.startsWith("$PCAS06")) {
            // Create response with version information
            String response = "$PCAS06,";
            // Adding padding to ensure the response is at least 33 characters long
            while (response.length() < 29) {
                response += "0";
            }
            response += "0008";
            response += "*" + calculateChecksum(response);
            response += "\r\n";
            MySerial1.println(response);
            Serial.print("Sent response: ");
            Serial.println(response);
        }
    }
}

// Checksum function
String calculateChecksum(String sentence) {
    byte checksum = 0;
    for (int i = 1; i < sentence.length(); i++) {
        checksum ^= byte(sentence[i]);
    }
    char checksumStr[3];
    sprintf(checksumStr, "%02X", checksum);
    return String(checksumStr);
}
