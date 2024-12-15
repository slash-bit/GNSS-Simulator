#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include <time.h>
#include "credentials.h"

#define JOYSTICK // Enable joystick code
// #define DEBUG_UPDATEPOSITION // Enable debug output for updatePosition
// #define DEBUG_JOYSTICK // Enable debug output for joystick
#ifdef JOYSTICK
#define JOYSTICK_X_PIN 2 // ADC pin for X-axis
#define JOYSTICK_Y_PIN 3 // ADC pin for Y-axis
int JOYSTICK_NEUTRAL_X = 1935; // Approx. mid-point of ADC for X-axis
int JOYSTICK_NEUTRAL_Y = 2125; // Approx. mid-point of ADC for Y-axis
int JOYSTICK_MAX_X = 4095; // Maximum ADC value for X-axis
int JOYSTICK_MIN_X = 0; // Minimum ADC value for X-axis
int JOYSTICK_MAX_Y = 4095; // Maximum ADC value for Y-axis
int JOYSTICK_MIN_Y = 0; // Minimum ADC value for Y-axis
#define ALTITUDE_RATE 10.0 // Rate of change in altitude per unit deviation
#define COURSE_RATE 20.0 // Rate of change in course per unit deviation

int joystickXValue = 0;
int joystickYValue = 0;
float heading = 90.0; // Aircraft heading in degrees
float currentCourse = 90.0; // Starting course in degrees (east)
#else
#define JOYSTICK_X_PIN 0
#define JOYSTICK_Y_PIN 1
#endif

//define ssid and password from wifi.h file located in the include folder
const char* ssid = SSID;
const char* password = PASSWORD;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;
time_t fixTime;


// Function prototypes 
void updatePosition(int courseDeg);
void updatePositionOther();
String generateGPGGASentence(float lat, float lon, float alt, const char* timeStr);
String generateGNGSASentence1();
String generateGNGSASentence2();
String generateGNRMCSentence(float lat, float lon, float speed, float currentCourse, const char* timeStr);
String generatePFSIMSentence(float lat, float aircraftLon, float speed, float alt, float course, float altitudeChange, float aircraftTurnRate, const char* timeStr);
String generateLXWP0sentence(float altitude, float altitudeChange, int heading, int windDirection, float windStrength);
float distancebetweenAircrafts(float lat1, float lon1, float lat2, float lon2);
String sentence;
String calculateChecksum(String sentence);
void handleUbloxQuery();

// Define two Serial devices mapped to the two internal UARTs
// HardwareSerial MySerial0(0);
HardwareSerial MySerial1(1);

const unsigned long interval = 1000; // Interval in milliseconds for updates
unsigned long previousMillis = 0;
//section for other aircraft
String aircraftAddress = "0A0001";
int aircraftAddressType = 1;
int aircraftType = 1;
float aircraftLat = 51.880814;
float aircraftLon = 0.100230;
float aircraftSpeed = 45.0;
float aircraftCourse = 120.0;
float aircraftAltitude = 150.0;
float aircrafteffectiveSpeed = 0.0;
float aircrafteffectiveCourse = 0.0;
float aircraftTurnRate = 0.0; // Rate of turn in degrees per second
float aircraftAltitudeChange = 0.0; // Rate of change in altitude in meters per second
//end of other aircraft section
//myShip section
float currentLat = 51.869999;
float currentLon = 0.164370;
float speed_kmh = 45.0; // Speed in km/h
int courseDeg = 90; // Direction in degrees
float oldCourse = 0.0;
float altitude = 150.0; // Altitude in meters
float altitudeChange = 0.0; // Rate of change in altitude in meters per second
float effectiveSpeed = 0.0; // Effective speed in km/h based on wind
float effectiveCourse = 0.0; // Effective course in degrees based on wind
//wind section
float windDirection = 0.0; // Wind direction in degrees
float windStrength = 0.0; // Wind strength in km/h

int baudRate = 9600; // Default baud rate
bool ubloxQuery = false;

AsyncWebServer server(80);

void setup() {
    Serial.begin(9600);

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
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);
    #endif


 	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body>"
                  "<h1>GPS Simulator Settings</h1>"
                  "<form action='/set_parameters' method='POST'>"
                  "<h3>GPS Simulation Parameters</h3>"
                  "Latitude: <input type='text' name='lat' value='" + String(currentLat) + "'><br>"
                  "Longitude: <input type='text' name='lon' value='" + String(currentLon) + "'><br>"
                  "Speed (km/h): <input type='text' name='speed' value='" + String(speed_kmh) + "'><br>"
                  "Heading (degrees): <input type='text' name='heading' value='" + String(heading) + "'><br>"
                  "Altitude (m): <input type='text' name='altitude' value='" + String(altitude) + "'><br>"
                  "Wind Direction (degrees): <input type='text' name='wind_dir' value='" + String(windDirection) + "'><br>"
                  "Wind Strength (km/h): <input type='text' name='wind_str' value='" + String(windStrength) + "'><br>"
                  "<hr>"
                    "<h3>Aircraft Simulation Parameters</h3>"
                  "Address: <input type='text' name='aircraft_address' value='" + aircraftAddress + "'><br>"
                  "Address Type: <input type='number' name='aircraft_address_type' value='" + String(aircraftAddressType) + "'><br>"
                  "Aircraft Type: <input type='number' name='aircraft_type' value='" + String(aircraftType) + "'><br>"
                  "Latitude: <input type='text' name='aircraft_lat' value='" + String(aircraftLat) + "'><br>"
                  "Longitude: <input type='text' name='aircraft_lon' value='" + String(aircraftLon) + "'><br>"
                  "Speed: <input type='text' name='aircraft_speed' value='" + String(aircraftSpeed) + "'><br>"
                  "Course: <input type='text' name='aircraft_course' value='" + String(aircraftCourse) + "'><br>"
                  "Altitude: <input type='text' name='aircraft_altitude' value='" + String(aircraftAltitude) + "'><br>"
                  "<input type='submit' value='Set Parameters'>"
                  "</form></body></html>"
                  "<hr>"
                  "<form action='/set_joystick' method='POST'>"
                  "<h3>Joystick Settings:</h3>"
                  "<input type='checkbox' name='enable_joystick' value='1' checked> Enable Joystick<br>"
                  "Joystick X Pin: <input type='text' name='joystick_x_pin' value='" + String(JOYSTICK_X_PIN) + "'><br>"
                  "Joystick Y Pin: <input type='text' name='joystick_y_pin' value='" + String(JOYSTICK_Y_PIN) + "'><br>"
                  "Joystick X Neutral: <input type='number' name='joystick_x_neutral' value='" + String(JOYSTICK_NEUTRAL_X) + "'><br>"
                  "Joystick Y Neutral: <input type='number' name='joystick_y_neutral' value='" + String(JOYSTICK_NEUTRAL_Y) + "'><br>"
                  "Joystick X Max: <input type='number' name='joystick_x_max' value='" + String(JOYSTICK_MAX_X) + "'><br>"
                  "Joystick X Min: <input type='number' name='joystick_x_min' value='" + String(JOYSTICK_MIN_X) + "'><br>"
                  "Joystick Y Max: <input type='number' name='joystick_y_max' value='" + String(JOYSTICK_MAX_Y) + "'><br>"
                  "Joystick Y Min: <input type='number' name='joystick_y_min' value='" + String(JOYSTICK_MIN_Y) + "'><br>"
                  "<input type='submit' value='Save Settings'>"
                  "</form></body></html>";
    request->send(200, "text/html", html);
});

server.on("/set_parameters", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("lat", true) && request->hasParam("lon", true) && request->hasParam("speed", true) && request->hasParam("heading", true) && request->hasParam("altitude", true) && request->hasParam("wind_dir", true) && request->hasParam("wind_str", true) && request->hasParam("aircraft_address", true) && request->hasParam("aircraft_address_type", true) &&
        request->hasParam("aircraft_type", true) && request->hasParam("aircraft_lat", true) &&
        request->hasParam("aircraft_lon", true) && request->hasParam("aircraft_speed", true) &&
        request->hasParam("aircraft_course", true) && request->hasParam("aircraft_altitude", true)) {
        currentLat = request->getParam("lat", true)->value().toFloat();
        currentLon = request->getParam("lon", true)->value().toFloat();
        speed_kmh = request->getParam("speed", true)->value().toFloat();
        heading = request->getParam("heading", true)->value().toFloat();
        altitude = request->getParam("altitude", true)->value().toFloat();
        windDirection = request->getParam("wind_dir", true)->value().toFloat();
        windStrength = request->getParam("wind_str", true)->value().toFloat();
        aircraftAddress = request->getParam("aircraft_address", true)->value();
        aircraftAddressType = request->getParam("aircraft_address_type", true)->value().toInt();
        aircraftType = request->getParam("aircraft_type", true)->value().toInt();
        aircraftLat = request->getParam("aircraft_lat", true)->value().toFloat();
        aircraftLon = request->getParam("aircraft_lon", true)->value().toFloat();
        aircraftSpeed = request->getParam("aircraft_speed", true)->value().toFloat();
        aircraftCourse = request->getParam("aircraft_course", true)->value().toFloat();
        aircraftAltitude = request->getParam("aircraft_altitude", true)->value().toFloat();
    }
    request->send(200, "text/html", "Parameters updated! <a href='/'>Go Back</a>");
});

server.on("/set_joystick", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("joystick_x_neutral", true) && request->hasParam("joystick_y_neutral", true) && 
        request->hasParam("joystick_x_max", true) && request->hasParam("joystick_x_min", true) &&
        request->hasParam("joystick_y_max", true) && request->hasParam("joystick_y_min", true)) {

        JOYSTICK_NEUTRAL_X = request->getParam("joystick_x_neutral", true)->value().toInt();
        JOYSTICK_NEUTRAL_Y = request->getParam("joystick_y_neutral", true)->value().toInt();
        JOYSTICK_MAX_X = request->getParam("joystick_x_max", true)->value().toInt();
        JOYSTICK_MIN_X = request->getParam("joystick_x_min", true)->value().toInt();
        JOYSTICK_MAX_Y = request->getParam("joystick_y_max", true)->value().toInt();
        JOYSTICK_MIN_Y = request->getParam("joystick_y_min", true)->value().toInt();
    }
    if (request->hasParam("enable_joystick", true)) {
        #define JOYSTICK
    } else {
        #undef JOYSTICK
    }
    request->send(200, "text/html", "Settings updated! <a href='/'>Go Back</a>");
});

server.begin();
Serial.println("Server started");
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        updatePosition(courseDeg);
        updatePositionOther();

        fixTime = time(nullptr);  // Get current time in seconds
        struct tm * ptm = gmtime(&fixTime); // Convert to tm struct for UTC
        char timeStr[11];
        strftime(timeStr, sizeof(timeStr), "%H%M%S", ptm); // Format time as HHMMSS

        String gnrmcSentence = generateGNRMCSentence(currentLat, currentLon, effectiveSpeed, currentCourse, timeStr);
        String gpggaSentence = generateGPGGASentence(currentLat, currentLon, altitude, timeStr);
        String pfsimSentence = generatePFSIMSentence(aircraftLat, aircraftLon, aircrafteffectiveSpeed, aircraftAltitude, aircraftCourse, altitudeChange, aircraftTurnRate, timeStr);
        String lxwp0Sentence = generateLXWP0sentence(altitude, altitudeChange, heading, windDirection, windStrength);

        Serial.println(gpggaSentence);
        MySerial1.println(gpggaSentence);
        
        Serial.println(gnrmcSentence);
        MySerial1.println(gnrmcSentence);
        
        // Serial.println(pfsimSentence);
        MySerial1.println(pfsimSentence);

        // Serial.println(lxwp0Sentence);
        MySerial1.println(lxwp0Sentence);
#ifdef DEBUG_UPDATEPOSITION
        Serial.printf("Time: %s Distance between aircrafts: %.2f\n\r", timeStr, distancebetweenAircrafts(currentLat, currentLon, aircraftLat, aircraftLon));
#endif
        // Serial.println(gngsaSentence1);
        // MySerial1.println(gngsaSentence1);

        // Serial.println(gngsaSentence2);
        // MySerial1.println(gngsaSentence2);
       
        // Serial.println();

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
    float distance_km = (speed_kmh / 3600.0) * (interval / 1000.0); // Distance traveled in km
    float headingRad = radians(heading); // Convert heading to radians
    float windRad = radians(windDirection); // Convert wind direction to radians

    // Calculate effective components due to wind and heading
    float effectiveSpeedX = (speed_kmh * cos(headingRad)) + (windStrength * cos(windRad));
    float effectiveSpeedY = (speed_kmh * sin(headingRad)) + (windStrength * sin(windRad));
    effectiveSpeed = sqrt(sq(effectiveSpeedX) + sq(effectiveSpeedY));

    // Calculate the effective course considering wind
    float effectiveCourseRad = atan2(effectiveSpeedY, effectiveSpeedX);
    effectiveCourse = degrees(effectiveCourseRad);

    // Convert effective course back to degrees
    currentCourse = effectiveCourse;
    if (currentCourse >= 360) currentCourse -= 360;
    if (currentCourse < 0) currentCourse += 360;

    // Calculate changes in latitude and longitude based on the effective course
    float deltaLat = (effectiveSpeed / 3600.0) * (interval / 1000.0) * cos(effectiveCourseRad) / 111.32; // Approx. 1 degree of latitude = 111.32 km
    float deltaLon = (effectiveSpeed / 3600.0) * (interval / 1000.0) * sin(effectiveCourseRad) / (111.32 * cos(radians(currentLat))); // Longitude distance depends on latitude

    currentLat += deltaLat;
    currentLon += deltaLon;

    // #ifdef JOYSTICK
    // Read joystick values
    joystickXValue = analogRead(JOYSTICK_X_PIN);
    joystickYValue = analogRead(JOYSTICK_Y_PIN);

   // Normalize joystick values
    float normalizedX = float(joystickXValue - JOYSTICK_NEUTRAL_X) / float(JOYSTICK_MAX_X - JOYSTICK_MIN_X);
    float normalizedY = float(joystickYValue - JOYSTICK_NEUTRAL_Y) / float(JOYSTICK_MAX_Y - JOYSTICK_MIN_Y);

    // Calculate changes based on joystick position
    float altitudeChange = normalizedX * ALTITUDE_RATE;
    float headingChange = normalizedY * COURSE_RATE;

        // If joystick is in the neutral position for X-axis (Acsend rate), set altitude change to 0
    if (abs(joystickXValue - JOYSTICK_NEUTRAL_X) < 60) { // 60 is a small threshold for neutral position
        altitudeChange = 0;
    }

            // If joystick is in the neutral position for Y-axis (Turn rate), set course change to 0
    if (abs(joystickYValue - JOYSTICK_NEUTRAL_Y) < 15) { // 15 is a small threshold for neutral position
        headingChange = 0;
    }

    altitude += altitudeChange;
    heading += headingChange;

    // Ensure course is within 0 to 360 degrees
    if (currentCourse >= 360) currentCourse -= 360;
    if (currentCourse < 0) currentCourse += 360;

    aircraftTurnRate = oldCourse - currentCourse;
    oldCourse = currentCourse;

        // Ensure heading is within 0 to 360 degrees
    if (heading >= 360) heading -= 360;
    if (heading < 0) heading += 360;

#ifdef DEBUG_JOYSTICK
    // Joystick Debug output
    Serial.print("Joystick X: ");
    Serial.print(joystickXValue);
    Serial.print(" Joystick Y: ");
    Serial.println(joystickYValue);
    Serial.print(" X Norm: ");
    Serial.print(normalizedX);
    Serial.print(" Y Norm: ");
    Serial.println(normalizedY);
#endif
#ifdef DEBUG_UPDATEPOSITION
    Serial.printf("OwnShip : Lat: %f, Lon: %f, Speed: %.0f, Course: %.0f, Altitude: %.1f\n\r", currentLat, currentLon, effectiveSpeed, effectiveCourse, altitude);
    // Serial.println(altitude);

    // Serial.print("ClimbRate: ");
    // Serial.println(altitudeChange);

    // Serial.print("Heading: ");
    // Serial.println(heading);
 
    // Serial.print("Course: ");
    // Serial.println(currentCourse);

    // Serial.print("TurnRate: ");
    // Serial.println(aircraftTurnRate);

    // Serial.print("GrSpeed: ");
    // Serial.println(effectiveSpeed);
#endif
}

void updatePositionOther() {
    float distance_km = (aircraftSpeed / 3600.0) * (interval / 1000.0); // Distance traveled in km
    float headingRad = radians(aircraftCourse); // Convert heading to radians
    float windRad = radians(windDirection); // Convert wind direction to radians

    // Calculate effective components due to wind and heading
    float effectiveSpeedX = (aircraftSpeed * cos(headingRad)) + (windStrength * cos(windRad));
    float effectiveSpeedY = (aircraftSpeed * sin(headingRad)) + (windStrength * sin(windRad));
    aircrafteffectiveSpeed = sqrt(sq(effectiveSpeedX) + sq(effectiveSpeedY));

    // Calculate the effective course considering wind
    float effectiveCourseRad = atan2(effectiveSpeedY, effectiveSpeedX);
    aircrafteffectiveCourse = degrees(effectiveCourseRad);

    // Convert effective course back to degrees
    if (aircrafteffectiveCourse >= 360) aircrafteffectiveCourse -= 360;
    if (aircrafteffectiveCourse < 0) aircrafteffectiveCourse += 360;

    // Calculate changes in latitude and longitude based on the effective course
    float deltaLat = (aircrafteffectiveSpeed / 3600.0) * (interval / 1000.0) * cos(effectiveCourseRad) / 111.32; // Approx. 1 degree of latitude = 111.32 km
    float deltaLon = (aircrafteffectiveSpeed / 3600.0) * (interval / 1000.0) * sin(effectiveCourseRad) / (111.32 * cos(radians(aircraftLat))); // Longitude distance depends on latitude

    aircraftLat += deltaLat;
    aircraftLon += deltaLon;
    #ifdef DEBUG_UPDATEPOSITION
    Serial.printf("Aircraft: Lat: %f, Lon: %f, Speed: %.0f, Course: %.0f, Altitude: %.1f\n\r", aircraftLat, aircraftLon, aircrafteffectiveSpeed, aircrafteffectiveCourse, aircraftAltitude);

    // Serial.print("aircraft Effective Speed: ");
    // Serial.print(aircrafteffectiveSpeed);
    // Serial.print(" aircraft effective Course : ");
    // Serial.println(aircrafteffectiveCourse );
    // Serial.print(" aircraft Lat: ");
    // Serial.print(aircraftLat, 6);
    // Serial.print(" aircraft Lon: ");
    // Serial.println(aircraftLon, 6);

    // Serial.print("Altitude: ");
    // Serial.println(aircraftAltitude);

    // Serial.print("Delta Lat: ");
    // Serial.println(deltaLat, 6);

    // Serial.print("Delta Lon: ");
    // Serial.println(deltaLon, 6);
 
#endif

}
float distancebetweenAircrafts(float lat1, float lon1, float lat2, float lon2) {
    float R = 6371; // Radius of the Earth in kilometers
    float dLat = radians(lat2 - lat1); // Difference in latitude in radians
    float dLon = radians(lon2 - lon1); // Difference in longitude in radians

    // Haversine formula to calculate the great-circle distance
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c; // Distance in kilometers
}
String generateGPGGASentence(float lat, float lon, float alt, const char* timeStr) {
    char nmea[120];
    int lat_deg = (int)lat;
    float lat_min = (lat - lat_deg) * 60.0;
    char lat_dir = (lat >= 0) ? 'N' : 'S';

    int lon_deg = (int)lon;
    float lon_min = (lon - lon_deg) * 60.0;
    char lon_dir = (lon >= 0) ? 'E' : 'W';

    snprintf(nmea, sizeof(nmea), "$GPGGA,%s,%02d%07.5f,%c,%03d%07.5f,%c,1,08,1.33,%.1f,M,45.6,M,,",
             timeStr, lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir, alt);

    String sentence = String(nmea);
    sentence += "*" + calculateChecksum(sentence);
    return sentence;
}


String generateGNRMCSentence(float lat, float lon, float speed, float currentCourse, const char* timeStr) {
    char nmea[120];
    int lat_deg = (int)lat;
    float lat_min = (lat - lat_deg) * 60.0;
    char lat_dir = (lat >= 0) ? 'N' : 'S';
    
    int lon_deg = (int)lon;
    float lon_min = (lon - lon_deg) * 60.0;
    char lon_dir = (lon >= 0) ? 'E' : 'W';

    float speed_knots = speed * 0.539957; // Convert km/h to knots
    float course = currentCourse; // Course over ground in degrees (east)
    // Create date string
    char dateStr[7];
    struct tm * ptm = gmtime(&fixTime);
    strftime(dateStr, sizeof(dateStr), "%d%m%y", ptm); // Format date as DDMMYY

    snprintf(nmea, sizeof(nmea), "$GPRMC,%s,A,%02d%07.5f,%c,%03d%07.5f,%c,%.3f,%.1f,%s,,,A",
             timeStr, lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir, speed_knots, course, dateStr);

    String sentence = String(nmea);
    sentence += "*" + calculateChecksum(sentence);
    return sentence;
}

String generatePFSIMSentence(float lat, float lon, float speed, float alt, float course, float altitudeChange, float aircraftTurnRate, const char* timeStr) {
    char nmea[120];
    float aircraftCourse = course; 
    float aircraftAlt = alt; // Altitude in meters
    float speed_ms = speed * 0.2777777; // Convert km/h to meters per second

    snprintf(nmea, sizeof(nmea), "$PFSIM,%s,%s,%d,%d,%.6f,%.6f,%.1f,%.2f,%.2f,%.2f,%.2f",
             timeStr, aircraftAddress.c_str(), aircraftAddressType, aircraftType,
             lat, lon, aircraftAlt, speed_ms, aircraftCourse,
             altitudeChange, aircraftTurnRate);

    String sentence = String(nmea);
    sentence += "*" + calculateChecksum(sentence);
    return sentence;
}

String generateLXWP0sentence(float altitude, float altitudeChange, int heading, int windDirection, float windStrength) {
    //$LXWP0,N,,5.85,-0.01,,,,,,0,180,0.0*6E
    char nmea[120];
    snprintf(nmea, sizeof(nmea), "$LXWP0,N,,%.2f,%.2f,,,,,,%d,%d,%.1f",
             altitude, altitudeChange, heading, windDirection, windStrength);

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
