#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include <time.h>
#include "credentials.h"

//define ssid and password from wifi.h file located in the include folder

const char* ssid = SSID;
const char* password = PASSWORD;

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

// const char* ssid = SSID;
// const char* password = PASSWORD

// Define two Serial devices mapped to the two internal UARTs
HardwareSerial MySerial0(0);
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

const int ppsPin = 2; // Define the PPS pin (change this to the pin you want to use)

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

    // Set the PPS pin as an output
    pinMode(ppsPin, OUTPUT);
    digitalWrite(ppsPin, LOW);

    // Set up the web server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = "<html><body>"
                      "<h1>GPS Simulation Parameters</h1>"
                      "<form action='/set_parameters' method='POST'>"
                      "Latitude:     <input type='text' name='lat' value='" + String(currentLat) + "'><br>"
                      "Longitude:    <input type='text' name='lon' value='" + String(currentLon) + "'><br>"
                      "Speed (km/h): <input type='text' name='speed' value='" + String(speed_kmh) + "'><br>"
                      "courseDeg:    <input type='text' name='courseDeg' value='" + String(courseDeg) + "'><br><br>"
                      "<input type='submit' value='Set Parameters'>"
                      "</form></body></html>";
        request->send(200, "text/html", html);
    });

    server.on("/set_parameters", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("lat", true) && request->hasParam("lon", true) && request->hasParam("speed", true)) {
            currentLat = request->getParam("lat", true)->value().toFloat();
            currentLon = request->getParam("lon", true)->value().toFloat();
            speed_kmh = request->getParam("speed", true)->value().toFloat();
            courseDeg = request->getParam("courseDeg", true)->value().toInt();
        }
        request->send(200, "text/html", "Parameters updated! <a href='/'>Go Back</a>");
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
    // Calculate the distance travelled in kilometers
    float distance_km = (speed_kmh / 3600.0) * (interval / 1000.0);
    
    // Convert the course direction from degrees to radians
    float courseRad = radians(courseDeg);

    // Calculate changes in latitude and longitude based on the course direction
    float deltaLat = distance_km * cos(courseRad) / 111.32; // Approximation: 1 degree of latitude is ~111.32 km
    float deltaLon = distance_km * sin(courseRad) / (111.32 * cos(radians(currentLat))); // Longitude distance depends on latitude

    // Update current latitude and longitude
    currentLat += deltaLat;
    currentLon += deltaLon;

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
