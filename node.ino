#include <ESP8266WiFi.h>
#include <AdafruitIO_WiFi.h>
#include <Servo.h>

// WiFi and Adafruit IO credentials
#define IO_USERNAME "YOUR_ADAFRUIT_IO_USERNAME"
#define IO_KEY "YOUR_ADAFRUIT_IO_KEY"
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// Servo configuration
Servo myServo;
const int servoPin = D5;  // Use D5 (GPIO 14) for MG996R signal line
const int stopPosition = 90;  // Stop position (~1500µs, calibrate if needed)
const int speedCW = 70;  // Slower clockwise speed (was 0 for full speed)
const int speedCCW = 180;  // Full speed counterclockwise (max torque, ~2400µs)
const int rotationTime = 250;  // Rotation time (1 second = 1000ms)
const int pauseTime = 400;  // Pause time (1 second = 1000ms)

// Button configuration
const int buttonPin = D6;  // Button connected to D6 (GPIO 12)
bool buttonPressed = false;
bool lastButtonState = HIGH;  // Assuming pull-up resistor
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // 50ms debounce delay

// Relay configuration
const int relay1Pin = D1;  // Relay 1 connected to D1 (GPIO 5)
const int relay2Pin = D2;  // Relay 2 connected to D2 (GPIO 4)
const int relay3Pin = D7;  // Relay 3 connected to D7 (GPIO 13)
const int relay4Pin = D0;  // Relay 4 connected to D0 (GPIO 16)

// Spray cycle state variables
bool sprayActive = false;      // Track if spray cycle is active
int sprayStep = 0;             // Current step in spray cycle (0=idle, 1=CW, 2=pause1, 3=CCW, 4=pause2)
unsigned long sprayStartTime = 0;  // Track when current step started
unsigned long bootTime = 0;   // Track boot time for startup delay

// Adafruit IO setup
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *command = io.feed("command");

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize button with internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
  
  // Initialize relay pins as outputs (LOW = relay off)
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);
  pinMode(relay4Pin, OUTPUT);
  digitalWrite(relay1Pin, HIGH);  // Ensure all relays start OFF
  digitalWrite(relay2Pin, HIGH);
  digitalWrite(relay3Pin, HIGH);
  digitalWrite(relay4Pin, HIGH);
  
  // Initialize servo
  myServo.attach(servoPin);
  myServo.write(stopPosition);  // Ensure servo is stopped initially
  delay(1000);  // Wait for stabilization
  
  // Connect to Adafruit IO
  io.connect();
  command->onMessage(handleMessage);
  
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
    yield();  // Feed the watchdog during connection
  }
  
  Serial.println("Connected to Adafruit IO!");
  Serial.println("Servo initialized and ready for spray commands");
  bootTime = millis();  // Record boot time for startup delay
}

// Function to read button with debouncing
bool readButton() {
  bool reading = digitalRead(buttonPin);
  
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // If the reading has been stable for debounceDelay milliseconds
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed:
    if (reading != buttonPressed) {
      buttonPressed = reading;
      
      // Return true only on button press (HIGH to LOW transition with pull-up)
      if (buttonPressed == LOW) {
        lastButtonState = reading;
        return true;
      }
    }
  }
  
  lastButtonState = reading;
  return false;
}

void handleMessage(AdafruitIO_Data *data) {
  String cmd = data->toString();
  Serial.println("Received command: " + cmd);
  
  // Prevent command execution for 3 seconds after boot to let system stabilize
  if (millis() - bootTime < 3000) {
    Serial.println("System stabilizing... Command ignored for 3 seconds after boot");
    return;
  }
  
  // Handle SPRAY command (existing functionality)
  if (cmd == "SPRAY" && !sprayActive) {
    digitalWrite(LED_BUILTIN, LOW);  // Turn on LED as indicator
    Serial.println("LED turned ON - Starting spray cycle");
    startSprayCycle();  // Starts clockwise rotation
  }
  
  // Handle relay control commands
  else if (cmd == "SWITCHON1") {
    digitalWrite(relay1Pin, LOW);
    Serial.println("Relay 1 turned ON");
  }
  else if (cmd == "SWITCHOFF1") {
    digitalWrite(relay1Pin, HIGH);
    Serial.println("Relay 1 turned OFF");
  }
  else if (cmd == "SWITCHON2") {
    digitalWrite(relay2Pin, LOW);
    Serial.println("Relay 2 turned ON");
  }
  else if (cmd == "SWITCHOFF2") {
    digitalWrite(relay2Pin, HIGH);
    Serial.println("Relay 2 turned OFF");
  }
  else if (cmd == "SWITCHON3") {
    digitalWrite(relay3Pin, LOW);
    Serial.println("Relay 3 turned ON");
  }
  else if (cmd == "SWITCHOFF3") {
    digitalWrite(relay3Pin, HIGH);
    Serial.println("Relay 3 turned OFF");
  }
  else if (cmd == "SWITCHON4") {
    digitalWrite(relay4Pin, LOW);
    Serial.println("Relay 4 turned ON");
  }
  else if (cmd == "SWITCHOFF4") {
    digitalWrite(relay4Pin, HIGH);
    Serial.println("Relay 4 turned OFF");
  }
  else {
    Serial.println("Unknown command: " + cmd);
  }
}

void startSprayCycle() {
  sprayActive = true;
  sprayStep = 1;  // Set to step 1 (clockwise phase)
  sprayStartTime = millis();
  
  myServo.write(speedCW);  // Start clockwise rotation
  Serial.println("=== Starting Spray Cycle ===");
  Serial.println("Clockwise, pulse: " + String(myServo.readMicroseconds()) + "µs");
}

void updateSprayCycle() {
  if (!sprayActive) return;
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - sprayStartTime;
  
  // Check for button press
  bool buttonClicked = readButton();
  yield();  // Feed the watchdog during spray cycle
  
  switch (sprayStep) {
    case 1:  // Clockwise rotation - stop on button press
      if (buttonClicked) {
        myServo.write(stopPosition);  // Stop only when button pressed
        Serial.println("Button pressed! Stopped");
        sprayStep = 2;  // Move to pause phase
        sprayStartTime = currentTime;
      }
      break;
      
    case 2:  // First pause - use timing
      if (elapsed >= pauseTime) {
        myServo.write(speedCCW);
        Serial.println("Counterclockwise, pulse: " + String(myServo.readMicroseconds()) + "µs");
        sprayStep = 3;
        sprayStartTime = currentTime;
      }
      break;
      
    case 3:  // Counterclockwise rotation - run for 1 second automatically
      if (elapsed >= rotationTime) {  // Run for exactly 1 second
        myServo.write(stopPosition);
        Serial.println("1 second complete! Stopped counterclockwise, pulse: " + String(myServo.readMicroseconds()) + "µs");
        sprayStep = 4;
        sprayStartTime = currentTime;
      }
      break;
      
    case 4:  // Final pause - use timing
      if (elapsed >= pauseTime) {
        // Cycle complete
        sprayActive = false;
        sprayStep = 0;
        digitalWrite(LED_BUILTIN, HIGH);  // Turn off LED
        Serial.println("=== Spray Cycle Complete ===");
        Serial.println("LED turned OFF - Ready for next command");
      }
      break;
  }
  yield();  // Feed the watchdog after spray cycle operations
}

void executeSprayCycle() {
  Serial.println("=== Starting Spray Cycle ===");
  
  // Rotate clockwise for 1 second (high torque)
  myServo.write(speedCW);
  Serial.println("Clockwise, pulse: " + String(myServo.readMicroseconds()) + "µs");
  delay(rotationTime);
  
  // Stop for 1 second
  myServo.write(stopPosition);
  Serial.println("Stopped, pulse: " + String(myServo.readMicroseconds()) + "µs");
  delay(pauseTime);
  
  // Rotate counterclockwise for 1 second (high torque)
  myServo.write(speedCCW);
  Serial.println("Counterclockwise, pulse: " + String(myServo.readMicroseconds()) + "µs");
  delay(rotationTime);
  
  // Stop for 1 second
  myServo.write(stopPosition);
  Serial.println("Stopped, pulse: " + String(myServo.readMicroseconds()) + "µs");
  delay(pauseTime);
  
  Serial.println("=== Spray Cycle Complete ===");
}

void loop() {
  io.run();
  updateSprayCycle();  // Handle spray cycle with button control
  yield();  // Feed the watchdog
  delay(50);  // Reduced delay for better button responsiveness
}