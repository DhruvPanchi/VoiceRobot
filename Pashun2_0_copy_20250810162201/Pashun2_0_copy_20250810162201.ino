#include <WiFi.h>
#include <WebServer.h>

// ─── Wi-Fi credentials ───────────────────────────────────────────
const char* ssid     = "PANCHIS@58";
const char* password = "P@nch1MAN";

// ─── Web-server setup ────────────────────────────────────────────
WebServer server(80);
String lastCommand = "";

// ─── FRONT Motor-Driver Pins ────────────────────────────────────
#define IN1_F 13
#define IN2_F 12
#define IN3_F 14
#define IN4_F 27
#define ENA_F 25
#define ENB_F 26

// ─── BACK Motor-Driver Pins ─────────────────────────────────────
#define IN1_B 18
#define IN2_B 19
#define IN3_B 21
#define IN4_B 22
#define ENA_B 23
#define ENB_B 5

// ─── Global speed (0-255) ───────────────────────────────────────
const uint8_t MOTOR_SPEED = 150;   // tweak for faster/slower spin

// ─── Helpers ─────────────────────────────────────────────────────
void setupMotors() {
  pinMode(IN1_F, OUTPUT); pinMode(IN2_F, OUTPUT);
  pinMode(IN3_F, OUTPUT); pinMode(IN4_F, OUTPUT);
  pinMode(ENA_F, OUTPUT); pinMode(ENB_F, OUTPUT);

  pinMode(IN1_B, OUTPUT); pinMode(IN2_B, OUTPUT);
  pinMode(IN3_B, OUTPUT); pinMode(IN4_B, OUTPUT);
  pinMode(ENA_B, OUTPUT); pinMode(ENB_B, OUTPUT);
}

void enableAll() {
  analogWrite(ENA_F, MOTOR_SPEED);
  analogWrite(ENB_F, MOTOR_SPEED);
  analogWrite(ENA_B, MOTOR_SPEED);
  analogWrite(ENB_B, MOTOR_SPEED);
}

void stopAll() {
  digitalWrite(IN1_F, LOW); digitalWrite(IN2_F, LOW);
  digitalWrite(IN3_F, LOW); digitalWrite(IN4_F, LOW);
  analogWrite (ENA_F, 0);   analogWrite (ENB_F, 0);

  digitalWrite(IN1_B, LOW); digitalWrite(IN2_B, LOW);
  digitalWrite(IN3_B, LOW); digitalWrite(IN4_B, LOW);
  analogWrite (ENA_B, 0);   analogWrite (ENB_B, 0);
}

// ─── Motions ─────────────────────────────────────────────────────
void moveForwardAllMotors() {
  // FRONT wheels forward
  digitalWrite(IN1_F, HIGH); digitalWrite(IN2_F, LOW);
  digitalWrite(IN3_F, HIGH); digitalWrite(IN4_F, LOW);
  // BACK wheels forward
  digitalWrite(IN1_B, HIGH); digitalWrite(IN2_B, LOW);
  digitalWrite(IN3_B, HIGH); digitalWrite(IN4_B, LOW);
  enableAll();
}

void moveBackwardAllMotors() {
  // FRONT wheels backward
  digitalWrite(IN1_F, LOW);  digitalWrite(IN2_F, HIGH);
  digitalWrite(IN3_F, LOW);  digitalWrite(IN4_F, HIGH);
  // BACK wheels backward
  digitalWrite(IN1_B, LOW);  digitalWrite(IN2_B, HIGH);
  digitalWrite(IN3_B, LOW);  digitalWrite(IN4_B, HIGH);
  enableAll();
}

void turnLeft() {            // pivot left
  // Left wheels backward
  digitalWrite(IN1_F, LOW);  digitalWrite(IN2_F, HIGH);
  digitalWrite(IN1_B, LOW);  digitalWrite(IN2_B, HIGH);
  // Right wheels forward
  digitalWrite(IN3_F, HIGH); digitalWrite(IN4_F, LOW);
  digitalWrite(IN3_B, HIGH); digitalWrite(IN4_B, LOW);
  enableAll();
}

void turnRight() {           // pivot right
  // Left wheels forward
  digitalWrite(IN1_F, HIGH); digitalWrite(IN2_F, LOW);
  digitalWrite(IN1_B, HIGH); digitalWrite(IN2_B, LOW);
  // Right wheels backward
  digitalWrite(IN3_F, LOW);  digitalWrite(IN4_F, HIGH);
  digitalWrite(IN3_B, LOW);  digitalWrite(IN4_B, HIGH);
  enableAll();
}

// ─── HTTP handler ───────────────────────────────────────────────
void handleCommand() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("cmd")) {
    lastCommand = server.arg("cmd");
    Serial.println("Received command: " + lastCommand);
  }
  server.send(200, "text/plain", "#done");
}

// ─── Arduino setup / loop ───────────────────────────────────────
void setup() {
  Serial.begin(115200);
  setupMotors();
  stopAll();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected! IP address:");
  Serial.println(WiFi.localIP());

  server.on("/command", HTTP_GET, handleCommand);
  server.begin();
  Serial.println("Server started.");
}

void loop() {
  server.handleClient();

  if (lastCommand != "") {
    if      (lastCommand == "Dhruv_forward")  moveForwardAllMotors();
    else if (lastCommand == "Dhruv_backward") moveBackwardAllMotors();
    else if (lastCommand == "Dhruv_right")    turnLeft();   // swapped!
    else if (lastCommand == "Dhruv_left")     turnRight();  // swapped!
    else                                       Serial.println("Unknown command: " + lastCommand);

    delay(500);          // run for ½ s
    stopAll();           // then brake
    lastCommand = "";    // clear
  }
}
