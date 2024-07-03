#include "driver/timer.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "DHT.h"
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Keypad_I2C.h>
#include <ESP32Servo.h>
#include "ESP32_MailClient.h"


// Replace with your network credentials
const char *ssid = "Family";
const char *password = "88888888";

// Auxiliar variables to store the current output state
String output32State ;

// Assign output variables to GPIO pins
const int output32 = 33;
#define RELAY_NO true

// Set number of relays
#define NUM_RELAYS 5

// Assign each GPIO to a relay
int relayGPIOs[NUM_RELAYS] = {2, 26, 27, 25, 33};

const char* PARAM_INPUT_1 = "relay";
const char* PARAM_INPUT_2 = "state";




// Initialize AsyncWebServer object on port 80
AsyncWebServer server(80);
// Email credentials
#define emailSenderAccount    "21010868@st.phenikaa-uni.edu.vn" // Thay bằng email của bạn
#define emailSenderPassword   "0901772524p"       // Thay bằng mật khẩu của bạn
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "ALERT! Door Unlocked" // Tiêu đề email
#define emailSubject          "ALARM!FIRE DETECTED"

String inputMessage = "21010868@st.phenikaa-uni.edu.vn";   // Email người nhận thông báo

SMTPData smtpData;

#define DPIN  13   // Pin to connect DHT sensor (GPIO number)
#define DTYPE DHT22   // Define DHT 11 or DHT22 sensor type
#define SERVO_PIN 15       // ESP32 pin GPIO15 connected to servo motor
#define SERVO_PIN_2 16  // ESP32 pin GPIO16 connected to the second servo motor
#define RAIN_SENSOR_PIN  2 // The ESP32 pin GPIO2 connected to the rain sensor
#define I2CADDR 0x20
#define SENSOR_GAS 36 // ESP32's pin GPIO36 connected to AO pin of the MQ2 sensor
#define BUZZER 18 // The ESP32 pin GPIO18 connected to relay

#define ROW_NUM 4 // four rows
#define COLUMN_NUM 4 // four columns
#define BUZZER 18 // The ESP32 pin GPIO18 connected to relay
#define AO_PIN 36 // ESP32's pin GPIO36 connected to AO pin of the MQ2 sensor
#define L298_IN1 26
#define L298_IN2 27
#define L298_ENA 14

const int relay1Pin = 4;  // GPIO 4 for relay 1
const int relay2Pin = 5;  // GPIO 5 for relay 2
const int relay3Pin = 6;  // GPIO 6 for relay 3
const int relay4Pin = 7;  // GPIO 7 for relay 4


const float Kp_value = 28;
const float Kd_value = 10;
const float Ki_value = 0.01;
int angle = 0;              // Current angle of servo motor
int prev_rain_state;        // Previous state of rain sensor
int rain_state;             // Current state of rain sensor
bool active_pid = false;  // Initial state of active_pid
String doorStatus = "Locked";
String ALARM="SAFETY";


Servo myServo;
Servo myServo2;  // Khai báo Servo thứ hai

float nhietdodat = 40;  // Desired temperature
float nhietdo = 0;
float doam=0;

float E, E1, E2, alpha, gamma_value, beta;

float T = 1.5;
float Output = 0;
float LastOutput = 0;



unsigned long lastGasCheckTime = 0; // Last time gas was checked
const unsigned long gasCheckInterval = 1000; // Interval for gas check in milliseconds

unsigned long lastRainCheckTime = 0; // Last time rain was checked
const unsigned long rainCheckInterval = 1000; // Interval for rain check in milliseconds

unsigned long lastDoorLockTime = 0; // Last time door lock was checked
const unsigned long doorLockInterval = 500; // Interval for door lock check in milliseconds

unsigned long lastPIDUpdateTime = 0; // Last time PID update was checked
const unsigned long PIDUpdateInterval = 500; // Interval for PID update in milliseconds

bool relayOpen = false; // flag to check if relay is currently open
unsigned long relayOpenTime = 0; // variable to store the time when relay is opened

String input_password;
const String password_1 = "1234"; // change your password here
const String password_2 = "4444"; // change your password here
const String password_3 = "55555"; // change your password here

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool newTemperatureAvailable = false;
////////////////////




char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte pin_column[COLUMN_NUM] = {3, 2, 1, 0}; // connect to the row pinouts of the keypad
byte pin_rows[ROW_NUM] = {7, 6, 5, 4}; // connect to the column pinouts of the keypad

Keypad_I2C kpd(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM, I2CADDR, PCF8574);
LiquidCrystal_I2C lcd(0x3F, 16, 2); // Khai báo địa chỉ I2C (0x27 or 0x3F) và LCD 16x02

DHT dht(DPIN, DTYPE);  // Declare DHT object with pin and type

void IRAM_ATTR onTimer(); // Forward declaration

bool sendEmailNotification(String emailMessage) {
  // Cấu hình thông tin SMTP
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);
  smtpData.setSender("ESP32_Door_Alert", emailSenderAccount);
  smtpData.setPriority("High");
  smtpData.setSubject(emailSubject);
  smtpData.setMessage(emailMessage, true);
  smtpData.addRecipient(inputMessage);

  if (!MailClient.sendMail(smtpData)) {
    Serial.println("Error sending Email, " + MailClient.smtpErrorReason());
    return false;
  }
  smtpData.empty();
  return true;
}


String relayState(int numRelay) {
  if (RELAY_NO) {
    if (digitalRead(relayGPIOs[numRelay - 1])) {
      return "";
    } else {
      return "checked";
    }
  } else {
    if (digitalRead(relayGPIOs[numRelay - 1])) {
      return "checked";
    } else {
      return "";
    }
  }
}



String generateHTML() {
    String html = "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<link rel=\"icon\" href=\"data:,\">";
    html += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}";
    html += ".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;";
    html += "text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}";
    html += ".button2 {background-color: #555555;}</style>";
    html += "<link rel=\"stylesheet\" href=\"https://cdnjs.cloudflare.com/ajax/libs/font-awesome/5.15.4/css/all.min.css\">";
    html += "</head>";
    html += "<body><h1>SMART HOME WEB SERVER</h1>";

    // Display temperature and humidity
    html += "<h2>Environment Data</h2>";
    html += "<p><i class=\"fas fa-thermometer-half\" style=\"color:#ff0000;\"></i> Temperature: <span id=\"temperature\">--</span> &deg;C</p>";
    html += "<p><i class=\"fas fa-thermometer-half\" style=\"color:#00add6;\"></i> Humidity: <span id=\"humidity\">--</span> %</p>";

    // Door and alarm status display
    html += "<h2>Status</h2>";
    html += "<p><i class=\"fas fa-door-open\" style=\"color:#000;\"></i> Door Status: <span id=\"doorStatus\"></span></p>";
    html += "<p><i class=\"fas fa-exclamation-circle\" style=\"color:#ff0000;\"></i> ALARM Status: <span id=\"alarmStatus\"></span></p>";

    // Temperature control section
    html += "<h2>Temperature Control</h2>";
    html += "<p>NHIET DO DAT: <span id=\"temperatureValue\">" + String(nhietdodat, 1) + "</span> &deg;C</p>";
    html += "<input type=\"range\" min=\"0\" max=\"100\" value=\"" + String(nhietdodat, 1) + "\" class=\"slider\" id=\"temperatureSlider\" oninput=\"updateTemperature(this.value)\">";

    // CSS for switches and sliders
    html += "<style>.switch { position: relative; display: inline-block; width: 120px; height: 68px; }";
    html += ".switch input { display: none; }";
    html += ".slider { -webkit-appearance: none; appearance: none; width: 100%; height: 15px; border-radius: 5px; background: #d3d3d3; outline: none; opacity: 0.7; transition: opacity .15s ease-in-out;}";
    html += ".slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 25px; height: 25px; border-radius: 50%; background: #4CAF50; cursor: pointer; }";
    html += ".slider::-moz-range-thumb { width: 25px; height: 25px; border-radius: 50%; background: #4CAF50; cursor: pointer; }</style>";

    // Loop through each relay to create toggle switches
    for (int i = 1; i <= NUM_RELAYS; i++) {
        String relayStateValue = relayState(i);
        html += "<h2>Relay #" + String(i) + " - GPIO " + relayGPIOs[i-1] + "</h2>";
        html += "<label class=\"switch\"><input type=\"checkbox\" id=\"relay" + String(i) + "\" onchange=\"toggleCheckbox(this)\">";
        html += "<span class=\"slider\"></span></label>";
        // Set initial state based on relayStateValue (checked/unchecked)
        if (relayStateValue == "checked") {
            html += "<script>document.getElementById('relay" + String(i) + "').checked = true;</script>";
        }
    }

    // JavaScript functions for AJAX requests
    html += "<script>function toggleCheckbox(element) {";
    html += "var relayId = element.id.substring(5);"; // Extract relay number from id
    html += "var xhr = new XMLHttpRequest();";
    html += "if (element.checked) { xhr.open('GET', '/update?relay=' + relayId + '&state=1', true); }";
    html += "else { xhr.open('GET', '/update?relay=' + relayId + '&state=0', true); }";
    html += "xhr.send();";
    html += "}";
    
    html += "function controlGPIO(url) {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.open('GET', url, true);";
    html += "xhr.send();";
    html += "}";
    
    html += "function updateTemperature(value) {";
    html += "document.getElementById('temperatureValue').innerText = value;";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.open('GET', '/setTemperature?value=' + value, true);";
    html += "xhr.send();";
    html += "}";

    // AJAX request to update temperature and humidity
    html += "setInterval(function() {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.open('GET', '/dht', true);";
    html += "xhr.onreadystatechange = function() {";
    html += "if (xhr.readyState == 4 && xhr.status == 200) {";
    html += "var dhtData = JSON.parse(xhr.responseText);";
    html += "document.getElementById('temperature').innerText = dhtData.temperature;";
    html += "document.getElementById('humidity').innerText = dhtData.humidity;";
    html += "}";
    html += "};";
    html += "xhr.send();";
    html += "}, 1000);"; // Update every second

    // AJAX request to update door status
    html += "setInterval(function() {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.open('GET', '/doorStatus', true);";
    html += "xhr.onreadystatechange = function() {";
    html += "if (xhr.readyState == 4 && xhr.status == 200) {";
    html += "var doorData = JSON.parse(xhr.responseText);";
    html += "document.getElementById('doorStatus').innerHTML = doorData.doorStatus;";
    html += "}";
    html += "};";
    html += "xhr.send();";
    html += "}, 1000);"; // Update every second

    // AJAX request to update ALARM status
    html += "setInterval(function() {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.open('GET', '/alarmStatus', true);";
    html += "xhr.onreadystatechange = function() {";
    html += "if (xhr.readyState == 4 && xhr.status == 200) {";
    html += "var alarmData = JSON.parse(xhr.responseText);";
    html += "document.getElementById('alarmStatus').innerText = alarmData.alarmStatus;";
    html += "}";
    html += "};";
    html += "xhr.send();";
    html += "}, 1000);"; // Update every second

    html += "</script>";
    html += "</body></html>";

    return html;
}









void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_GAS, INPUT);

  pinMode(BUZZER, OUTPUT);
  pinMode(L298_IN1, OUTPUT);
  pinMode(L298_IN2, OUTPUT);
  pinMode(L298_ENA, OUTPUT);

  dht.begin();  // Initialize DHT sensor
  E = 0;
  E1 = 0;
  E2 = 0;
  T = 1.5;
  

  timer = timerBegin(0, 80, true);  // timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, T * 1000000, true);  // T seconds in microseconds
  timerAlarmEnable(timer);  // Enable timer alarm

  input_password.reserve(32); // maximum input characters is 33, change if needed
  lcd.init(); // initialize the lcd
  kpd.begin(makeKeymap(keys));


  myServo.attach(SERVO_PIN);  // Attaches the servo to the ESP32 pin
  myServo.write(0);           // Initialize servo to 0 degrees
   myServo2.attach(SERVO_PIN_2);
  myServo2.write(0); // Servo thứ hai ở góc 0 độ ban đầu
  Serial.println("Servo motor control started.");
  pinMode(RAIN_SENSOR_PIN, INPUT); // Initialize the rain sensor pin as an input
  rain_state = digitalRead(RAIN_SENSOR_PIN);
  prev_rain_state = rain_state;    // Initialize the previous state

  for (int i = 1; i <= NUM_RELAYS; i++) {
    pinMode(relayGPIOs[i - 1], OUTPUT);
    if (RELAY_NO) {
      digitalWrite(relayGPIOs[i - 1], HIGH);
    } else {
      digitalWrite(relayGPIOs[i - 1], LOW);
    }
  }
  Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // Print local IP address
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  server.on("/32/on", HTTP_GET, [](AsyncWebServerRequest *request) {
        digitalWrite(output32, HIGH);
        output32State = "on";
        active_pid = true;
        request->send(200, "text/plain", "GPIO 32 on");
    });
    server.on("/32/off", HTTP_GET, [](AsyncWebServerRequest *request) {
        digitalWrite(output32, LOW);
        output32State = "off";
        active_pid = false;
        // Stop the motor
        ledcWrite(0, 0);  // Set PWM to 0
        digitalWrite(L298_IN1, LOW);  // Stop motor
        digitalWrite(L298_IN2, LOW);  // Stop motor
        request->send(200, "text/plain", "GPIO 32 off");
    });

    // Serve HTML page with buttons, slider and DHT data
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = generateHTML();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
        response->addHeader("Cache-Control", "no-store"); // Disable caching
        request->send(response);
    });

    // Endpoint to get DHT sensor data
    server.on("/dht", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{\"temperature\":" + String(nhietdo, 1) + ", \"humidity\":" + String(doam, 1) + "}";
        request->send(200, "application/json", json);
    });

    // Endpoint to set desired temperature
    server.on("/setTemperature", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            nhietdodat = request->getParam("value")->value().toFloat();
            request->send(200, "text/plain", "Temperature set to " + String(nhietdodat, 1) + " &deg;C");
        } else {
            request->send(400, "text/plain", "Invalid request");
        }
    });
    server.on("/alarmStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"alarmStatus\":\"" + ALARM + "\"}";
    request->send(200, "application/json", json);
});

// Register web server endpoints
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", generateHTML());
  });

  server.on("/doorStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"doorStatus\":\"" + doorStatus + "\"}";
    request->send(200, "application/json", json);
  });

// Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", generateHTML());
  });

  // Send a GET request to <ESP_IP>/update?relay=<inputMessage>&state=<inputMessage2>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    String inputMessage2;
    String inputParam2;
    // GET input1 value on <ESP_IP>/update?relay=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1) & request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      inputParam2 = PARAM_INPUT_2;
      if (RELAY_NO) {
        Serial.print("NO ");
        digitalWrite(relayGPIOs[inputMessage.toInt() - 1], !inputMessage2.toInt());
      } else {
        Serial.print("NC ");
        digitalWrite(relayGPIOs[inputMessage.toInt() - 1], inputMessage2.toInt());
      }
    } else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.println(inputMessage + inputMessage2);
    request->send(200, "text/plain", "OK");
  });


   
    server.begin();
}

void loop() {
  unsigned long currentMillis = millis();

  // Kiểm tra và cập nhật trạng thái của cửa khi có sự kiện nhấn phím
  char key = kpd.getKey();
  if (key) {
    Serial.println(key);

    if (key == '*') {
      input_password = ""; // Xóa mật khẩu đang nhập
      lcd.clear();
    } else {
      if (input_password.length() == 0) {
        lcd.clear();
      }
      input_password += key; // Thêm ký tự mới vào mật khẩu đang nhập
      lcd.setCursor(input_password.length(), 0); // Di chuyển con trỏ đến vị trí mới
      lcd.print('*'); // In ký tự '*' thay thế cho ký tự nhập vào
    }
  }

  // Kiểm tra và điều khiển cửa
  if (currentMillis - lastDoorLockTime >= doorLockInterval) {
    lastDoorLockTime = currentMillis;
    system_door_lock();
  }

  // Kiểm tra và điều khiển cảm biến MQ2
  if (currentMillis - lastGasCheckTime >= gasCheckInterval) {
    lastGasCheckTime = currentMillis;
    phongbep();
  }

  // Kiểm tra và điều khiển cảm biến mưa và servo
  if (currentMillis - lastRainCheckTime >= rainCheckInterval) {
    lastRainCheckTime = currentMillis;
    gianphoi();
  }

  // Kiểm tra và cập nhật nhiệt độ và điều khiển PID
  // Điều khiển servo và xử lý ngắt
    if (newTemperatureAvailable) {
        portENTER_CRITICAL(&timerMux);
        newTemperatureAvailable = false;
        portEXIT_CRITICAL(&timerMux);

        // Đọc nhiệt độ và độ ẩm từ DHT
        Temperature();

        // Thực hiện vòng lặp PID nếu active_pid là true
        if (active_pid) {
            PID();
        } else {
            // Dừng động cơ nếu PID không active
            Output = 0;
            digitalWrite(L298_IN1, LOW);
            digitalWrite(L298_IN2, LOW);
        }

        // Ghi giá trị PWM output, temperature và active_pid ra Serial Plotter
        Serial.print("Temperature: ");
        Serial.print(nhietdo);
        Serial.print(" | PWM Output: ");
        Serial.print(Output);
        Serial.print(" | active_pid: ");
        Serial.println(active_pid);
    }
}

void system_door_lock() {
  // Xử lý mật khẩu khi nhấn phím '#'
if (input_password.endsWith("#")) {
      lcd.clear();
      // Loại bỏ ký tự '#' từ mật khẩu
      input_password = input_password.substring(0, input_password.length() - 1);

      // Kiểm tra mật khẩu
      if (input_password == password_1 || input_password == password_2 || input_password == password_3) {
        Serial.println("Valid Password => unlock the door");

        lcd.setCursor(0, 0);
        lcd.print("CORRECT!");
        lcd.setCursor(0, 1);
        lcd.print("DOOR UNLOCKED!");

        myServo2.write(180); // Move secondary servo to 180 degrees
        relayOpenTime = millis(); // Lưu thời gian hiện tại
        doorStatus = "Unlocked";  // Update door status
        relayOpen = true; // Đặt cờ mở cửa
        
        // Gửi email thông báo
        String emailMessage = "Door has been unlocked by correct password.";
        if (sendEmailNotification(emailMessage)) {
          Serial.println("Email sent successfully");
        } else {
          Serial.println("Failed to send email");
        }

      } else {
        Serial.println("Invalid Password => Try again");
        doorStatus = "Locked";  // Update door status

        lcd.setCursor(0, 0);
        lcd.print("INCORRECT!");
        lcd.setCursor(0, 1);
        lcd.print("ACCESS DENIED!");
      }

      // Xóa mật khẩu sau khi xử lý
      input_password = "";
    }

  // Đóng relay sau 10 giây mở cửa
  if (relayOpen && millis() - relayOpenTime >= 10000) {
     myServo2.write(0); // Move secondary servo back to 0 degrees
    relayOpen = false; // Đặt lại cờ
     doorStatus = "Locked";  // Update door status
    Serial.println("Relay closed after 10 seconds");
  }
}
void phongbep() {
  int gasValue = analogRead(AO_PIN);

  if (gasValue >= 400) {
    digitalWrite(BUZZER, HIGH); // turn on
    ALARM="DANGEROUS";
    
    // Send email notification
    String emailMessage = "ALERT! Fire detected. Gas sensor value: " + String(gasValue);
    if (sendEmailNotification(emailMessage)) {
      Serial.println("Fire alert email sent successfully");
    } else {
      Serial.println("Failed to send fire alert email");
    }

  } else {
    digitalWrite(BUZZER, LOW); // turn off
    ALARM="SAFETY";
  }
  Serial.print("MQ2 sensor AO value: ");
  Serial.println(gasValue);
}
void gianphoi() {
  prev_rain_state = rain_state;             // Save the last state
  rain_state = digitalRead(RAIN_SENSOR_PIN); // Read new state

  if (rain_state == LOW && prev_rain_state == HIGH) { // Pin state change: HIGH -> LOW (rain detected)
    Serial.println("Rain detected!");
    myServo.write(180); // Move servo to 180 degrees
  } else if (rain_state == HIGH && prev_rain_state == LOW) { // Pin state change: LOW -> HIGH (rain stopped)
    Serial.println("Rain stopped!");
    myServo.write(0); // Move servo back to 0 degrees
  }
}


void Temperature() {
    nhietdo = dht.readTemperature(false);  // Read temperature in C
    doam = dht.readHumidity();  // Read humidity
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  newTemperatureAvailable = true; // Flag to signal temperature reading
  portEXIT_CRITICAL_ISR(&timerMux);
}

void PID() {
  E = nhietdodat - nhietdo;
  alpha = 2 * T * Kp_value + Ki_value * T * T + 2 * Kd_value;
  beta = T * T * Ki_value - 4 * Kd_value - 2 * T * Kp_value;
  gamma_value = 2 * Kd_value;
  Output = (alpha * E + beta * E1 + gamma_value * E2 + 2 * T * LastOutput) / (2 * T);

  LastOutput = Output;
  E2 = E1;
  E1 = E;

  if (Output > 255) {
    Output = 255;
  } else if (Output < 0) {
    Output = 0;
  }

  analogWrite(L298_ENA, Output);
  digitalWrite(L298_IN1, HIGH);  // Set direction
  digitalWrite(L298_IN2, LOW);   // Set direction

} 