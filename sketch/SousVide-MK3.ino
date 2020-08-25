#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AutoPID.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <AutoConnect.h>
#include <BlynkSimpleEsp32.h>
#include "SPIFFS.h"

String version = "v3.1";

#define LED_BUILTIN 2

WebServer Server;
AutoConnect portal(Server);
AutoConnectConfig config;

//!MARK: custom page
const char blynk_aux[] PROGMEM = R"raw(
[
	{
		"title": "Blynk Config",
		"uri": "/blynk-config",
		"menu": true,
		"element": [
			{
				"name": "auth",
				"type": "ACInput",
				"value": "",
				"label": "Auth Key"
			},
			{
				"name": "host",
				"type": "ACInput",
				"value": "blynk-cloud.com",
				"label": "Host",
				"placeholder": "blynk-cloud.com"
			},
			{
				"name": "port",
				"type": "ACInput",
				"value": "80",
				"label": "Port",
				"placeholder": "80"
			},
			{
				"name": "save",
				"type": "ACSubmit",
				"value": "SAVE",
				"uri": "/blynk-save"
			}
		]
	},
	{
		"title": "Blynk Config",
		"uri": "/blynk-save",
		"menu": false,
		"element": [
			{
				"name": "header",
				"type": "ACText",
				"value": "<h4>SAVED!</h4>"
			},
			{
				"name": "caption",
				"type": "ACText",
				"value": "Please restart for changes to take affect."
			}
		]
	}
]
)raw";

bool blynked = false;
BlynkTimer blynk_timer;
WidgetLED blynk_led(V4);

OneWire oneWire(27);
DallasTemperature thermometers(&oneWire);

#ifdef __cplusplus
extern "C" {
#endif

	uint8_t temprature_sens_read();
 
#ifdef __cplusplus
}
#endif
 
uint8_t temprature_sens_read();

struct Encoder {
	const uint8_t CLK;
	const uint8_t DT;
	const uint8_t SW;
};

Encoder encoder = {32, 35, 34};

volatile boolean activity_detected = true;
int currentStateCLK;
int lastStateCLK;
volatile int encoder_direction = 0;
double currentTemperature = 0.0;
static double targetTemperature = 60.0;

bool powerState = LOW;

//!MARK: PID setup

int relay_pin = 23;
bool relay_state = LOW;

#define PULSEWIDTH 2500
double kP = 0.3;
double kI = 0.001;
double kD = kI / 2;
AutoPIDRelay myPID(&currentTemperature, &targetTemperature, &relay_state, PULSEWIDTH, kP, kI, kD);

unsigned long previousMillis = 0;
int screenOffset = 0;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void IRAM_ATTR encoder_turning() {
	currentStateCLK = digitalRead(encoder.CLK);

	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
		if (digitalRead(encoder.DT) != currentStateCLK) {
			encoder_direction = 1;
		}
		else {
			encoder_direction = 2;
		}

		activity_detected = true;
	}

	lastStateCLK = currentStateCLK;
}

void IRAM_ATTR encoder_switched() {
	powerState = !powerState;

	/*For some reason the line below causes
		Guru Meditation Error: Core  1 panic'ed (Interrupt wdt timeout on CPU1)
		Low priority*/
//	if (blynked) { 
//		Blynk.virtualWrite(V3, powerState);
//	}
}

void setup(){
	Serial.begin(115200);
	Serial.println();
  
	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
		Serial.println(F("SSD1306 allocation failed"));
		for(;;); // Don't proceed, loop forever
	}

	display.clearDisplay();
	display.setTextWrap(false);
	display.setTextColor(WHITE, BLACK);
	
	display.setTextSize(1);
	display.setCursor(128-5*(version.length()+2), 64-8);
	display.print(version);

  thermometers.begin();

	myPID.setBangBang(4);
  myPID.setTimeStep(4000);
	pinMode(relay_pin, OUTPUT);
	digitalWrite(relay_pin, relay_state);

	pinMode(LED_BUILTIN, OUTPUT);

  pinMode (encoder.CLK, INPUT);
  pinMode (encoder.DT, INPUT);
  pinMode (encoder.SW, INPUT_PULLUP);

	attachInterrupt (encoder.CLK,encoder_turning,CHANGE);
	attachInterrupt (encoder.SW,encoder_switched,FALLING);

  captive_setup();
}

void loop() {
	thermometers.requestTemperatures();
	currentTemperature = thermometers.getTempCByIndex(0);
	if(currentTemperature < 0) currentTemperature = 0;

	// encoder
	if (activity_detected)  {       // do this only if rotation was detected
		if (encoder_direction == 1) {
			targetTemperature += 0.5;
			if (targetTemperature >= 99) targetTemperature = 99;
		}
		else if (encoder_direction == 2) {
			targetTemperature -= 0.5;
			if (targetTemperature <= 0) targetTemperature = 0;
		}
		activity_detected = false; // reset
		encoder_direction = 0;

		if (blynked) {
			Blynk.virtualWrite(V2, targetTemperature);
		}
	}

	if(powerState) {
		myPID.run();
	}
	else {
    myPID.stop();
    relay_state = LOW;
	}
  digitalWrite(relay_pin, relay_state);
	digitalWrite(LED_BUILTIN, relay_state);

	updateScreen();

	captive_loop();
}

void updateScreen() {
	display.setTextSize(2);
	display.setCursor(0, 16);
	display.print(currentTemperature, 1);
	display.print("   "); // flush remaining chars in case of error reading the sensor.

	unsigned long currentMillis = millis();
	if(currentMillis - previousMillis >= 250) { //250ms delay

		display.fillRect(0, 34, 128, 18, BLACK); // clear
		if(powerState) {
			int offset = screenOffset;
			for (int i=0; i<5; i++) {
				display.fillTriangle(offset, 40+8, offset, 40, offset+8, 40+(8/2), WHITE);
				offset += 8+4;
			}
			screenOffset += 3;
			if(screenOffset >= 80) screenOffset = 0;
		}
		else {
			screenOffset = 0;
		}
		previousMillis = currentMillis;
	}

	display.setCursor(128-(10*5), 16);
	display.print(targetTemperature, 1);
	
	display.fillRect(0, 0, 128, 16, BLACK); // clear the bar
	display.drawRect(0, 0, 128, 14, WHITE); // draw outter box

	// fill box until current temp
	int x = map(currentTemperature*10, 0, 990, 0, 124);
	display.fillRect(2, 2, x, 10, WHITE);

	// target temperature line
	int y = map(targetTemperature*10, 0, 990, 0, 124);
	display.drawFastVLine(y, 0, 16, WHITE);

	display.display();
}

//!MARK: Captive

void captive_setup() {
	Server.on("/", captive_rootPage);

	portal.load(blynk_aux);
	portal.on("/blynk-save", captive_blynk_save, AC_EXIT_AHEAD);

	config.ota = AC_OTA_BUILTIN;
	config.apid = "SV-" + String((uint32_t)(ESP.getEfuseMac() >> 32), HEX);
	config.hostName = "katara";
	config.psk = "SousVide";
	config.portalTimeout = 1;
	config.retainPortal = true;
	portal.config(config);

	display.setCursor(0, 64-8);
  display.print("Connect WiFi!");
  display.display();
  
  if (portal.begin()) {
    Serial.println("WiFi connected: " + WiFi.localIP().toString());

    display.setTextSize(1);
		display.setCursor(0, 64-8);
		display.print(WiFi.localIP().toString());
		display.display();

		SPIFFS.begin(true);

		if(!SPIFFS.exists("/blynk-config.json")) {
			SPIFFS.end();
			return;
		}
		
		File file = SPIFFS.open("/blynk-config.json", FILE_READ);

		AutoConnectAux* auxPage = portal.aux("/blynk-config");
		auxPage->loadElement(file, { "auth", "host", "port" });
		
		AutoConnectInput& input_auth = auxPage->getElement<AutoConnectInput>("auth");
		int strlen = input_auth.value.length() + 1;
		char bl_auth[strlen];
		input_auth.value.toCharArray(bl_auth, strlen);

		AutoConnectInput& input_host = auxPage->getElement<AutoConnectInput>("host");
		strlen = input_host.value.length() + 1;
		char bl_host[strlen];
		input_host.value.toCharArray(bl_host, strlen);

		AutoConnectInput& input_port = auxPage->getElement<AutoConnectInput>("port");
		int bl_port = input_port.value.toInt();
		
		file.close();
		SPIFFS.end();

		if (strcmp(bl_auth, "") != 0) {
			Blynk.config(bl_auth, bl_host, bl_port);
			blynked = Blynk.connect();
	
			if (blynked) {
				blynk_timer.setInterval(1000L, updateBlynk);
			}
		}
  }
}

void captive_loop() {
    portal.handleClient();
    if (blynked) {
    	Blynk.run();
    	blynk_timer.run();
    }
}


String captive_blynk_save(AutoConnectAux& aux, PageArgument& args) {	
	SPIFFS.begin();
	File file = SPIFFS.open("/blynk-config.json", FILE_WRITE);

	AutoConnectAux* auxPage = portal.aux("/blynk-config");
	auxPage->saveElement(file, { "auth", "host", "port" });
	
	file.close();
	SPIFFS.end();

	return "";
}

void captive_rootPage() {
  String content = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8" name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
<p style="text-align:center;font-weight:bold;"><a href="/_ac">Click through to configure WiFi</a></p>
</body>
</html>
    )";
  Server.send(200, "text/html", content);
}

//!MARK: Blynk

BLYNK_WRITE(V2) {
  targetTemperature = param.asFloat();
}

BLYNK_WRITE(V3) {
  powerState = param.asInt();
}

void updateBlynk() {
	float internalTemp = (temprature_sens_read() - 32) / 1.8;
	
	Blynk.virtualWrite(V0, currentTemperature);
	Blynk.virtualWrite(V1, internalTemp);

	if (relay_state) {
		blynk_led.on();
	}
	else {
		blynk_led.off();
	}
}
