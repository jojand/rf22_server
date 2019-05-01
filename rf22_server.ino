/*
 * Base radio station based on rf22 modules
 * [MQTT] rf/<address>/config 1/1/0*
 */

//#define RF_22
#define RF_95

//#define RAM_DEBUG
//#define SERIAL_DEBUG

#ifdef RF_22
#include <RH_RF22.h>
#elif defined RF_95
#include <RH_RF95.h>
#endif

#include <RHReliableDatagram.h>
#include <SPI.h>
#include "utils/Streaming.h"
#include "utils/Config.h"
#include <avr/wdt.h>

#define SERVER_ADDRESS  	200
#define SERIAL_BAUD     	57600
#define RF_BUFFER_SIZE		50
#define SERIAL_BUFFER_SIZE	50
#define RCV_FAIL_THRESHOLD	50

#define ERROR_HEADER    F("[ERROR] ")
#define WARN_HEADER     F("[WARN] ")
#define INFO_HEADER     F("[INFO] ")
#define MQTT_HEADER     F("[MQTT] ")

#define MQTT_RF_CONFIG_PREFIX	"rf/config/200"
#define MQTT_TOPIC_PREFIX 		"rf/"
#define MQTT_DS_TEMPERATURE 	"/dst"
#define MQTT_DH_TEMPERATURE 	"/dht"
#define MQTT_DH_HUMIDITY    	"/dhh"
#define MQTT_SHT_TEMPERATURE 	"/sht"
#define MQTT_SHT_HUMIDITY    	"/shh"
#define MQTT_HTU_TEMPERATURE    "/htt"
#define MQTT_HTU_HUMIDITY    	"/hth"
#define MQTT_BAT_VOLTAGE    	"/bat"
#define MQTT_STAT_OK_TOPIC		"/ok"
#define MQTT_STAT_ERR_TOPIC		"/err"
#define MQTT_RSSI_TOPIC			"/rssi"

#ifdef RF_22
#define TX_POWER					RH_RF22_TXPOW_11DBM
#elif defined RF_95
#define TX_POWER					17
#endif

#ifdef RF_22
#define DEFAULT_SENSOR_TX_POWER		RH_RF22_TXPOW_11DBM
#elif defined RF_95
#define DEFAULT_SENSOR_TX_POWER		17
#endif
#define DEFAULT_SENSOR_TX_PERIOD	15
#define SLOW_LOOP					10000

#ifdef RF_22
RH_RF22 radioDriver;
#elif defined RF_95
RH_RF95 radioDriver;
#endif

RHReliableDatagram radio(radioDriver, SERVER_ADDRESS);
Config *config;
int rcvFailed = 0;

// in data buffer for sensor data
uint8_t inBuffer[RF_BUFFER_SIZE];
// out buffer used to setup of the remote sensor
uint8_t outBuffer[RF_BUFFER_SIZE];

#define RADIO_SHDN	4

void radioOn() {
#ifdef RF_22
	digitalWrite(RADIO_SHDN, LOW);
#elif defined RF_95
	digitalWrite(RADIO_SHDN, HIGH);
#endif
}

void radioOff() {
#ifdef RF_22
	digitalWrite(RADIO_SHDN, HIGH);
#elif defined RF_95
	digitalWrite(RADIO_SHDN, LOW);
#endif
}

void setup() {
	config = new Config(outBuffer);
	config->setTxPeriod(DEFAULT_SENSOR_TX_PERIOD);
	config->setTxPower(DEFAULT_SENSOR_TX_POWER);

	Serial.begin(SERIAL_BAUD);
	Serial << INFO_HEADER << F("Base rf22 station start") << endl;
#ifdef RAM_DEBUG
	Serial << F("init ") << freeRam() << endl;
#endif

    // watchdog
    wdt_reset();
#ifdef SERIAL_DEBUG
	Serial << INFO_HEADER << "enabling WDTO_8S" << endl;
#endif
	wdt_enable(WDTO_8S);

	// init radio
	pinMode(RADIO_SHDN, OUTPUT);
	radioOff();
	delay(1000);
	radioOn();
	delay(500);

	if (!radio.init()) {
		Serial << ERROR_HEADER << F("radio init failed! Stopping.") << endl;
	} else {
#ifdef SERIAL_DEBUG
		Serial << INFO_HEADER << F("radio init ok ...") << endl;
#endif
	}

	radioDriver.setTxPower(TX_POWER);
	wdt_reset();
}

void sendChunkToSerial(char *&chunk, char *topic, uint8_t from) {
	if (chunk != NULL) {
		Serial << MQTT_HEADER << MQTT_TOPIC_PREFIX << String(from) << topic << " " << chunk << "*" << '\n';
	}
	chunk = strtok(NULL, "/");
	delay(300);
}

void sendToSerial(char *message, uint8_t from) {
	char *chunk;

	chunk = strtok(message, "/");

	sendChunkToSerial(chunk, (char*) MQTT_BAT_VOLTAGE, from);
	sendChunkToSerial(chunk, (char*) MQTT_STAT_OK_TOPIC, from);
	sendChunkToSerial(chunk, (char*) MQTT_STAT_ERR_TOPIC, from);
	sendChunkToSerial(chunk, (char*) MQTT_DS_TEMPERATURE, from);
	sendChunkToSerial(chunk, (char*) MQTT_DH_TEMPERATURE, from);
	sendChunkToSerial(chunk, (char*) MQTT_DH_HUMIDITY, from);
	sendChunkToSerial(chunk, (char*) MQTT_SHT_TEMPERATURE, from);
	sendChunkToSerial(chunk, (char*) MQTT_SHT_HUMIDITY, from);
	sendChunkToSerial(chunk, (char*) MQTT_HTU_TEMPERATURE, from);
	sendChunkToSerial(chunk, (char*) MQTT_HTU_HUMIDITY, from);
	Serial << MQTT_HEADER << MQTT_TOPIC_PREFIX << String(from) << MQTT_RSSI_TOPIC << " " << radioDriver.lastRssi() << "*" << '\n';
}

void radioLoop() {
	if (radio.available()) {
#ifdef RAM_DEBUG
		Serial << F("radio start ") << freeRam() << endl;
#endif
		uint8_t len = sizeof(inBuffer);
		uint8_t from;


		if (radio.recvfromAckTimeout(inBuffer, &len, 100, &from)) {
			wdt_reset();
#ifdef SERIAL_DEBUG
			Serial << INFO_HEADER << F("sending response ...") << endl;
#endif
			delay(10);
			if (radio.sendtoWait(outBuffer, sizeof(outBuffer), from)) {
				wdt_reset();
#ifdef SERIAL_DEBUG
				Serial << INFO_HEADER << F("response sent OK ...") << endl;
#endif
			} else {
#ifdef SERIAL_DEBUG
				Serial << INFO_HEADER << F("response sent FAILED ...") << endl;
#endif
			}

			sendToSerial((char*) inBuffer, from);
		} else {
			rcvFailed++;
#ifdef SERIAL_DEBUG
			Serial << WARN_HEADER << F("Receiving failed ") << rcvFailed << F("/") << RCV_FAIL_THRESHOLD << F("...") << endl;
#endif
			if (rcvFailed > RCV_FAIL_THRESHOLD) {
#ifdef SERIAL_DEBUG
				Serial << ERROR_HEADER << F("Receive failed threshold exceeded! Reset will be performed.") << endl;
#endif
				Serial << MQTT_HEADER << MQTT_TOPIC_PREFIX << SERVER_ADDRESS << F("/debug RCV_THRESHOLD_RESET") << "*" << '\n';
				while(true); // will cause watchdog reset
			}
		}

#ifdef RAM_DEBUG
		Serial << F("radio end ") << freeRam() << endl;
#endif
	}
}

void serialLoop() {
	if (Serial.available()) {
#ifdef RAM_DEBUG
		Serial << F("Serial start ") << freeRam() << endl;
#endif

		Serial.setTimeout(500);

		String command = Serial.readStringUntil('\n');

		if (command.charAt(command.length()-1) != '*') {
#ifdef SERIAL_DEBUG
			Serial << ERROR_HEADER << F("Tailing * not detected: ") << command << endl;
#endif
			return;
		}

		if (!command.startsWith(MQTT_HEADER)) {
#ifdef SERIAL_DEBUG
			Serial << ERROR_HEADER << F("Wrong header detected: ") << command << endl;
#endif
			return;
		}

		String message = command.substring(String(MQTT_HEADER).length(), command.length() - 1);
#ifdef SERIAL_DEBUG
		Serial << F("Message detected: '") << message << "'" << endl;
#endif

		if (!message.startsWith(MQTT_RF_CONFIG_PREFIX)) {
#ifdef SERIAL_DEBUG
			Serial << ERROR_HEADER << F("Unknown MQTT topic: ") << command << endl;
#endif
			return;
		}

		message = message.substring(String(MQTT_RF_CONFIG_PREFIX).length());

		int valuesCount = 0;
		int index = 0;
		int lastIndex = 0;
		do {
			index = message.indexOf('/', lastIndex);

			int val = message.substring(lastIndex, index != -1 ? index : message.length()).toInt();
#ifdef SERIAL_DEBUG
			Serial << INFO_HEADER << F("decoded value - i: ") << valuesCount << F(", val: ") << val << endl;
#endif
			outBuffer[valuesCount++] = val;

			lastIndex = index + 1;
		} while (index >= 0 && ((unsigned int)index+1 < message.length()) && (valuesCount<RF_BUFFER_SIZE));
#ifdef RAM_DEBUG
		Serial << F("Serial end ") << freeRam() << endl;
#endif
	}
}

void slowLoop() {
    static unsigned long timeLoop = 0;

    if ((millis() - timeLoop) >= SLOW_LOOP) {
        timeLoop = millis();
        Serial << MQTT_HEADER << MQTT_TOPIC_PREFIX << SERVER_ADDRESS << F("/heartbeat ") << timeLoop << "*" << '\n';
    }
}

void loop() {
	wdt_reset();
	radioLoop();
	serialLoop();
	slowLoop();
}

#ifdef RAM_DEBUG
int freeRam()
{
     extern int __heap_start, *__brkval;
     int v;
     return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
#endif
