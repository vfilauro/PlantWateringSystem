/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "5f5229c643474fbab7c813ccce676cd4";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Goldrake";
char pass[] = "vaffanculoatecazzone";

// Hardware Serial on Mega, Leonardo, Micro...
//#define EspSerial Serial1

// or Software Serial on Uno, Nano...
#include <SoftwareSerial.h>
SoftwareSerial EspSerial(2, 3); // RX, TX

// Your ESP8266 baud rate:
#define ESP8266_BAUD 19200

// Moisture sensors
#define MoistureSamples 3	// the number of samples used for each reading
#define MSensor1 A0			// the Arduino pin to which this sensor's analog output is connected
#define MSensor1Virtual 1	// the Blynk Virtual pin to which this sensor is connected
#define MSensor1Alarm 8		// the Arduino pin to which this sensor's digital alarm is connected

ESP8266 wifi(&EspSerial);

void setup()
{
	pinMode(MSensor1, INPUT);
	pinMode(MSensor1Alarm, INPUT);
	
	// Debug console
	Serial.begin(115200);


	// Set ESP8266 baud rate
	EspSerial.begin(ESP8266_BAUD);
	delay(10);

	Blynk.begin(auth, wifi, ssid, pass);
	// You can also specify server:
	//Blynk.begin(auth, wifi, ssid, pass, "blynk-cloud.com", 8442);
	//Blynk.begin(auth, wifi, ssid, pass, IPAddress(192,168,1,100), 8442);


}

void loop()
{
	Blynk.run();
	// You can inject your own code or combine it with other sketches.
	// Check other examples on how to communicate with Blynk. Remember
	// to avoid delay() function!
}
