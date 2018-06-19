/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "6737339ce6434ad6b64b6858c6366739";

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
#define MoistureSamples			3		// the number of samples used for each reading
#define MSensor0				A0		// the Arduino pin to which this sensor's analog output is connected
#define MSensor0Virtual			1		// the Blynk Virtual pin to which this sensor is connected
#define MSensor0Alarm			8		// the Arduino pin to which this sensor's digital alarm is connected
#define MSensor0AlarmVirtual	16		// the BlynkVirtual pin to which this sensor's digital alarm is connected
#define MSensor1				A1		// the Arduino pin to which this sensor's analog output is connected
#define MSensor1Virtual			2		// the Blynk Virtual pin to which this sensor is connected
#define MSensor1Alarm			9		// the Arduino pin to which this sensor's digital alarm is connected
#define MSensor1AlarmVirtual	17		// the BlynkVirtual pin to which this sensor's digital alarm is connected
#define MSensor2				A2		// the Arduino pin to which this sensor's analog output is connected
#define MSensor2Virtual			3		// the Blynk Virtual pin to which this sensor is connected
#define MSensor2Alarm			10		// the Arduino pin to which this sensor's digital alarm is connected
#define MSensor2AlarmVirtual	18		// the BlynkVirtual pin to which this sensor's digital alarm is connected
#define MSensor3				A3		// the Arduino pin to which this sensor's analog output is connected
#define MSensor3Virtual			4		// the Blynk Virtual pin to which this sensor is connected
#define MSensor3Alarm			11		// the Arduino pin to which this sensor's digital alarm is connected
#define MSensor3AlarmVirtual	19		// the BlynkVirtual pin to which this sensor's digital alarm is connected

#define PumpRelay				12		// the Arduino output pin to which the pump relay is connected
#define PumpRelayVirtual		5		// the Blynk Virtual pin to which the pump relay is connected

#define WaterLevelEmpty			13		// the Arduino input pin to which the empty water sensor is connected
#define WaterLevelHalf			14		// the Arduino input pin to which the half water sensor is connected
#define WaterLevelFull			15		// the Arduino input pin to which the full water sensor is connected
#define WaterLevelVirtual		6		// the Arduino input pin to which the full water sensor is connected

#define Sensor0ThresholdVirtual 20		// virtual pin 0 asigned to change sleep period
#define Sensor1ThresholdVirtual 21		// virtual pin 0 asigned to change sleep period
#define Sensor2ThresholdVirtual 22		// virtual pin 0 asigned to change sleep period
#define Sensor3ThresholdVirtual 23		// virtual pin 0 asigned to change sleep period
#define DEFAULTSENSORTHRESHOLD	124		// the default moisture threshold value
#define DEFAULTSENSORVERYDRY	1		// the default moisture "very dry" threshold value

byte Sensor0Threshold = DEFAULTSENSORTHRESHOLD;		// the default moisture threshold for this sensor
byte Sensor1Threshold = DEFAULTSENSORTHRESHOLD;		// the default moisture threshold for this sensor
byte Sensor2Threshold = DEFAULTSENSORTHRESHOLD;		// the default moisture threshold for this sensor
byte Sensor3Threshold = DEFAULTSENSORTHRESHOLD;		// the default moisture threshold for this sensor

byte Sensor0VeryDry = DEFAULTSENSORVERYDRY;		// the default moisture "very dry" threshold for this sensor
byte Sensor1VeryDry = DEFAULTSENSORVERYDRY;		// the default moisture "very dry" for this sensor
byte Sensor2VeryDry = DEFAULTSENSORVERYDRY;		// the default moisture "very dry" for this sensor
byte Sensor3VeryDry = DEFAULTSENSORVERYDRY;		// the default moisture "very dry" for this sensor

#define WATERING_INTERVAL			300000L		// 5 minutes
#define WATERLEVELMONITOR_INTERVAL	15000L		// 15 secs
#define MOISTUREMONITOR_INTERVAL	60000L		// 1 minute

bool WaterPumpIsON;
uint16_t WaterPumpOnCounter;
uint16_t WaterPumpOffCounter;


BlynkTimer timer;

ESP8266 wifi(&EspSerial);

void setup()
{
	pinMode(MSensor0, INPUT);
	pinMode(MSensor0Alarm, INPUT);
	pinMode(MSensor1, INPUT);
	pinMode(MSensor1Alarm, INPUT);
	pinMode(MSensor2, INPUT);
	pinMode(MSensor2Alarm, INPUT);
	pinMode(MSensor3, INPUT);
	pinMode(MSensor3Alarm, INPUT);

	pinMode(PumpRelay, OUTPUT);

	pinMode(WaterLevelEmpty, INPUT);
	pinMode(WaterLevelHalf, INPUT);
	pinMode(WaterLevelFull, INPUT);

	
	// Debug console
	Serial.begin(115200);


	// Set ESP8266 baud rate
	EspSerial.begin(ESP8266_BAUD);
	delay(10);

	Blynk.begin(auth, wifi, ssid, pass);
	// You can also specify server:
	//Blynk.begin(auth, wifi, ssid, pass, "blynk-cloud.com", 8442);
	//Blynk.begin(auth, wifi, ssid, pass, IPAddress(192,168,1,100), 8442);

	timer.setInterval(1000L, readSoilMoisture);

}

void loop()
{
	Blynk.run();
	timer.run();

}




void readSoilMoisture() {
	uint16_t moisture[NUM_SENSORS][NUM_SAMPLES];
	byte alarm[NUM_SENSORS];

	//digitalWrite(SensorsPwr, HIGH);
	//delay(1000);

	alarm[0] = digitalRead(MSensor0_Alarm);
	alarm[1] = digitalRead(MSensor1_Alarm);
	alarm[2] = digitalRead(MSensor2_Alarm);
	alarm[3] = digitalRead(MSensor3_Alarm);

	for (int i = 0; i < NUM_SAMPLES; i++) {
		delay(10);
		moisture[0][i] = analogRead(MSensor0);
		moisture[1][i] = analogRead(MSensor1);
		moisture[2][i] = analogRead(MSensor2);
		moisture[3][i] = analogRead(MSensor3);
	}


	//digitalWrite(SensorsPwr, LOW);
	

	//average all samples
	float moisture_average[NUM_SAMPLES];

	for (int i = 0; i< NUM_SENSORS; i++) {
		moisture_average[i] = 0;
		for (int y = 0; y < NUM_SAMPLES; y++) {
			moisture_average[i] += moisture[i][y];
		}
		moisture_average[i] /= NUM_SAMPLES;
	}

	Blynk.virtualWrite(MSensor0AlarmVirtual, alarm[0] ? 255 : 0);
	Blynk.virtualWrite(MSensor1AlarmVirtual, alarm[1] ? 255 : 0);
	Blynk.virtualWrite(MSensor2AlarmVirtual, alarm[2] ? 255 : 0);
	Blynk.virtualWrite(MSensor3AlarmVirtual, alarm[3] ? 255 : 0);

	Blynk.virtualWrite(MSensor0Virtual, moisture_average[0]);
	Blynk.virtualWrite(MSensor1Virtual, moisture_average[1]);
	Blynk.virtualWrite(MSensor2Virtual, moisture_average[2]);
	Blynk.virtualWrite(MSensor3Virtual, moisture_average[3]);

	//determine if we need to turn the water on
	byte PlantsNeedingWater = 0;
	byte PlantsVeryDry = 0;

	if (moisture_average[0] < Sensor0Threshold)
		PlantsNeedingWater++;
	if (moisture_average[0] < Sensor0VeryDry)
		PlantsVeryDry++;
	if (moisture_average[1] < Sensor1Threshold)
		PlantsNeedingWater++;
	if (moisture_average[1] < Sensor1VeryDry)
		PlantsVeryDry++;
	if (moisture_average[2] < Sensor2Threshold)
		PlantsNeedingWater++;
	if (moisture_average[2] < Sensor2VeryDry)
		PlantsVeryDry++;
	if (moisture_average[3] < Sensor3Threshold)
		PlantsNeedingWater++;
	if (moisture_average[3] < Sensor3VeryDry)
		PlantsVeryDry++;


	if (PlantsNeedingWater >= 3 || PlantsVeryDry > 0) {
		// Start pump
		if (WaterPumpIsON == false)
			StartWatering();
	}
	else if (PlantsNeedingWater == 0 ) {
		// Stop pump
		if (WaterPumpIsON == true)
			StopWatering();
	}

}

void StartWatering() {
	if (digitalRead(WaterLevelEmpty)) {
		//digitalWrite(PumpRelay, HIGH);
		WaterPumpIsON = true;
		WaterPumpOnCounter++;
		timer.setInterval(WATERING_INTERVAL, StopWatering);					// set timer to stop the pump
		timer.setInterval(WATERLEVELMONITOR_INTERVAL, CheckWaterLevel);		// set timer to continuously monitor the water level
		timer.setInterval(MOISTUREMONITOR_INTERVAL, readSoilMoisture);		// set timer to continuously monitor the moisture level
	}
}

void CheckWaterLevel() {
	byte WaterLevel;
	if (digitalRead(WaterLevelEmpty) == 0) {
		WaterLevel = 0;
		if (WaterPumpIsON == true)
			StopWatering();
	}
	else if (digitalRead(WaterLevelHalf) == 0)
		WaterLevel = 1;
	else if (digitalRead(WaterLevelFull) == 0)
		WaterLevel = 2;
	else
		WaterLevel = 3;
}

void StopWatering() {
	//digitalWrite(PumpRelay, LOW);
	WaterPumpIsON = false;
	WaterPumpOffCounter++;

}

BLYNK_WRITE(Sensor0ThresholdVirtual) {
	Sensor0Threshold = param.asInt();
#if DEBUG >=1
	Serial.print("Blynk writing Sensor0Threshold = ");
	Serial.println(Sensor0Threshold);
#endif
}
BLYNK_READ(Sensor0ThresholdVirtual) {
	Blynk.virtualWrite(Sensor0ThresholdVirtual, Sensor0Threshold);
#if DEBUG >=1
	Serial.print("Blynk reading Sensor0Threshold = ");
	Serial.println(Sensor0Threshold);
#endif
}

BLYNK_WRITE(Sensor1ThresholdVirtual) {
	Sensor1Threshold = param.asInt();
#if DEBUG >=1
	Serial.print("Blynk writing Sensor1Threshold = ");
	Serial.println(Sensor1Threshold);
#endif
}
BLYNK_READ(Sensor1ThresholdVirtual) {
	Blynk.virtualWrite(Sensor1ThresholdVirtual, Sensor1Threshold);
#if DEBUG >=1
	Serial.print("Blynk reading Sensor1Threshold = ");
	Serial.println(Sensor1Threshold);
#endif
}

BLYNK_WRITE(Sensor2ThresholdVirtual) {
	Sensor2Threshold = param.asInt();
#if DEBUG >=1
	Serial.print("Blynk writing Sensor2Threshold = ");
	Serial.println(Sensor2Threshold);
#endif
}
BLYNK_READ(Sensor2ThresholdVirtual) {
	Blynk.virtualWrite(Sensor2ThresholdVirtual, Sensor2Threshold);
#if DEBUG >=1
	Serial.print("Blynk reading Sensor2Threshold = ");
	Serial.println(Sensor2Threshold);
#endif
}

BLYNK_WRITE(Sensor3ThresholdVirtual) {
	Sensor3Threshold = param.asInt();
#if DEBUG >=1
	Serial.print("Blynk writing Sensor3Threshold = ");
	Serial.println(Sensor3Threshold);
#endif
}
BLYNK_READ(Sensor3ThresholdVirtual) {
	Blynk.virtualWrite(Sensor3ThresholdVirtual, Sensor3Threshold);
#if DEBUG >=1
	Serial.print("Blynk reading Sensor3Threshold = ");
	Serial.println(Sensor3Threshold);
#endif
}

BLYNK_CONNECTED() {
	Blynk.syncAll;
}