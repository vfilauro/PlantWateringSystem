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
#define NUM_SENSORS				4		// the number of sensors attached
#define NUM_SAMPLES				3		// the number of samples used for each reading
#define MSensor0				A0		// the Arduino pin to which this sensor's analog output is connected
#define MSensor0Virtual			1		// the Blynk Virtual pin to which this sensor is connected
#define MSensor0Alarm			4		// the Arduino pin to which this sensor's digital alarm is connected
#define MSensor0AlarmVirtual	16		// the BlynkVirtual pin to which this sensor's digital alarm is connected
#define MSensor1				A1		// the Arduino pin to which this sensor's analog output is connected
#define MSensor1Virtual			2		// the Blynk Virtual pin to which this sensor is connected
#define MSensor1Alarm			5		// the Arduino pin to which this sensor's digital alarm is connected
#define MSensor1AlarmVirtual	17		// the BlynkVirtual pin to which this sensor's digital alarm is connected
#define MSensor2				A2		// the Arduino pin to which this sensor's analog output is connected
#define MSensor2Virtual			3		// the Blynk Virtual pin to which this sensor is connected
#define MSensor2Alarm			6		// the Arduino pin to which this sensor's digital alarm is connected
#define MSensor2AlarmVirtual	18		// the BlynkVirtual pin to which this sensor's digital alarm is connected
#define MSensor3				A3		// the Arduino pin to which this sensor's analog output is connected
#define MSensor3Virtual			4		// the Blynk Virtual pin to which this sensor is connected
#define MSensor3Alarm			7		// the Arduino pin to which this sensor's digital alarm is connected
#define MSensor3AlarmVirtual	19		// the BlynkVirtual pin to which this sensor's digital alarm is connected

#define PumpRelay				8		// the Arduino output pin to which the pump relay is connected
#define PumpRelayVirtual		5		// the Blynk Virtual pin to which the pump relay is connected

#define WaterLevelEmpty			11		// the Arduino input pin to which the empty water sensor is connected
#define WaterLevelHalf			12		// the Arduino input pin to which the half water sensor is connected
#define WaterLevelFull			13		// the Arduino input pin to which the full water sensor is connected
byte WaterLevel;
#define WaterLevelVirtual		6		// the Arduino input pin to which the full water sensor is connected

#define Sensor0ThresholdVirtual 20		// virtual pin 0 asigned to change sleep period
#define Sensor1ThresholdVirtual 21		// virtual pin 0 asigned to change sleep period
#define Sensor2ThresholdVirtual 22		// virtual pin 0 asigned to change sleep period
#define Sensor3ThresholdVirtual 23		// virtual pin 0 asigned to change sleep period
#define DEFAULTSENSORTHRESHOLD	1024	// the default moisture threshold value
#define DEFAULTSENSORVERYDRY	1024	// the default moisture "very dry" threshold value

uint16_t Sensor0Threshold = DEFAULTSENSORTHRESHOLD;		// the default moisture threshold for this sensor
uint16_t Sensor1Threshold = DEFAULTSENSORTHRESHOLD;		// the default moisture threshold for this sensor
uint16_t Sensor2Threshold = DEFAULTSENSORTHRESHOLD;		// the default moisture threshold for this sensor
uint16_t Sensor3Threshold = DEFAULTSENSORTHRESHOLD;		// the default moisture threshold for this sensor

uint16_t SensorVeryDry = DEFAULTSENSORVERYDRY;		// the default moisture "very dry" threshold for all sensors

#define DEFAULT_WATERING_MAX_INTERVAL		300000L		// 5 minutes
long WATERING_MAX_INTERVAL = DEFAULT_WATERING_MAX_INTERVAL;
#define SetMaxWaterIntervalVirtual	26

#define MAIN_WATERLEVELMONITOR_INTERVAL		120000L		// 2 mins while not watering
#define WATERING_WATERLEVELMONITOR_INTERVAL	5000L		// 5 secs while watering
#define MAIN_MOISTUREMONITOR_INTERVAL		60000L		// 1 minute while not watering
#define WATERING_MOISTUREMONITOR_INTERVAL	15000L		// 15 secs while not watering

bool WaterPumpIsON;						//current status of the water pump
#define WaterPumpStatusVirtual	24		//virtual pin to read the current status of the water pump

uint16_t WaterPumpOnCounter;
uint16_t WaterPumpOffCounter;
#define ResetCountersVirtual	25		//virtual pin to reset the pump ON/OFF counters

#define RedLED					9		// the Arduino ouput pin to which the RED LED is connected
#define GreenLED				10		// the Arduino ouput pin to which the GREEN LED is connected

BlynkTimer timer;
int mainSoilMoistureTimer;						// ID for the main timer to monitor soil mositure (used when not watering). 
int wateringSoilMoistureTimer;					// ID for the main timer to monitor soil mositure (used when watering).
int mainWaterLevelTimer;						// ID for the main timer to monitor water level (used when not watering).
int wateringWaterLevelTimer;					// ID for the main timer to monitor water level (used when watering).

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

	
	//make sure pump is off
	StopWatering();
	WaterPumpIsON = false;		//redundant (already done in StopWatering);
	WaterPumpOffCounter = 0;
	WaterPumpOnCounter = 0;

	CheckWaterLevel();		// initialize the waterLevel variable

	pinMode(RedLED, OUTPUT);
	pinMode(GreenLED, OUTPUT);


	// Debug console
	Serial.begin(115200);
	
	// Set ESP8266 baud rate
	EspSerial.begin(ESP8266_BAUD);
	delay(10);

	Blynk.begin(auth, wifi, ssid, pass);
	// You can also specify server:
	//Blynk.begin(auth, wifi, ssid, pass, "blynk-cloud.com", 8442);
	//Blynk.begin(auth, wifi, ssid, pass, IPAddress(192,168,1,100), 8442);

	mainSoilMoistureTimer = timer.setInterval(MAIN_MOISTUREMONITOR_INTERVAL, readSoilMoisture);				// set timer to continuously monitor the moisture level (when not watering)
	wateringSoilMoistureTimer = timer.setInterval(WATERING_MOISTUREMONITOR_INTERVAL, readSoilMoisture);		// set timer to continuously monitor the moisture level (when watering)
	timer.disable(wateringSoilMoistureTimer);

	mainWaterLevelTimer = timer.setInterval(MAIN_WATERLEVELMONITOR_INTERVAL, CheckWaterLevel);				// set timer to continuously monitor the water level (when not watering)
	wateringWaterLevelTimer = timer.setInterval(WATERING_WATERLEVELMONITOR_INTERVAL, CheckWaterLevel);		// set timer to continuously monitor the water level (when watering)
	timer.disable(wateringWaterLevelTimer);
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

	alarm[0] = digitalRead(MSensor0Alarm);
	alarm[1] = digitalRead(MSensor1Alarm);
	alarm[2] = digitalRead(MSensor2Alarm);
	alarm[3] = digitalRead(MSensor3Alarm);

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

	if (moisture_average[0] > Sensor0Threshold)
		PlantsNeedingWater++;
	if (moisture_average[0] > SensorVeryDry)
		PlantsVeryDry++;
	if (moisture_average[1] > Sensor1Threshold)
		PlantsNeedingWater++;
	if (moisture_average[1] > SensorVeryDry)
		PlantsVeryDry++;
	if (moisture_average[2] > Sensor2Threshold)
		PlantsNeedingWater++;
	if (moisture_average[2] > SensorVeryDry)
		PlantsVeryDry++;
	if (moisture_average[3] > Sensor3Threshold)
		PlantsNeedingWater++;
	if (moisture_average[3] > SensorVeryDry)
		PlantsVeryDry++;


	if (PlantsNeedingWater >= 3 || PlantsVeryDry > 0) {
		// Start pump
		if (WaterPumpIsON == false)
			StartWatering();
	}
	else if (PlantsNeedingWater == 0 ) {
		// Stop pump if moisture level is good for all plants
		if (WaterPumpIsON == true)
			StopWatering();
	}

}

void StartWatering() {
	if (digitalRead(WaterLevelEmpty>0)) {
		digitalWrite(PumpRelay, HIGH);
		WaterPumpIsON = true;
		WaterPumpOnCounter++;
		Blynk.virtualWrite(PumpRelayVirtual, 255);
	#if DEBUG >= 1
		BLYNK_ASSERT(timer.isEnabled(mainSoilMoistureTimer) == false);
		BLYNK_ASSERT(timer.isEnabled(wateringSoilMoistureTimer) == true);
		BLYNK_ASSERT(timer.isEnabled(mainWaterLevelTimer) == false);
		BLYNK_ASSERT(timer.isEnabled(wateringWaterLevelTimer) == true);
#endif
		timer.disable(mainSoilMoistureTimer);
		timer.enable(wateringSoilMoistureTimer);
		timer.disable(mainWaterLevelTimer);
		timer.enable(wateringWaterLevelTimer);
		timer.setTimeout(WATERING_MAX_INTERVAL, StopWatering);				// set timer to stop the pump when the MAX_INTERVAL value is passed
	}

}

void CheckWaterLevel() {
	if (digitalRead(WaterLevelEmpty)) {
		WaterLevel = 0;
		if (WaterPumpIsON == true)
			StopWatering();
	}
	else if (digitalRead(WaterLevelHalf))
		WaterLevel = 1;
	else if (digitalRead(WaterLevelFull))
		WaterLevel = 2;
	else
		WaterLevel = 3;
	Blynk.virtualWrite(WaterLevelVirtual, WaterLevel);
}

void StopWatering() {
	digitalWrite(PumpRelay, LOW);
	WaterPumpIsON = false;
	WaterPumpOffCounter++;
	Blynk.virtualWrite(PumpRelayVirtual, 0);

#if DEBUG >= 1
	BLYNK_ASSERT(timer.isEnabled(mainSoilMoistureTimer) == true);
	BLYNK_ASSERT(timer.isEnabled(wateringSoilMoistureTimer) == false);
	BLYNK_ASSERT(timer.isEnabled(mainWaterLevelTimer) == true);
	BLYNK_ASSERT(timer.isEnabled(wateringWaterLevelTimer) == false);

#endif
	timer.disable(wateringSoilMoistureTimer);
	timer.enable(mainSoilMoistureTimer);
	timer.disable(wateringWaterLevelTimer);
	timer.enable(mainWaterLevelTimer);
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


BLYNK_READ(WaterLevelVirtual) {
	Blynk.virtualWrite(WaterLevelVirtual, WaterLevel);
#if DEBUG >=1
	Serial.print("Blynk reading WaterLevel = ");
	Serial.println(WaterLevel);
#endif
}

BLYNK_WRITE(ResetCountersVirtual) {
	WaterPumpOffCounter = WaterPumpOnCounter = 0;
#if DEBUG >=1
	Serial.print("Blynk resetting counters - param = ");
	Serial.println(param.asInt());
#endif
}

BLYNK_READ(WaterPumpStatusVirtual) {
	Blynk.virtualWrite(WaterPumpStatusVirtual, WaterPumpIsON?1:0);
#if DEBUG >=1
	Serial.print("Blynk reading WaterPumpStatus = ");
	Serial.println(Sensor3Threshold);
#endif
}

BLYNK_WRITE(SetMaxWaterIntervalVirtual) {
	WATERING_MAX_INTERVAL = param.asLong()*1000L;
#if DEBUG >=1
	Serial.print("Blynk setting Watering Max Interval = ");
	Serial.println(WATERING_MAX_INTERVAL);
#endif
}

BLYNK_WRITE(PumpRelayVirtual) {
	if (WaterPumpIsON)
		StopWatering();
	else
		StartWatering();

	Serial.print("Blynk toggling water Pump relay = ");
	Serial.println(param.asInt());
}


BLYNK_CONNECTED() {
	Blynk.syncAll();
}