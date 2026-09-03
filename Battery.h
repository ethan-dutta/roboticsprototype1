#ifndef BATTERY_H_
#define BATTERY_H_

#include <Arduino.h>

typedef uint8_t(*mapFn_t)(uint16_t, uint16_t, uint16_t);

class Battery {
	public:

		Battery(uint16_t minVoltage, uint16_t maxVoltage, uint8_t sensePin, uint8_t adcBits = 10);


		void begin(uint16_t refVoltage, float dividerRatio, mapFn_t = 0);


		void onDemand(uint8_t activationPin, uint8_t activationMode = LOW);


		static const uint8_t ON_DEMAND_DISABLE = 0xFF;

		uint8_t level();
		uint8_t level(uint16_t voltage);


		uint16_t voltage(uint8_t delay = 2);

	private:
		uint16_t refVoltage;
		uint16_t minVoltage;
		uint16_t maxVoltage;
		float dividerRatio;
		uint8_t sensePin;
		uint8_t activationPin;
		uint8_t activationMode;
		mapFn_t mapFunction;

		const uint16_t adc;
};



static inline uint8_t sigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {

	uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
	return result >= 100 ? 100 : result;
}


static inline uint8_t asigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
	uint8_t result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage) ,4.5), 3));
	return result >= 100 ? 100 : result;
}


static inline uint8_t linear(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
	return (unsigned long)(voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
}
#endif
