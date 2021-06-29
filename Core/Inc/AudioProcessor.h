/*
 * AudioProcessor.h
 *
 *  Created on: 8 dec. 2018
 *      Author: bartv
 */

#ifndef AUDIOPROCESSOR_H_
#define AUDIOPROCESSOR_H_



#define SCALE(value, scaleFactor) value * scaleFactor;
#define REMOVE_THRESHOLD(value, threshold) value < threshold ? 0 : (uint32_t)(value - threshold) * 2047 / (2047 - threshold);
#define DECREASE(value, slope) value >= slope ? value - slope : 0;
#define MAX(valueA, valueB) valueA > valueB ? valueA : valueB;
#define MIN(valueA, valueB) valueA < valueB ? valueA : valueB;
#define LIMIT(min, x, max) ((x < min) ? min : (x > max) ? max : x)
#define RECTIFY(value, center) value >= center ? value - center : center - value;
#define THRESHOLD 150
#define ADC_CENTER 2047
#define SAVED_MAX_VALUES 10


class AudioProcessor {
public:
	AudioProcessor(){}

	uint16_t process(BiquadFilter& biquadFilter, uint16_t valueAudio, uint8_t decreaseSlope, uint16_t threshold, float scaleFactor, uint16_t centerADC) {

		volatile uint16_t rectifiedValueX, limitedValueX, valuePeakXNoNoise, valuePeakAverageXNoNoise;

		// Biquad filter the incoming audio
		filteredValueX = biquadFilter.calculate(valueAudio, centerADC);

		// Rectify the filtered audio
		rectifiedValueX = RECTIFY(filteredValueX, centerADC);

		// Limit to 11 bits
		limitedValueX = LIMIT(0, rectifiedValueX, centerADC);

		// Decrease old peak
		valuePeakX = DECREASE(valuePeakX, decreaseSlope);

		// Peak detect
		valuePeakX = MAX(valuePeakX, limitedValueX);

		// Scales valuePeakX with the highest measured ADC value
		notScaledValue = valuePeakX;
		scaledValue = valuePeakX * scaleFactor;
		valuePeakXX = scaledValue;

		// Threshold cut the value
		valuePeakXNoNoise = REMOVE_THRESHOLD(valuePeakXX, threshold);

		// Averages valuePeakXNoNoise
		valuePeakAverageXNoNoise = biquadFilter.average(valuePeakXNoNoise);

		// Get highest sample before new send (higestAverageSendValue gets reset after sending)
		highestAverageSendValue = MAX(highestAverageSendValue, valuePeakAverageXNoNoise);

		return highestAverageSendValue;
	}


	uint16_t process(BiquadFilter& biquadFilter, float valueAudio, uint8_t decreaseSlope, uint16_t threshold, float scaleFactor, uint16_t centerADC) {
		volatile uint16_t rectifiedValueX, limitedValueX, valuePeakXNoNoise, valuePeakAverageXNoNoise;

		// Biquad filter the incoming audio
		filteredValueX = biquadFilter.calculate(valueAudio, centerADC);

		// Rectify the filtered audio
		rectifiedValueX = RECTIFY(filteredValueX, centerADC);

		// Limit to 11 bits
		limitedValueX = LIMIT(0, rectifiedValueX, centerADC);

		// Decrease old peak
		valuePeakX = DECREASE(valuePeakX, decreaseSlope);

		// Peak detect
		valuePeakX = MAX(valuePeakX, limitedValueX);

		notScaledValue = valuePeakX;
		scaledValue = valuePeakX * scaleFactor;
		valuePeakXX = scaledValue;

		// Threshold cut the value
		valuePeakXNoNoise = REMOVE_THRESHOLD(valuePeakXX, threshold);

		// Averages valuePeakXNoNoise
		valuePeakAverageXNoNoise = biquadFilter.average(valuePeakXNoNoise);

		// Get highest sample before new send (gets reset after sending)
		highestAverageSendValue = MAX(highestAverageSendValue, valuePeakAverageXNoNoise);

		asm("nop");

		// Scales highestAverageSendValue with the highest measured ADC value
//		notScaledValue = highestAverageSendValue;
//		scaledValue = notScaledValue * scaleFactor;
//		highestAverageSendValue2 = scaledValue;

		asm("nop");
		return highestAverageSendValue;
	}

	void resetHighestSendValue() {
		highestAverageSendValue = 0;
	}

	int getHighestSendValue() {
		return highestAverageSendValue;
	}

	uint16_t filter(BiquadFilter& biquadFilter, float valueAudio, uint16_t centerADC) {
		return biquadFilter.calculate(valueAudio, centerADC);
	}

	uint16_t filter(BiquadFilter& biquadFilter, uint16_t valueAudio, uint16_t centerADC) {
		return biquadFilter.calculate(valueAudio, centerADC);
	}

private:
	volatile float filteredValueX, scaledValue, notScaledValue;
	volatile uint16_t highestAverageSendValue, valuePeakX, valuePeakXX;
};



#endif /* AUDIOPROCESSOR_H_ */
