/*
 * Biquad.h
 *
 *  Created on: 8 dec. 2018
 *      Author: bartv
 */

#ifndef BIQUAD_H_
#define BIQUAD_H_

#include "main.h"


struct FilterParameters {
	volatile float a0, a1, a2, b1, b2;
};





class BiquadFilter {

public:
	BiquadFilter (struct FilterParameters fp) {
		a0 = fp.a0;
		a1 = fp.a1;
		a2 = fp.a2;
		b1 = fp.b1;
		b2 = fp.b2;
	}

	float calculate(float inputValue){
		valueFilter[2] = valueFilter[1];
		valueFilter[1] = valueFilter[0];
		valueFilter[0] = inputValue - b1 * valueFilter[1] - b2 * valueFilter[2];
		outputValue = a0 * valueFilter[0] + a1 * valueFilter[1] + a2 * valueFilter[2];
//		outputValue *= 10;

		return outputValue;
	}

	float calculate(float inputValue, float offsetValue){
		outputValue = calculate(inputValue) + offsetValue;
		return outputValue;
	}

	float getValue(){
		return outputValue;
	}

	uint16_t average(uint16_t valuePeak) {
		value[index] = valuePeak;

		// Calculates the average of the last three values
		averageValue = (value[0] + value[1] + value[2]) / 3;

		// Makes the value of index an infinite loop from 0 to 2
		if(++index >= 3) index = 0;

		return averageValue;
	}

//	// Threshold cut the value
//	valuePeakXNoNoise[index] = REMOVE_THRESHOLD(valuePeakX, threshold);
//
//	// Calculates the average of the last three valuePeakXNoNoise values
//	valuePeakAverageXNoNoise = (valuePeakXNoNoise[0] + valuePeakXNoNoise[1] + valuePeakXNoNoise[2]) / 3;
//
//	// Get highest sample before new send (gets reset after sending)
//	highestAverageSendValue = MAX(highestAverageSendValue, valuePeakAverageXNoNoise);





private:
	float a0, a1, a2, b1, b2;
	float valueFilter[3] = {0}; // Pads the rest of the list with 0's
	float outputValue = 0;
	uint16_t value[3] = {0}, averageValue;
	uint8_t index = 0;

};



template <class dataType, class totalType> class MeasArray {
public:
	MeasArray(int s) {
		size = s;
		take = new dataType[s];
		fill(0);
	}
	~MeasArray() {
		delete[] take;
	}

	void add(dataType value) {
		total -= take[index];
		take[index] = value;
		total += take[index];

		if (++index == size) index = 0;
	}
	void fill(dataType value) {
		for (int i = 0; i < size; i++) take[i] = value;
		index = 0;
		total = value * size;
	}
	void reset() {
		fill(0);
	}

	dataType getAverage() {
		return total / size;
	}

	// Returns a single sample. 0 is the latest one, -10 is 10 samples ago.
	dataType getSample(int32_t i) {
		int32_t calcIndex = index + i + size - 1;
		if (calcIndex >= size) calcIndex -= size;
		return take[calcIndex];
	}

private:
	int size;
	int index = 0;
	dataType * take;
	totalType total;
};


class Prescaler {
public:
	Prescaler() {}
	bool check(unsigned long value) {
		if (++counter == value) {
			counter = 0;
			return 1;
		}
		return 0;
	}
private:
	unsigned long counter = 0;
};


extern BiquadFilter BF_highPassFilter;



#endif /* BIQUAD_H_ */
