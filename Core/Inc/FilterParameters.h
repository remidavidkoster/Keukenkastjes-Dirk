/*
 * FilterParameters.h
 *
 *  Created on: 17 jan. 2019
 *      Author: bartv
 */

#ifndef FILTERPARAMETERS_H_
#define FILTERPARAMETERS_H_

// https://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/

// 100hz high pass filter @ 50khz sample rate
FilterParameters filterHP100Q1 = {
		0.9937169785095502,
		-1.9874339570191004,
		0.9937169785095502,
		-1.9873554942063338,
		0.9875124198318671
};


FilterParameters filterThrough = {
		1,
		0,
		0,
		0,
		0
};



#endif /* FILTERPARAMETERS_H_ */
