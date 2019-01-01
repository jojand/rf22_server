/*
 * Config.h
 *
 *  Created on: Sep 28, 2018
 *      Author: josef
 */

#ifndef UTILS_CONFIG_H_
#define UTILS_CONFIG_H_

#include <Arduino.h>

#define VERSION 1

class Config {
public:
	Config(uint8_t *inBuf)				{ buffer = inBuf; }
	virtual ~Config() {}

	boolean checkVersion()				{ return buffer[0] == VERSION; }

	uint8_t getTxPeriod()				{ return buffer[1]; }
	void setTxPeriod(uint8_t value)		{ buffer[1] = value; }

	uint8_t getTxPower() 				{ return buffer[2]; }
	void setTxPower(uint8_t value)		{ buffer[2] = value; }

private:
	int bufferSize = 0;
	uint8_t *buffer;
};

#endif /* UTILS_CONFIG_H_ */
