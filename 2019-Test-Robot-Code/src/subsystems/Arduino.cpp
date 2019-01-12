/*
 * Arduino.c
 *
 *  Created on: Jan 11, 2019
 *      Author: super-tails
 */

#include "Arduino.h"
#include <stdlib.h>
#include <time.h>
#include <string>

bool Arduino::handshake() {
	// Generate a random digit to send, we should recieve that digit + 1
	srand(time(0));

	int value = rand() % 9; // Value between 0 and 8, to make sure it fits in one digit

	unsigned char send_buffer[] = "Sending _";
	send_buffer[sizeof(send_buffer) - 1] = value + '0';

	unsigned char recieve_buffer[2] = { 0xFF, 0xFF }; // Digit and terminating zero

	m_i2c.Transaction(send_buffer, sizeof(send_buffer), recieve_buffer, sizeof(recieve_buffer));

	return recieve_buffer[0] - '0' == value + 1 && recieve_buffer[1] == 0;
};

std::pair< SensorFrame, bool > Arduino::readData() {
	RxFrame rawFrame = readRawData();

	// Check for basic transmission errors
	if (rawFrame.verification != RxFrame::magic_number) {
		return { {}, false };
	}

	return { {}, true };
}

Arduino::RxFrame Arduino::readRawData() {
	TxFrame txFrame{ TxFrame::magic_number };
	RxFrame rxFrame{};
	m_i2c.Transaction(reinterpret_cast< std::uint8_t* >(&txFrame), sizeof(TxFrame), reinterpret_cast< std::uint8_t* >(&rxFrame), sizeof(RxFrame));

	return rxFrame;
}
