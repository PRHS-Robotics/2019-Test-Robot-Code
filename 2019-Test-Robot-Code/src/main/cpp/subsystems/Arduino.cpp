/*
 * Arduino.c
 *
 *  Created on: Jan 11, 2019
 *      Author: super-tails
 */

#include "subsystems/Arduino.h"
#include <stdlib.h>
#include <time.h>
#include <string>
#include <iostream>

// "Sending _"
// "_"

// "Sending 5"
// "6"

bool Arduino::handshake() {
	// Generate a random digit to send, we should recieve that digit + 1
	srand(time(0));

	int value = rand() % 9; // Value between 0 and 8, to make sure it fits in one digit

	unsigned char send_buffer[] = "Sending _";
	send_buffer[sizeof(send_buffer) - 2] = value + '0';

	unsigned char recieve_buffer[100] = { 0xFF, 0xFF }; // Digit and terminating zero

	std::cout << "Sending the string:\n" << send_buffer << '\n';

	m_i2c.WriteBulk(send_buffer, sizeof(send_buffer));
	m_i2c.ReadOnly(2, recieve_buffer);

	std::cout << "Recieved: 0x" << std::hex << int(recieve_buffer[0]) << "\n";
	std::cout << "Recieved: 0x" << std::hex << int(recieve_buffer[1]) << "\n";

	return (recieve_buffer[0] - '0' == value + 1);
};

// TODO: Check endianness here too
std::pair< SensorFrame, bool > Arduino::readData() {
	RxFrame rawFrame = readRawData();

	// Check for basic transmission errors
	if (rawFrame.verification != RxFrame::magic_number) {
		return { {}, false };
	}

	return { {}, true };
}

// TODO: Check endianness
// RoboRio is likely big endian and Arduinos are little endian
Arduino::RxFrame Arduino::readRawData() {
	TxFrame txFrame{ TxFrame::magic_number };
	RxFrame rxFrame{};

	m_i2c.WriteBulk(reinterpret_cast< uint8_t* >(&txFrame), sizeof(txFrame));
	m_i2c.ReadOnly(sizeof(RxFrame), reinterpret_cast< uint8_t* >(&rxFrame));

	return rxFrame;
}
