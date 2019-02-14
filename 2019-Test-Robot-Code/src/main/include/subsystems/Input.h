/*
 * Input.h
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_INPUT_H_
#define SRC_SUBSYSTEMS_INPUT_H_

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/buttons/Button.h>
#include <unordered_map>
#include <string>
#include <bitset>
#include <tuple>
#include <array>

static const std::unordered_map< std::string, std::pair< std::string, int > > defaultButtonMap = {
		{ "SHIFT_FAST", { "High Speed", 3 }, },
		{ "SHIFT_SLOW", { "Low Speed", 5 }, },
		{ "TRIGGER", 	{ "Trigger", 1 } },
		{ "DEBUG_BUTTON", { "DO NOT TOUCH", 2 } },
		{ "SEARCH_AND_DESTROY", { "Search and Destroy", 7 } },
		{ "DEBUG_BUTTON_2", { "DO NOT TOUCH 2", 8 } },
		{ "MANUAL_OVERRIDE", { "Manual Override", 9 } },
		{ "FIND_TAPE", { "FIND_TAPE", 10 } }
};

constexpr const std::size_t MAX_PRIMARY_BUTTONS = 11;
constexpr const std::size_t MAX_SECONDARY_BUTTONS = 13;
constexpr const std::size_t MAX_BUTTONS = MAX_PRIMARY_BUTTONS + MAX_SECONDARY_BUTTONS;

// Stores the current state of the joystick & xbox controller axes, buttons, etc.
struct InputState {
	double x, y, r, t;
	std::bitset< MAX_BUTTONS > buttons;
};

// Returns the index into InputState::buttons of a given button name
std::size_t buttonIndex(const std::string& buttonId);

// Returns true if button mapped to code input 'buttonId' (e.g. 'SHIFT_FAST') is pressed
// Returns false if button map is invalid
bool buttonValue(InputState input, const std::string& buttonId);

// Applies deadzone correction to a raw input value
double applyDeadzone(double value, double deadzoneRange);

class Input {
public:

	Input(int primaryPort, int secondaryPort);

	// Returns unmodified, actual input state
	InputState getRawInput();

	// Returns input state with deadzone correction applied
	InputState getInput();

	// Returns the WPILib button handle for a given button name
	frc::Button *getButton(const std::string& name);

private:

	std::array< std::unique_ptr< frc::Button >, MAX_BUTTONS > buttons;

	frc::Joystick primary;
	frc::XboxController secondary;

	constexpr const static double deadzone = 0.2;
};



#endif /* SRC_SUBSYSTEMS_INPUT_H_ */
