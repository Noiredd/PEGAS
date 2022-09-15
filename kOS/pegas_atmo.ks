//	Atmospheric ascent library

//	Passive guidance within the atmosphere
FUNCTION atmosphericSteeringControl {
	//	Call the appropriate steering controller, depending on vehicle control settings.
	//	Expects a global variable "controls" as lexicon.

	DECLARE PARAMETER steeringRoll.	//	Expects a scalar

	IF controls:HASKEY("pitchProgram") {
		pitchProgramControl(steeringRoll).
	} ELSE {
		zeroAoAPitchControl(steeringRoll).
	}
}

//	Simple pitch over and hold zero angle-of-attack trajectory
FUNCTION zeroAoAPitchControl {
	//	Handle the entire atmospheric ascent phase, from vertical ascent, through pitching over,
	//	to holding prograde at a given azimuth. Called regularly by the main loop, updates the
	//	steeringVector directly, without returning anything.
	//	Expects global variables:
	//	"ascentFlag" as integer
	//	"controls" as lexicon
	//	"liftoffTime" as timespan
	//	"mission" as lexicon
	//	"SETTINGS" as lexicon
	//	"steeringVector" as vector

	DECLARE PARAMETER steeringRoll.	//	Expects a scalar

	//	Define the global at first run
	IF NOT (DEFINED ascentFlag) {
		//	Basic pitch&hold ascent consists of 4 phases, ascentFlag stores the phase index:
		//	0: vertical ascent
		//	1: pitch over by a given angle
		//	2: hold prograde with a given launch azimuth, emit a UI message
		//	3: hold prograde with a given azimuth
		GLOBAL ascentFlag IS 0.
	}

	IF ascentFlag = 0 {
		//	The vehicle is going straight up for given amount of time
		IF TIME:SECONDS >= liftoffTime:SECONDS + controls["verticalAscentTime"] {
			//	Then it changes attitude for an initial pitchover "kick"
			SET steeringVector TO aimAndRoll(HEADING(mission["launchAzimuth"], 90-controls["pitchOverAngle"]):VECTOR, steeringRoll).
			SET ascentFlag TO 1.
			pushUIMessage( "Pitching over by " + ROUND(controls["pitchOverAngle"], 1) + " degrees." ).
		}
	}
	ELSE IF ascentFlag = 1 {
		//	It keeps this attitude until velocity vector matches it closely
		IF TIME:SECONDS < liftoffTime:SECONDS + controls["verticalAscentTime"] + 3 {
			//	Delay this check for the first few seconds to allow the vehicle to pitch away from current prograde
		} ELSE {
			IF controls["pitchOverAngle"] - VANG(SHIP:UP:VECTOR, SHIP:VELOCITY:SURFACE) < 0.1 {
				SET ascentFlag TO 2.
			}
		}
		//	As a safety check - do not stay deadlocked in this state for too long (might be unnecessary).
		IF TIME:SECONDS >= liftoffTime:SECONDS + controls["verticalAscentTime"] + SETTINGS["pitchOverTimeLimit"] {
			SET ascentFlag TO 2.
			pushUIMessage( "Pitchover time limit exceeded!", 5, PRIORITY_HIGH ).
		}
	}
	ELSE IF ascentFlag = 2 {
		//	Enter the minimal angle of attack phase. This case is different only in that we push a transition message.
		SET steeringVector TO minAoASteering(steeringRoll).
		pushUIMessage( "Holding prograde at " + ROUND(mission["launchAzimuth"], 1) + " deg azimuth." ).
		SET ascentFlag TO 3.
	}
	ELSE {
		//	Maintain minimal AoA trajectory
		SET steeringVector TO minAoASteering(steeringRoll).
	}
}

//	Passive pitch control by following a pitch-altitude program
FUNCTION pitchProgramControl {
	//	In this mode, user provides a pitch program as a function of altitude by specifying two lists.
	//	Linear interpolation is used to compute pitch at any altitude.
	//	Expects global variables:
	//	"controls" as lexicon
	//	"steeringVector" as vector

	DECLARE PARAMETER steeringRoll.	//	Expects a scalar

	//	For pretty-printing the linear coefficients
	FUNCTION toSCI {
		DECLARE PARAMETER number.
		DECLARE PARAMETER precision IS 3.

		IF number = 0 RETURN "0.0".

		LOCAL exponent IS FLOOR(LOG10(ABS(number))).
		RETURN ROUND(number*10^(-exponent), precision-1) + "E" + exponent.
	}

	//	Define globals and the first trigger point at first run
	IF NOT (DEFINED pitchProgramIndex) {
		GLOBAL pitchProgramIndex IS -1.	//	Points at the last altitude point
		GLOBAL pitchProgramTrigger IS controls["pitchProgram"]["altitude"][0].
		GLOBAL pitchProgramLength IS controls["pitchProgram"]["altitude"]:LENGTH - 1.
		GLOBAL pitchFactorA IS 0.
		GLOBAL pitchFactorB IS 0.
	}

	//	Recalculate the a and b values of the linear function on each alitude trigger,
	//	but only if we're still within the program (maintain the last coefficients after the final trigger)
	IF SHIP:ALTITUDE > pitchProgramTrigger AND pitchProgramIndex < pitchProgramLength {
		SET pitchProgramIndex TO pitchProgramIndex + 1.

		SET beginPitch TO controls["pitchProgram"]["pitch"][pitchProgramIndex].
		SET beginAlt TO controls["pitchProgram"]["altitude"][pitchProgramIndex].
		SET endPitch TO controls["pitchProgram"]["pitch"][pitchProgramIndex + 1].
		SET endAlt TO controls["pitchProgram"]["altitude"][pitchProgramIndex + 1].

		SET pitchFactorA TO (endPitch - beginPitch) / (endAlt - beginAlt).
		SET pitchFactorB TO endPitch - (pitchFactorA*endAlt).

		SET pitchProgramTrigger TO endAlt.
		pushUIMessage("Pitch following " + toSCI(pitchFactorA) + " * ALT + " + toSCI(pitchFactorB)).
	}

	IF pitchProgramIndex >= 0 {
		SET steeringVector TO aimAndRoll(HEADING(mission["launchAzimuth"], pitchFactorA*SHIP:ALTITUDE+pitchFactorB):VECTOR, steeringRoll).
	}
}
