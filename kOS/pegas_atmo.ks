//	Atmospheric ascent library

//	Basic pitch&hold ascent consists of 4 phases, ascentFlag stores the phase index:
//	0: vertical ascent
//	1: pitch over by a given angle
//	2: hold prograde with a given launch azimuth, emit a UI message
//	3: hold prograde with a given azimuth
GLOBAL ascentFlag IS 0.

//	Passive guidance within the atmosphere
FUNCTION atmosphericSteeringControl {
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
