//	Miscellaneous, user interface related functions.

//	Initialize messaging system
GLOBAL uiMessage IS LEXICON("content", "",
							"timeToLive", 0,
							"priority", 0,
							"deadline", TIME:SECONDS,
							"received", FALSE,
							"printed", FALSE).
//	Initializing "enum" type priorities for messaging system
GLOBAL PRIORITY_LOW IS 0.
GLOBAL PRIORITY_NORMAL IS 1.	//	Default priority
GLOBAL PRIORITY_HIGH IS 2.
GLOBAL PRIORITY_CRITICAL IS 3.
//	Set screen dimensions
SET TERMINAL:WIDTH TO 43.
SET TERMINAL:HEIGHT TO 26 + 14.	//	Few more lines for debugging

FUNCTION createUI {
	CLEARSCREEN.
	PRINT ".-----------------------------------------.".
	PRINT "| PEGAS                          v1.0beta |".
	PRINT "| Powered Explicit Guidance Ascent System |".
	PRINT "|-----------------------------------------|".
	PRINT "| T   h  m  s |                           |".
	PRINT "|-----------------------------------------|".
	PRINT "|                                         |".
	PRINT "|-----------------------------------------|".
	PRINT "| Stage:                          (    s) |".
	PRINT "| UPFG status  =                          |".
	PRINT "| Throttle     =      %    Tgo =      s   |".
	PRINT "| Acceleration =      m/s2 Vgo =      m/s |".
	PRINT "|-----------------------------------------|".
	PRINT "|               Current       Target      |".
	PRINT "| Altitude    |        km   |        km   |".
	PRINT "| Velocity    |        m/s  |        m/s  |".
	PRINT "| Vertical    |        m/s  |        m/s  |".
	PRINT "| Periapsis   |        km   |        km   |".
	PRINT "| Apoapsis    |        km   |        km   |".
	PRINT "| Inclination |        deg  |        deg  |".
	PRINT "| Long. of AN |        deg  |        deg  |".
	PRINT "| Angle between orbits:       deg         |".
	PRINT "|-----------------------------------------|".
	PRINT "|                                         |".
	PRINT "*-----------------------------------------*".
	
	textPrint(SHIP:NAME, 6, 2, 41, "L").
	refreshUI().
}.

//	Print text at the given place on the screen. Pad and trim when needed.
FUNCTION textPrint {
	DECLARE PARAMETER str.			//	Message to write (string).
	DECLARE PARAMETER line.			//	Line to write to (scalar).
	DECLARE PARAMETER start.		//	First column to write to, inclusive (scalar).
	DECLARE PARAMETER end.			//	Last column to write to, exclusive (scalar).
	DECLARE PARAMETER align IS "l".	//	Align: "l"eft or "r"ight.
	
	SET align TO align:TOLOWER().
	LOCAL flen IS end - start.
	//	If message is too long to fit in the field - trim, depending on type.
	IF str:LENGTH>flen {
		IF align="r" { SET str TO str:SUBSTRING(str:LENGTH-flen, flen). }
		ELSE IF align="l" { SET str TO str:SUBSTRING(0, flen). }
	}
	ELSE {
		IF align="r" { SET str TO str:PADLEFT(flen). }
		ELSE IF align="l" { SET str TO str:PADRIGHT(flen). }
	}
	PRINT str AT(start, line).
}.

//	Print a number like textPrint but round to 1 decimal place.
FUNCTION numberPrint {
	DECLARE PARAMETER val.			//	Numer to write (scalar).
	DECLARE PARAMETER line.			//	Line to write to (scalar).
	DECLARE PARAMETER start.		//	First column to write to, inclusive (scalar).
	DECLARE PARAMETER end.			//	Last column to write to, exclusive (scalar).
	DECLARE PARAMETER prec IS 1.	//	Decimal places (scalar)
	DECLARE PARAMETER align IS "r".	//	Align: "l"eft or "r"ight.
	
	LOCAL str IS "" + ROUND(val, prec).
	//	Make sure the number has all the decimal places it needs to have
	IF prec > 0 {
		LOCAL hasZeros IS 0.
		IF str:CONTAINS(".") { SET hasZeros TO str:LENGTH - str:FIND(".") - 1. }
		ELSE { SET str TO str + ".". }
		FROM { LOCAL i IS hasZeros. } UNTIL i = prec STEP { SET i TO i + 1. } DO {
			SET str TO str + "0".
		}
	}
	textPrint(str, line, start, end, align).
}.

//	Specialized function for printing current time and T+.
FUNCTION timePrint {
	//	Expects a global variable "liftoffTime" as timespan.
	
	LOCAL currentTime IS TIME.
	LOCAL deltaT IS currentTime.
	LOCAL sign IS "".
	IF liftoffTime<currentTime {
		SET sign TO "+".
		SET deltaT TO currentTime - liftoffTime.
	} ELSE {
		SET sign TO "-".
		SET deltaT TO liftoffTime - currentTime.
	}
	
	textPrint(sign, 4, 3, 4, "L").
	numberPrint(deltaT:HOUR, 4, 4, 6, 0).
	numberPrint(deltaT:MINUTE, 4, 7, 9, 0).
	numberPrint(deltaT:SECOND, 4, 10, 12, 0).
	textPrint(currentTime:CALENDAR+",", 4, 15, 32, "R").
	textPrint(currentTime:CLOCK, 4, 33, 41, "R").
	
	RETURN currentTime.
}.

//	Just fill in the blanks, do not redraw the whole GUI.
FUNCTION refreshUI {
	//	Expects global variables "liftoffTime" as timespan, "throttleSetting" as scalar, "controls", "mission", "upfgTarget" and "upfgInternal" as lexicon and "upfgConverged" as bool.

	LOCAL currentTime IS timePrint().
	
	//	Section offsets, for easier extendability
	LOCAL vehicleInfoOffset IS 8.	//	Reads: vehicle info section starts at row 8
	LOCAL orbitalInfoOffset IS 14.
	LOCAL currentOrbitOffset IS 15.	//	Horizontal offset for the current orbit info
	LOCAL targetOrbitOffset IS 29.	//	Horizontal offset for the target orbit info
	LOCAL messageBoxOffset IS 23.
	
	//	First figure out what phase of the flight are we in: passively or actively guided.
	IF NOT (DEFINED upfgInternal) {
		//	Don't try to access any UPFG-related variables
		textPrint("INACTIVE", vehicleInfoOffset + 1, 17, 41).
		textPrint("N/A", vehicleInfoOffset + 2, 33, 37, "R").
		textPrint("N/A", vehicleInfoOffset + 3, 33, 37, "R").
		//	In passive flight, assuming we're low in the atmosphere, print ground speed.
		numberPrint(SHIP:VELOCITY:SURFACE:MAG, orbitalInfoOffset + 1, currentOrbitOffset, 22).
		//	Print time until activation of UPFG (only if we're flying at all)
		IF TIME:SECONDS > liftoffTime:SECONDS {
			numberPrint(liftoffTime:SECONDS + controls["upfgActivation"] - upfgConvergenceDelay - currentTime:SECONDS, vehicleInfoOffset, 35, 39, 0).
		}
	} ELSE {
		//	Print convergence flag
		IF stagingInProgress {
			textPrint("STAGING", vehicleInfoOffset + 1, 17, 41).
		} ELSE {
			IF upfgConverged {
				textPrint("CONVERGED", vehicleInfoOffset + 1, 17, 41).
				numberPrint(upfgInternal["tgo"], vehicleInfoOffset + 2, 33, 37, 0).
				numberPrint(upfgInternal["vgo"]:MAG, vehicleInfoOffset + 3, 33, 37, 0).
			} ELSE {
				textPrint("converging...", vehicleInfoOffset + 1, 17, 41).
			}
		}
		//	In active flight we're going to orbit
		numberPrint(SHIP:VELOCITY:ORBIT:MAG, orbitalInfoOffset + 1, currentOrbitOffset, 22).
		//	Print name of the current stage and time till next (just not during staging, this would be confusing)
		IF NOT stagingInProgress {
			textPrint(vehicle[upfgStage]["name"], vehicleInfoOffset, 9, 33 ).
			LOCAL timeToNextStage IS 0.
			IF upfgStage < vehicle:LENGTH - 1 { SET timeToNextStage TO nextStageTime - currentTime:SECONDS. }	//	Time from now to next staging, if we still have any stages to fly
			ELSE { SET timeToNextStage TO nextStageTime + vehicle[upfgStage]["maxT"] - currentTime:SECONDS. }	//	Time from now to (time of activation of the current stage + length of that stage) if this is the last one
			numberPrint(timeToNextStage, vehicleInfoOffset, 35, 39, 0).
		} ELSE { textPrint("", vehicleInfoOffset, 9, 33). }
	}
	
	//	Print physical information
	//	Read throttle depending on phase (in UPFG flight we have "throttleSetting" global var,
	//	otherwise we're flying in passive mode with initial setting) for other calculations.
	LOCAL throttle_ IS 0.
	IF DEFINED throttleSetting { SET throttle_ TO throttleSetting. }
	ELSE { SET throttle_ TO controls["initialThrottle"]. }
	LOCAL currentAcc IS (SHIP:AVAILABLETHRUST * throttle_) / (SHIP:MASS).
	numberPrint(100*throttle_, vehicleInfoOffset + 2, 17, 21, 0).
	numberPrint(currentAcc, vehicleInfoOffset + 3, 17, 21).
	
	//	Print current vehicle state
	numberPrint(SHIP:ALTITUDE/1000,			orbitalInfoOffset + 0, currentOrbitOffset, currentOrbitOffset + 7).
	numberPrint(SHIP:VERTICALSPEED,			orbitalInfoOffset + 2, currentOrbitOffset, currentOrbitOffset + 7).
	numberPrint(SHIP:ORBIT:PERIAPSIS/1000,	orbitalInfoOffset + 3, currentOrbitOffset, currentOrbitOffset + 7).
	numberPrint(SHIP:ORBIT:APOAPSIS/1000,	orbitalInfoOffset + 4, currentOrbitOffset, currentOrbitOffset + 7).
	numberPrint(SHIP:ORBIT:INCLINATION,		orbitalInfoOffset + 5, currentOrbitOffset, currentOrbitOffset + 7, 2).
	numberPrint(SHIP:ORBIT:LAN,				orbitalInfoOffset + 6, currentOrbitOffset, currentOrbitOffset + 7, 2).
	
	//	Print target state
	numberPrint(mission["altitude"],		orbitalInfoOffset + 0, targetOrbitOffset, targetOrbitOffset + 7).
	numberPrint(upfgTarget["velocity"],		orbitalInfoOffset + 1, targetOrbitOffset, targetOrbitOffset + 7).
	numberPrint(upfgTarget["velocity"]*SIN(upfgTarget["angle"]), orbitalInfoOffset + 2, targetOrbitOffset, targetOrbitOffset + 7).
	numberPrint(mission["periapsis"],		orbitalInfoOffset + 3, targetOrbitOffset, targetOrbitOffset + 7).
	numberPrint(mission["apoapsis"],		orbitalInfoOffset + 4, targetOrbitOffset, targetOrbitOffset + 7).
	numberPrint(mission["inclination"],		orbitalInfoOffset + 5, targetOrbitOffset, targetOrbitOffset + 7, 2).
	numberPrint(mission["LAN"],				orbitalInfoOffset + 6, targetOrbitOffset, targetOrbitOffset + 7, 2).
	
	//	Calculate and print angle between orbits
	LOCAL currentOrbitNormal IS targetNormal(SHIP:ORBIT:INCLINATION, SHIP:ORBIT:LAN).
	LOCAL relativeAngle IS VANG(currentOrbitNormal, upfgTarget["normal"]).
	numberPrint(relativeAngle, orbitalInfoOffset + 7, 24, 29, 2).
	
	//	Handle messages
	IF uiMessage["received"] {
		//	If we have any message
		IF NOT uiMessage["printed"] {
			//	If it hasn't been yet printed - do so and set the deadline message
			textPrint(uiMessage["content"], messageBoxOffset, 2, 41).
			SET uiMessage["deadline"] TO currentTime:SECONDS + uiMessage["timeToLive"].
			SET uiMessage["printed"] TO TRUE.
		} ELSE {
			//	If it has already been printed - just check if it should be erased
			IF currentTime:SECONDS >= uiMessage["deadline"] {
				textPrint("", messageBoxOffset, 2, 41).
				SET uiMessage["printed"] TO FALSE.
				SET uiMessage["received"] TO FALSE.
			}
		}
	}
}.

//	Message printing interface
FUNCTION pushUIMessage {
	//	Only passes messages into the system. refreshUI does the actual printing.
	//	If there is a message pending or printed already, an incoming one will be only
	//	accepted (thus overwriting the old one) if it is at least the same priority.
	DECLARE PARAMETER message.		//	Expects a string.
	DECLARE PARAMETER ttl IS 5.		//	Message time-to-live (scalar).
	DECLARE PARAMETER priority IS PRIORITY_NORMAL.
	
	//	If we already have a message - only accept the new one if it's important enough
	IF uiMessage["received"] {
		IF priority >= uiMessage["priority"] {
			SET uiMessage["content"] TO message.
			SET uiMessage["timeToLive"] TO ttl.
			SET uiMessage["priority"] TO priority.
			SET uiMessage["printed"] TO FALSE.
		}
	} ELSE {
		//	Otherwise just a normal message passing
		SET uiMessage["content"] TO message.
		SET uiMessage["timeToLive"] TO ttl.
		SET uiMessage["priority"] TO priority.
		SET uiMessage["received"] TO TRUE.
	}
}