//	Miscellaneous, user interface related functions.

//	Initialize messaging system
GLOBAL uiMessage IS LEXICON("content", "", "time", TIME, "received", FALSE, "printed", FALSE).	//	Currently some of it is unused

FUNCTION createUI {
	CLEARSCREEN.
	PRINT ".-----------------------------------------.".
	PRINT "| PEGAS                          v1.0beta |".
	PRINT "| Powered Explicit Guidance Ascent System |".
	PRINT "|-----------------------------------------|".
	PRINT "|                                         |".
	PRINT "|-----------------------------------------|".
	PRINT "| T   h  m  s |                           |".
	PRINT "|-----------------------------------------|".
	PRINT "| UPFG status  =                          |".
	PRINT "| Throttle     =      %    Tgo =      s   |".
	PRINT "| Acceleration =      m/s  Vgo =      m/s |".
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
	
	textPrint(SHIP:NAME, 4, 2, 41, "L").
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
	
	LOCAL deltaT IS TIME.
	LOCAL sign IS "".
	IF liftoffTime<TIME {
		SET sign TO "+".
		SET deltaT TO TIME - liftoffTime.
	} ELSE {
		SET sign TO "-".
		SET deltaT TO liftoffTime - TIME.
	}
	
	textPrint(sign, 6, 3, 4, "L").
	numberPrint(deltaT:HOUR, 6, 4, 6, 0).
	numberPrint(deltaT:MINUTE, 6, 7, 9, 0).
	numberPrint(deltaT:SECOND, 6, 10, 12, 0).
	textPrint(TIME:CALENDAR+",", 6, 15, 32, "R").
	textPrint(TIME:CLOCK, 6, 33, 41, "R").
}.

//	Just fill in the blanks, do not redraw the whole GUI.
FUNCTION refreshUI {
	//	Expects global variables "liftoffTime" as timespan, "controls", "mission", and "upfgTarget" as lexicon 
	//	Optionally also global variables "throttleSetting" as scalar, "upfgInternal" as lexicon and "upfgConverged" as bool.
	timePrint().
	
	//	First figure out what phase of the flight are we in: passively or actively guided.
	IF NOT (DEFINED upfgInternal) {
		//	Don't try to access any UPFG-related variables
		textPrint("INACTIVE", 8, 17, 41).
		textPrint("N/A", 9, 33, 37, "R").
		textPrint("N/A", 10, 33, 37, "R").
		//	In passive flight, assuming we're low in the atmosphere, print ground speed.
		numberPrint(SHIP:VELOCITY:SURFACE:MAG, 14, 15, 22).
	} ELSE {
		//	Print convergence flag
		IF upfgConverged {
			textPrint("CONVERGED", 8, 17, 41).
		} ELSE {
			textPrint("converging...", 8, 17, 41).
		}.
		numberPrint(upfgInternal["tgo"], 9, 33, 37, 0).
		numberPrint(upfgInternal["vgo"]:MAG, 10, 33, 37, 0).
		//	In active flight we're going to orbit
		numberPrint(SHIP:VELOCITY:ORBIT:MAG, 14, 15, 22).
	}.
	
	//	Print physical information
	//	Read throttle depending on phase (in UPFG flight we have "throttleSetting" global var,
	//	otherwise we're flying in passive mode with initial setting) for other calculations.
	LOCAL throttle_ IS 0.
	IF DEFINED throttleSetting { SET throttle_ TO throttleSetting. }
	ELSE { SET throttle_ TO controls["initialThrottle"]. }
	LOCAL currentAcc IS (SHIP:AVAILABLETHRUST * throttle_) / (SHIP:MASS).
	numberPrint(100*throttle_, 9, 17, 21, 0).
	numberPrint(currentAcc, 10, 17, 21).
	
	//	Print current vehicle state
	numberPrint(SHIP:ALTITUDE/1000, 13, 15, 22).		//	Convert to km
	numberPrint(SHIP:VERTICALSPEED, 15, 15, 22).
	numberPrint(SHIP:ORBIT:PERIAPSIS/1000, 16, 15, 22).	//	Convert to km
	numberPrint(SHIP:ORBIT:APOAPSIS/1000, 17, 15, 22).	//	Convert to km
	numberPrint(SHIP:ORBIT:INCLINATION, 18, 15, 22, 2).
	numberPrint(SHIP:ORBIT:LAN, 19, 15, 22, 2).
	
	//	Print target state
	numberPrint(mission["altitude"], 13, 29, 36).
	numberPrint(upfgTarget["velocity"], 14, 29, 36).
	numberPrint(upfgTarget["velocity"]*SIN(upfgTarget["angle"]), 15, 29, 36).
	numberPrint(mission["periapsis"], 16, 29, 36).
	numberPrint(mission["apoapsis"], 17, 29, 36).
	numberPrint(mission["inclination"], 18, 29, 36, 2).
	numberPrint(mission["LAN"], 19, 29, 36, 2).
	
	//	Calculate and print angle between orbits
	LOCAL currentOrbitNormal IS targetNormal(SHIP:ORBIT:INCLINATION, SHIP:ORBIT:LAN).
	LOCAL relativeAngle IS VANG(currentOrbitNormal, upfgTarget["normal"]).
	numberPrint(relativeAngle, 20, 24, 29, 2).
	
	//	Handle messages
	IF uiMessage["printed"] {
		IF TIME:SECONDS >= uiMessage["time"] {
			textPrint("", 22, 2, 41).
			SET uiMessage["printed"] TO FALSE.
		}
	}
}.

//	Message printing interface
FUNCTION pushUIMessage {
	DECLARE PARAMETER message.	//	Expects a string.
	DECLARE PARAMETER ttl IS 5.	//	Message time-to-live (scalar).
	
	//	Currently the simplest mechanism of overwriting the past message.
	//	TODO: consider a priority-based or queue system.
	textPrint(message, 22, 2, 41).
	//	Set timeout to clear the message
	SET uiMessage["time"] TO TIME:SECONDS + ttl.
	SET uiMessage["printed"] TO TRUE.
}