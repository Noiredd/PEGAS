//	Miscellaneous, user interface related functions.

//	Initialize messaging system
GLOBAL uiMessage IS LEXICON(
	"content", "",
	"timeToLive", 0,
	"priority", 0,
	"deadline", TIME:SECONDS,
	"received", FALSE,
	"printed", FALSE
).
//	Initialize "enum" type priorities for messaging system
GLOBAL PRIORITY_LOW IS 0.
GLOBAL PRIORITY_NORMAL IS 1.	//	Default priority
GLOBAL PRIORITY_HIGH IS 2.
GLOBAL PRIORITY_CRITICAL IS 3.
//	Set screen dimensions
SET TERMINAL:WIDTH TO 43.
SET TERMINAL:HEIGHT TO 45.		//	Few more lines for debugging
//	Flight plan display
GLOBAL lastEventHandled IS -2.	//	Flight plan is redrawn whenever a change in eventPointer is observed
GLOBAL printMecoEvent IS FALSE.	//	In active guidance phase, this is how we'll know to update time on the MECO event placeholder
GLOBAL printableEvents IS LIST().

//	Redraw the UI and optionally refresh
FUNCTION createUI {
	DECLARE PARAMETER refresh IS TRUE.	//	Expects a boolean
	CLEARSCREEN.
	PRINT ".-----------------------------------------.".
	PRINT "| PEGAS                                   |".
	PRINT "| Powered Explicit Guidance Ascent System |".
	PRINT "|-----------------------------------------|".
	PRINT "|                                         |".	//	Vehicle name
	PRINT "|-----------------------------------------|".
	PRINT "| T   h  m  s                  |          |".	//	Current time
	PRINT "|-----------------------------------------|".
	PRINT "| Stage:                                  |".	//	Vehicle info
	PRINT "| Stage type:                             |".
	PRINT "| Veh. status:                            |".
	PRINT "| UPFG status:                            |".
	PRINT "| Tgo(stage)   =      s    Tgo =      s   |".
	PRINT "| Throttle     =      %    Vgo =      m/s |".
	PRINT "| Acceleration =      m/s2 (     G)       |".
	PRINT "|-----------------------------------------|".
	PRINT "|               Current       Target      |".
	PRINT "| Altitude    |        km   |        km   |".	//	Orbital info
	PRINT "| Velocity    |        m/s  |        m/s  |".
	PRINT "| Vertical    |        m/s  |        m/s  |".
	PRINT "| Periapsis   |        km   |        km   |".
	PRINT "| Apoapsis    |        km   |        km   |".
	PRINT "| Inclination |        deg  |        deg  |".
	PRINT "| Long. of AN |        deg  |        deg  |".
	PRINT "| Angle between orbits:       deg         |".
	PRINT "|-----------------------------------------|".
	PRINT "|                                         |".	//	Message box
	PRINT "|-----------------------------------------|".
	PRINT "|                                         |".	//	Flight schedule box
	PRINT "|                                         |".
	PRINT "|                                         |".
	PRINT "|                                         |".
	PRINT "|                                         |".
	PRINT "*-----------------------------------------*".

	textPrint(SHIP:NAME, 4, 2, 41, "L").
	textPrint(_PEGAS_VERSION_, 1, 20, 41, "R").
	IF refresh {
		refreshUI().
	}
}

//	Print text at the given place on the screen. Pad and trim when needed.
FUNCTION textPrint {
	DECLARE PARAMETER str.			//	Message to write (string).
	DECLARE PARAMETER line.			//	Line to write to (scalar).
	DECLARE PARAMETER start.		//	First column to write to, inclusive (scalar).
	DECLARE PARAMETER end.			//	Last column to write to, exclusive (scalar).
	DECLARE PARAMETER align IS "L".	//	Align: "L"eft or "R"ight.

	SET align TO align:TOUPPER.
	LOCAL flen IS end - start.
	//	If message is too long to fit in the field - trim, depending on type.
	IF str:LENGTH>flen {
		IF align="R" { SET str TO str:SUBSTRING(str:LENGTH-flen, flen). }
		ELSE IF align="L" { SET str TO str:SUBSTRING(0, flen). }
	}
	ELSE {
		IF align="R" { SET str TO str:PADLEFT(flen). }
		ELSE IF align="L" { SET str TO str:PADRIGHT(flen). }
	}
	PRINT str AT(start, line).
}

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
}

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
		SET deltaT TO liftoffTime - currentTime + 1.
	}

	textPrint(sign, 6, 3, 4, "L").
	numberPrint(deltaT:HOUR, 6, 4, 6, 0).
	numberPrint(deltaT:MINUTE, 6, 7, 9, 0).
	numberPrint(deltaT:SECOND, 6, 10, 12, 0).
	textPrint("(T" + sign + ROUND(deltaT:SECONDS, 1) + ")", 6, 14, 30).
	textPrint(currentTime:CLOCK, 6, 33, 41, "R").

	RETURN currentTime.
}

//	Just fill in the blanks, do not redraw the whole GUI.
FUNCTION refreshUI {
	//	Expects global variables:
	//	"liftoffTime" as timespan
	//	"throttleDisplay" as scalar
	//	"controls" as lexicon
	//	"mission" as lexicon
	//	"upfgTarget" as lexicon
	//	"upfgInternal" as lexicon
	//	"activeGuidanceMode" as bool
	//	"upfgConverged" as bool
	//	"upfgEngaged" as bool
	//	"stagingInProgress" as bool
	//	"prestageHold" as bool

	//	Print and acquire current time
	LOCAL currentTime IS timePrint().

	//	Section offsets, for easier extendability
	LOCAL vehicleInfoOffset IS 8.	//	Reads: vehicle info section starts at row 8
	LOCAL orbitalInfoOffset IS vehicleInfoOffset + 9.
	LOCAL currentOrbitOffset IS 15.	//	Horizontal offset for the current orbit info
	LOCAL targetOrbitOffset IS 29.	//	Horizontal offset for the target orbit info
	LOCAL messageBoxOffset IS orbitalInfoOffset + 9.
	LOCAL flightPlanOffset IS messageBoxOffset + 2.

	//	Vehicle info fields depend on the current flight phase (passive/active guidance).
	//	For clarity, first acquire all the necessary information.
	LOCAL isFlying IS TIME:SECONDS > liftoffTime:SECONDS.
	LOCAL isActive IS FALSE.
	LOCAL stageName IS "".
	LOCAL stageType IS "".
	LOCAL stageVirtual IS FALSE.
	LOCAL vehicleStatus IS "".
	LOCAL upfgStatus IS "".
	LOCAL stageTgo IS 0.
	LOCAL totalTgo IS 0.
	LOCAL totalVgo IS 0.
	//	Exception: the following is not a vehicle param but orbital info. We still check it
	//	here, as we want to display different velocities: in passive (=atmospheric) phase
	//	the surface velocity is of interest, but in active phase it's the orbital one.
	LOCAL currentVelocity IS 0.
	//	Figure out what phase of the flight are we in.
	IF NOT (DEFINED upfgInternal) {
		//	Passive guidance phase
		SET stageName TO "".
		SET stageType TO "passively guided".
		SET vehicleStatus TO "nominal".
		SET upfgStatus TO "inactive".
		// Time until activation of UPFG
		SET stageTgo TO liftoffTime:SECONDS + controls["upfgActivation"] - currentTime:SECONDS.
		SET currentVelocity TO SHIP:VELOCITY:SURFACE:MAG.
	} ELSE {
		SET isActive TO TRUE.
		SET stageName TO vehicle[upfgStage]["name"].
		SET stageType TO vehicle[upfgStage]["virtualStageType"].
		SET stageVirtual TO vehicle[upfgStage]["isVirtualStage"].
		SET stageSustainer TO vehicle[upfgStage]["isSustainer"].
		SET currentVelocity TO SHIP:VELOCITY:ORBIT:MAG.
		//	Time until the stage burns out (basing on ignition time and cumulative burn time - can be off by 1-2s)
		IF stageEndTime > currentTime {
			//	No matter what state we're in, stagingInProgress or not (as long as it's active guidance),
			//	stageEndTime is greater or equal to currentTime in all but one cases: when staging is indeed
			//	in progress and the previous stage has already burnt out but the next one has not ignited yet.
			//	In this situation stageEndTime has not been updated to the proper value yet, so the difference
			//	does not make any sense.
			SET stageTgo TO (stageEndTime - currentTime):SECONDS.
		} ELSE {
			SET stageTgo TO 0.
		}
		//	Print vehicle status flag
		IF stagingInProgress {
			IF stageVirtual {
				//                    virtual (post-jettison)xxx
				SET vehicleStatus TO "virtual stage transition".
			}
			ELSE IF stageSustainer {
				SET vehicleStatus TO "sustainer - preconvergence".
			}
			ELSE {
				SET vehicleStatus TO CHOOSE "preparing to stage" IF prestageHold ELSE "staging".
			}
		}
		ELSE {
			SET vehicleStatus TO "NOMINAL".
		}
		//	Print UPFG convergence flag
		IF upfgConverged {
			SET upfgStatus TO CHOOSE "ENGAGED" IF upfgEngaged ELSE "converged & waiting".
			SET totalTgo TO upfgInternal["tgo"].
			SET totalVgo TO upfgInternal["vgo"]:MAG.
		} ELSE {
			SET upfgStatus TO "converging...".
		}
	}

	//	Print physical information
	textPrint(stageName, vehicleInfoOffset + 0, 9, 41).
	textPrint(stageType, vehicleInfoOffset + 1, 15, 41).
	textPrint(vehicleStatus, vehicleInfoOffset + 2, 15, 41).
	textPrint(upfgStatus, vehicleInfoOffset + 3, 15, 41).
	IF isFlying {
		//	Don't print time until next event while we're still on the ground
		numberPrint(stageTgo, vehicleInfoOffset + 4, 17, 21, 0).
	}
	IF NOT isActive {
		textPrint("N/A", vehicleInfoOffset + 4, 33, 37, "R").
		textPrint("N/A", vehicleInfoOffset + 5, 33, 37, "R").
	} ELSE IF upfgConverged {
		numberPrint(totalTgo, vehicleInfoOffset + 4, 33, 37, 0).
		numberPrint(totalVgo, vehicleInfoOffset + 5, 33, 37, 0).
	}
	LOCAL throttle_ IS throttleDisplay.
	LOCAL currentAcc IS (SHIP:AVAILABLETHRUST * throttle_) / (SHIP:MASS).
	numberPrint(100*throttle_, vehicleInfoOffset + 5, 17, 21, 0).
	numberPrint(currentAcc, vehicleInfoOffset + 6, 17, 21).
	numberPrint(currentAcc / CONSTANT:g0, vehicleInfoOffset + 6, 28, 32, 1).

	//	Print current vehicle orbital info
	numberPrint(SHIP:ALTITUDE/1000,			orbitalInfoOffset + 0, currentOrbitOffset, currentOrbitOffset + 7).
	numberPrint(currentVelocity,			orbitalInfoOffset + 1, currentOrbitOffset, currentOrbitOffset + 7).
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

	//	Update the flight plan (this function can return early if there were no significant changes)
	flightPlanPrint(flightPlanOffset, currentTime).
}

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

//	Building messages for flight plan printing
FUNCTION makeMessage {
	DECLARE PARAMETER event.

	LOCAL eType IS event["type"].
	IF      eType = "print" OR eType = "p" { }
	ELSE IF eType = "stage" OR eType = "s" {
		RETURN "Staging event".
	}
	ELSE IF eType = "jettison" OR eType = "j" {
		RETURN "Jettison (" + event["massLost"] + "kg)".
	}
	ELSE IF eType = "throttle" OR eType = "t" {
		RETURN "Set throttle to " + event["throttle"].
	}
	ELSE IF eType = "shutdown" OR eType = "u" {
		RETURN "Engine shutdown: " + event["engineTag"].
	}
	ELSE IF eType = "roll" OR eType = "r" {
		RETURN "Roll to " + event["angle"] + " degrees".
	}
	ELSE IF eType = "delegate" OR eType = "d" {
		RETURN "Delegate call".
	}
	ELSE IF eType = "action" OR eType = "a" {
		RETURN "Action group " + event["action"].
	}
	ELSE IF eType = "_upfgstage" {
		RETURN event["fpMessage"].
	}
	ELSE IF eType = "_activeon" {
		RETURN "UPFG activation".
	}
}

//	Flight plan construction
FUNCTION buildFlightPlan {
	//	Extracts printable events from the sequence and converts them into event-like structures more suitable
	//	for display in the UI. The resulting lexicons contain a pre-constructed message string and an integer
	//	index to link back to the source position within the sequence.
	//	Depending on the boolean switch, it either creates a placeholder entry for UPFG activation time
	//	(for passive guidance phase, when the actual event has not yet been spawned), or a placeholder entry
	//	for the final MECO (which is later updated as UPFG converges).
	//	Expects global variables:
	//	"controls" as lexicon
	//	"sequence" as list
	//	"printableEvents" as list
	//	"lastEventHandled" as integer

	DECLARE PARAMETER isPassiveGuidance IS FALSE.	//	Expects a boolean
	//	Reset the list and last handled pointer (this forces a redraw)
	printableEvents:CLEAR().
	SET lastEventHandled TO -2.
	//	First, extract the printable events
	LOCAL i IS 0.
	FOR event IN sequence {
		IF event:HASKEY("isHidden") AND event["isHidden"] {} ELSE {
			LOCAL pEvent IS event:COPY().
			//	Add the source index
			pEvent:ADD("id", i).
			//	Add a pre-constructed time string
			pEvent:ADD("tstr", "" + ROUND(ABS(event["time"]), 1)).
			printableEvents:ADD(pEvent).
		}
		SET i TO i + 1.
	}

	//	Depending on guidance mode, insert a placeholder either for UPFG activation, or for final MECO.
	IF isPassiveGuidance {
		SET i TO 0.
		FOR this IN printableEvents {
			IF controls["upfgActivation"] < this["time"] {
				BREAK.
			}
			SET i TO i + 1.
		}
		printableEvents:INSERT(i, LEXICON(
			"id", printableEvents[i]["id"] + 1,	//	This is really irrelevant as we cannot possibly iterate past this
			"time", controls["upfgActivation"],
			"tstr", "" + ROUND(ABS(controls["upfgActivation"]), 1),
			"type", "_activeon"	//	Instead of message directly, so we always get the same string from makeMessage
		)).
	} ELSE {
		printableEvents:ADD(LEXICON(
			"id", 1000,		//	Irrelevant, it just needs to be impossibly high
			"type", "_meco",
			"time", 0,		//	Irrelevant at this moment too, will be recalculated and updated when UPFG converges
			"tstr", "N/A",
			"message", "FINAL ENGINE CUTOFF"
		)).
	}

	//	Find the longest time string to calculate padding
	LOCAL longest IS 0.
	FOR event IN printableEvents {
		LOCAL len IS event["tstr"]:LENGTH.
		IF len > longest { SET longest TO len. }
	}
	//	If we have the MECO event, it will be useful to store the padding value within it (for recalculation later)
	IF NOT isPassiveGuidance {
		printableEvents[printableEvents:LENGTH - 1]:ADD("padding", longest).
	}

	//	Assemble the printable line for each event
	FOR event IN printableEvents {
		LOCAL isneg IS event["time"] < 0.
		LOCAL timestr IS (CHOOSE "T-" IF isneg ELSE "T+") + event["tstr"]:PADRIGHT(longest).
		LOCAL message IS CHOOSE event["message"] IF event:HASKEY("message") ELSE makeMessage(event).
		event:ADD("line", " " + timestr + " " + message).
	}
}

//	Flight plan display
FUNCTION flightPlanPrint {
	//	Refresh the flight plan box and the "upcoming event" tick mark.
	//	Recalculate time on placeholder MECO event.
	//	Exit without printing if there were no changes: either to the last handled event, or the (visible!) Tgo estimation.
	//	Expects global variables:
	//	"eventPointer" as integer
	//	"liftoffTime" as timespan
	//	"printableEvents" as list
	//	"printMecoEvent" as bool
	//	"upfgConverged" as bool
	//	"upfgInternal" as lexicon
	DECLARE PARAMETER offset.		//	Expects an integer
	DECLARE PARAMETER currentTime.	//	Expects a timespan

	//	All the checks whether we should exit without printing will modify this flag
	//	(it's initialized such that if there was an event change, we'll have TRUE here)
	LOCAL needToRefresh IS lastEventHandled <> eventPointer.

	//	Figure out where on the printableEvents list we are, given the eventPointer value
	LOCAL printableEventPointer IS -1.
	FOR event in printableEvents {
		IF event["id"] > eventPointer {
			BREAK.
		}
		SET printableEventPointer TO printableEventPointer + 1.
	}

	//	Select a subrange of events to print. We want:
	//	* 1 past event and 4 future events normally (case 3),
	//	* 5 future events if there are no past events (case 1),
	//	* 5 final events if there are less than 5 future events (case 2).
	//	We also need to have a sublist pointer to the upcoming event.
	LOCAL showEvents IS LIST().
	LOCAL upcomingPointer IS 0.
	IF printableEventPointer = -1 {
		SET showEvents TO printableEvents:SUBLIST(0, 5).
	}
	ELSE IF printableEventPointer > printableEvents:LENGTH - 5 {
		SET showEvents TO printableEvents:SUBLIST(printableEvents:LENGTH - 5, 5).
		SET upcomingPointer TO printableEventPointer - printableEvents:LENGTH + 6.
	}
	ELSE {
		SET showEvents TO printableEvents:SUBLIST(printableEventPointer, 5).
		SET upcomingPointer TO 1.
	}

	//	If MECO event isn't visible yet, check if it should be (conveniently, it has to be the last event)
	IF NOT printMecoEvent {
		IF showEvents[showEvents:LENGTH - 1]["type"] = "_meco" { SET printMecoEvent TO TRUE. }
	}

	//	Update the MECO estimation if we have one, even behind the scenes (useful for then this placeholder
	//	event becomes visible for the first time - we'll have the last good estimate already here).
	IF upfgConverged {
		//	This cannot possibly happen before we transitioned into active guidance, and hence got the sequence
		//	and thus printableEvents updated to contain the MECO placeholder -> so no worries about indexing.
		LOCAL newMeco IS ROUND((currentTime - liftoffTime):SECONDS + upfgInternal["tgo"], 1).
		LOCAL oldMeco IS printableEvents[printableEvents:LENGTH - 1]["time"].
		IF ABS(newMeco - oldMeco) > 0.3 {
			LOCAL mecoEvent IS printableEvents[printableEvents:LENGTH - 1].
			SET mecoEvent["time"] TO newMeco.
			SET mecoEvent["tstr"] TO "" + newMeco.
			LOCAL padding IS MAX(mecoEvent["tstr"]:LENGTH, mecoEvent["padding"]).
			SET mecoEvent["line"] TO " T+" + mecoEvent["tstr"]:PADRIGHT(padding) + " " + mecoEvent["message"].
			//	In case the event is already visible on screen - force refresh
			IF printMecoEvent {
				SET needToRefresh TO TRUE.
			}
		}
	}

	//	In case we found no reasons to print anything - exit early
	IF NOT needToRefresh { RETURN. }

	//	Print the event list and add the upcoming event pointer
	LOCAL i IS 0.
	FOR event IN showEvents {
		textPrint(event["line"], offset + i, 1, 41).	//	Start printing on margin (1) to erase the old tick mark
		SET i TO i + 1.
	}
	textPrint(">", offset + upcomingPointer, 1, 2).

	//	Finally, mark that we refreshed after this event (cannot happen earlier because of the possible recursion)
	SET lastEventHandled TO eventPointer.
}
