//	Communication system functions

//	Setup a trigger listening for mesages from other CPUs
FUNCTION setComms {
	WHEN NOT CORE:MESSAGES:EMPTY THEN {
		SET commsEventFlag TO TRUE.
	}
}

//	Handles communication with other CPUs
FUNCTION commsEventHandler {
	//	Get the oldest message in the queue but don't delete it yet
	LOCAL message IS CORE:MESSAGES:PEEK:CONTENT.
	//	Create a response lexicon to send back
	LOCAL response IS LEXICON("type", "response").
	//	Create a lexicon to be populated with data to send back
	LOCAL responseData IS LEXICON().

	IF message["type"] = "request" {
		//	Example message: LEXICON("type", "request", "data", LIST("liftoffTime", "launchAzimuth"), "sender", CORE:TAG).

		//	Making sure data is in correct format
		LOCAL validation IS verifyMessageFormat(message["type"], message["data"], responseData).
		LOCAL data IS validation["verifiedData"].
		SET responseData TO validation["responseData"].

		IF validation["passed"] {
			//	List of available return data
			LOCAL availableData IS LEXICON(
				"liftoffTime", liftoffTime:SECONDS,
				"launchAzimuth", mission["launchAzimuth"],
				"upfgActivation", controls["upfgActivation"],
				"upfgConverged", upfgConverged
			).

			//	Iterate through requested data and add to responseData
			FOR d IN data {
				IF d:ISTYPE("String") {	//	Only do the following if data name is a string
					IF availableData:HASKEY(d) {
						responseData:ADD(d, availableData[d]).
					} ELSE {
						responseData:ADD(d, "ERROR (Requested data not found)").
					}
				} ELSE {				//	Otherwise respond with an error
					responseData:ADD(d, "ERROR (Requested data not passed in as string)").
				}
			}
		}

	} ELSE IF message["type"] = "command" {
		//	Example message: LEXICON("type", "command", "data", LIST("engineShutdown", "engine_1_tag", "engine_2_tag"), "sender", CORE:TAG).

		//	Making sure data is in correct format
		LOCAL validation IS verifyMessageFormat(message["type"], message["data"], responseData).
		LOCAL data IS validation["verifiedData"].
		SET responseData TO validation["responseData"].

		IF validation["passed"] {
			//	Iterate through commands and add to responseData
			FOR d IN data {
				//	If this list is empty, it will be skipped (no error returned as no command name to reference)
				IF NOT d:EMPTY {
					IF d[0]:ISTYPE("String") {	//	Only do the following if command name is a string
						IF availableCommands:HASKEY(d[0]) {
							LOCAL returned IS TRUE.
							IF d:LENGTH > 1 {
								SET returned TO availableCommands[d[0]](d:SUBLIST(1, d:LENGTH-1)).
							} ELSE {
								SET returned TO availableCommands[d[0]]().
							}
							//	If command was executed return value
							responseData:ADD(d[0], returned).
						} ELSE {
							//	Else, return an error
							responseData:ADD(d[0], "ERROR (Command not found)").
						}
					} ELSE {				//	Otherwise respond with an error
						responseData:ADD(d[0], "ERROR (Command name not passed in as string)").
					}
				}
			}
		}

	}

	//	Add all response data to the response
	response:ADD("data", responseData).
	//	Add the sender info
	response:ADD("sender", CORE:TAG).

	if message:HASKEY("sender") {
		LIST PROCESSORS IN CPUs.
		LOCAL senderExists IS FALSE.
		FOR CPU IN CPUs { IF CPU:tag = message["sender"] { SET senderExists TO TRUE. BREAK. } }
		//	Sending a message to specific CPU is only possible if it's on the same vessel. 
		IF senderExists { PROCESSOR(message["sender"]):CONNECTION:SENDMESSAGE(response). }
		//	Otherwise send response to vessel queue instead (this will work even after separation)
		ELSE { message:REMOVE("sender"). }
	}
	IF NOT message:HASKEY("sender") {
		LOCAL msg IS CORE:MESSAGES:PEEK.
		IF msg:HASSENDER {
			msg:SENDER:CONNECTION:SENDMESSAGE(response).
		}
	}

	//	After everything is done, remove the message from message queue
	CORE:MESSAGES:POP.
	//	Reset event flag
	SET commsEventFlag TO FALSE.
	//	Re-create the comms event trigger
	setComms().
}

//	Make sure message is in the correct format
FUNCTION verifyMessageFormat {
	PARAMETER type.
	PARAMETER data.
	PARAMETER responseData.

	IF type = "request" {
		IF NOT data:ISTYPE("List") {
			//	Single string can be passed
			IF data:ISTYPE("String") { SET data TO LIST(data). }
			//	If unrecognised data type is provided, respond with an error
			ELSE { SET responseData TO "ERROR (Unrecognised data format)". }
		//	If list is empty, respod with an error
		} ELSE IF data:EMPTY {
			SET responseData TO "ERROR (List of requested data empty)".
		}
	} ELSE IF type = "command" {
		IF NOT data:ISTYPE("List") {
			//	If user passes the command as a string, program assumes a single command with no parameters.
			IF data:ISTYPE("String") { SET data TO LIST(LIST(data)). }
			ELSE { SET responseData TO "ERROR (Unrecognised data format)". }
		} ELSE IF NOT data:EMPTY {
			IF NOT data[0]:ISTYPE("List") {
				//	If a single command is provided (inside 1 list), insert it into a second list
				IF data[0]:ISTYPE("String") { SET data TO LIST(data). }
				//	If list with unrecognised data type inside it is provided, respond with an error
				ELSE { SET responseData TO "ERROR (Unrecognised data format)". }
			} ELSE IF data:LENGTH = 1 AND data[0]:EMPTY {
				//	If there is only 1 list inside outer list and it's empty, respond with an error
				SET responseData TO "ERROR (List of commands empty)".
			}
		} ELSE {
			//	If list is empty, respod with an error
			SET responseData TO "ERROR (List of commands empty)".
		}
		// Make sure that all inner elements are lists
		FOR d IN data { IF NOT d:ISTYPE("List") { SET responseData TO "ERROR (Unrecognised data format)". } }
	}

	RETURN LEXICON("passed", responseData:ISTYPE("Lexicon"), "verifiedData", data, "responseData", responseData).
}

//	Change time of UPFG activation
FUNCTION command_setUpfgTime {
	PARAMETER params IS LIST().			//	New UPFG activation time (number of seconds after launch)
	IF upfgStage >= 0 { RETURN "ERROR (Command not available in UPFG guidance mode)". }
	IF params:EMPTY { params:ADD(TIME:SECONDS - liftoffTime:SECONDS). }
	IF params[0]:ISTYPE("Scalar") {
		IF params[0] <= 0 { RETURN "ERROR (UPFG cannot be activated before liftoff)". }
		IF params[0] < (TIME:SECONDS - liftoffTime:SECONDS) { SET parms[0] TO TIME:SECONDS - liftoffTime:SECONDS. }
		SET controls["upfgActivation"] TO params[0]. RETURN TRUE.
	} ELSE { RETURN "ERROR (Incorrect parameter type)". }
}

//	Shut down specific engines in flight. Requires engine nametags as parameters
FUNCTION command_engineShutdown {
	PARAMETER engList IS LIST().		//	List of engines (nametags) to shut down - multiple engines tagged with the same name allowed
	IF upfgStage >= 0 { RETURN "ERROR (Command not available in UPFG guidance mode)". }
	IF engList:EMPTY { RETURN "ERROR (Engines list empty)". }
	LOCAL errors IS FALSE.
	LOCAL confirmedEngines IS LIST().	//	Don't shutdown any engines until made all necessary checks
	FOR eng IN engList {
		IF eng:ISTYPE("String") {
			LOCAL en IS SHIP:PARTSTAGGED(eng).
			IF en:EMPTY { SET errors TO TRUE. } ELSE {
				FOR e IN en {
					IF NOT e:ISTYPE("Engine") { SET errors TO TRUE. BREAK. }
					IF e:ALLOWSHUTDOWN {
						IF e:IGNITION { confirmedEngines:ADD(e). }
					} ELSE { SET errors TO TRUE. BREAK. }
				}
			}
		} ELSE { SET errors TO TRUE. BREAK. }
	}
	IF errors { RETURN "ERROR (Could not shutdown specified engines)". }
	//	If nothing went wrong while checking engine, execute the engine shutdown
	FOR eng IN confirmedEngines { eng:SHUTDOWN. }
	RETURN TRUE.
}

//	Change throttle in atmospheric guidance phase
FUNCTION command_setThrottle {
	PARAMETER params IS LIST().	//	Requires desired throttle value and minimum throttle value
	IF upfgStage >= 0 { RETURN "ERROR (Command not available in UPFG guidance mode)". }
	IF params:LENGTH < 2 { RETURN "ERROR (Too few parameters)". }
	IF NOT params[0]:ISTYPE("Scalar") OR NOT params[1]:ISTYPE("Scalar") { RETURN "ERROR (Incorrect parameter type)". }
	IF params[0] > 1 OR params[0] <= 0 OR params[0] > 1 OR params[0] <= 0 { RETURN "ERROR (Parameter values out of range)". }
	IF params[1] = 1 { RETURN "ERROR (Engine unthrottleable)". }
	SET throttleSetting TO (params[0] - params[1]) / (1 - params[1]).
	//	If the algorithm requests a throttle setting lower than minimum limit, we might accidentally shutdown.
	SET throttleSetting TO MAX(throttleSetting, 0.01).
	//	For the GUI printout however, we want to see the final throttle value.
	SET throttleDisplay TO params[0].
	RETURN TRUE.
}

GLOBAL availableCommands IS LEXICON(
	"setUpfgTime", command_setUpfgTime@,
	"engineShutdown", command_engineShutdown@,
	"setThrottle", command_setThrottle@
).