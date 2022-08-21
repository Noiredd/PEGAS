//	Communication system functions

//	Handles communication with other CPUs
FUNCTION commsHandler {
	//	Only run if the message queue is not empty
	IF CORE:MESSAGES:EMPTY {
		RETURN.
	}
	//	Get the oldest message in the queue but don't delete it yet
	LOCAL message IS CORE:MESSAGES:PEEK:CONTENT.
	//	Create a response lexicon to send back
	LOCAL response IS LEXICON("type", "response").
	//	Make sure data is in correct format
	LOCAL validation IS verifyMessageFormat(message["type"], message["data"]).
	LOCAL data IS validation["verifiedData"].
	//	Create a lexicon to be populated with data to send back
	LOCAL responseData IS validation["responseData"].

	//	Handle the message if it passed format validation
	IF validation["passed"] {
		IF message["type"] = "request" {
			//	List of available return data
			LOCAL availableData IS LEXICON(
				"liftoffTime", liftoffTime:SECONDS,
				"launchAzimuth", mission["launchAzimuth"],
				"upfgActivation", controls["upfgActivation"],
				"upfgConverged", upfgConverged
			).
			//	Iterate through requested data and add to responseData
			FOR d IN data {
				//	Check whether the request has the right type and whether we understand it.
				IF d:ISTYPE("String") {
					IF availableData:HASKEY(d) {
						responseData:ADD(d, availableData[d]).
					} ELSE {
						responseData:ADD(d, "ERROR (Requested data not found)").
					}
				} ELSE {
					responseData:ADD(d, "ERROR (Requested data not passed in as string)").
				}
			}
		} ELSE IF message["type"] = "command" {
			//	Iterate through commands and add to responseData
			FOR d IN data {
				//	If this list is empty, it will be skipped (no error returned as no command name to reference)
				IF NOT d:EMPTY {
					//	Check whether the request has the right type
					IF d[0]:ISTYPE("String") {
						IF availableCommands:HASKEY(d[0]) {
							LOCAL returned IS TRUE.
							//	Commands with and without parameters need different calls
							IF d:LENGTH > 1 {
								SET returned TO availableCommands[d[0]](d:SUBLIST(1, d:LENGTH-1)).
							} ELSE {
								SET returned TO availableCommands[d[0]]().
							}
							responseData:ADD(d[0], returned).
						} ELSE {
							responseData:ADD(d[0], "ERROR (Command not found)").
						}
					} ELSE {
						responseData:ADD(d[0], "ERROR (Command name not passed in as string)").
					}
				}
			}
		} ELSE {
			SET responseData TO "ERROR (Unknown message type)".
		}
	}

	//	Add all response data to the response
	response:ADD("data", responseData).
	//	Add the sender info
	response:ADD("sender", CORE:TAG).

	IF message:HASKEY("sender") {
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
}

//	Make sure message is in the correct format
FUNCTION verifyMessageFormat {
	PARAMETER type.
	PARAMETER data.

	//	Response is a lexicon by default. If we encounter any error, it will change type to a string,
	//	and this is how we know that the format verification failed.
	SET responseData TO LEXICON().

	IF type = "request" {
		//	Make sure even a single request is inside a list, and if it was passed as a list - that it's not empty.
		IF NOT data:ISTYPE("List") {
			SET data TO LIST(data).
		} ELSE IF data:EMPTY {
			SET responseData TO "ERROR (List of requested data empty)".
		}
	} ELSE IF type = "command" {
		//	A command consists of a string (name of a command) and optionally arguments of any type.
		//	Commands data has to eventually become a list of lists. User can input either:
		//	 *	single string:	a no-arguments command
		//	 *	single list:	a single command with arguments
		//	 *	list of lists:	several commands, with or without arguments
		IF NOT data:ISTYPE("List") {
			//	Single string case
			IF data:ISTYPE("String") { SET data TO LIST(LIST(data)). }
			ELSE { SET responseData TO "ERROR (Unrecognised data format)". }
		} ELSE IF NOT data:EMPTY {
			IF NOT data[0]:ISTYPE("List") {
				//	Single list case - reject if does not begin with a string
				IF data[0]:ISTYPE("String") { SET data TO LIST(data). }
				ELSE { SET responseData TO "ERROR (Unrecognised data format)". }
			} ELSE IF data:LENGTH = 1 AND data[0]:EMPTY {
				//	List of lists case - reject if it only contains an empty list
				SET responseData TO "ERROR (List of commands empty)".
			}
		} ELSE {
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
		IF params[0] < (TIME:SECONDS - liftoffTime:SECONDS) { SET params[0] TO TIME:SECONDS - liftoffTime:SECONDS. }
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
