//	Event handling library.

//	Utility to insert an event into the sequence by time order
FUNCTION insertEvent {
	//	Expects a global variable "sequence" as list of lexicons.
	DECLARE PARAMETER event.		//	Expects a lexicon

	LOCAL eventTime IS event["time"].
	//	Go over the list to find the index of insertion...
	LOCAL index IS 0.
	FOR this IN sequence {
		IF eventTime < this["time"] {
			//	This will cause insertion of the new item AFTER any other item with exact same timing
			BREAK.
		}
		SET index TO index + 1.
	}
	//	...and insert there.
	sequence:INSERT(index, event).
}

//	Create sequence entries for staging events
FUNCTION spawnStagingEvents {
	//	For each active stage we have to schedule the preStage event and the staging event - except the FIRST ONE which is already
	//	preStaged by now and we just need to stage it (which will possibly be a no-op if it's a sustainer). We'll iterate over the
	//	vehicle, computing (cumulatively) burnout times for each stage and spawning events. We know exactly when everything starts:
	//	`controls["upfgActivation"]`.
	//	Expects global variables:
	//	"controls" as lexicon
	//	"vehicle" as list
	//	"stagingKillRotTime" as scalar
	LOCAL stageActivationTime IS controls["upfgActivation"].
	LOCAL vehicleIterator IS vehicle:ITERATOR.
	//	The first active stage is already pre-staged so we only need to create the staging event
	vehicleIterator:NEXT.
	LOCAL stagingEvent IS LEXICON(
		"time", stageActivationTime,
		"type", "_upfgstage",
		"isVirtual", vehicleIterator:VALUE["isVirtualStage"],
		"message", "active guidance on" // todo: clarify all messages related to staging and virtual stages
	).
	//	Insert it into sequence
	insertEvent(stagingEvent).
	//	Compute burnout time for this stage and add to sAT (this involves activation time and burn time)
	SET stageActivationTime TO stageActivationTime + getStageDelays(vehicleIterator:VALUE) + vehicleIterator:VALUE["maxT"].
	//	Loop over remaining stages
	UNTIL NOT vehicleIterator:NEXT {
		//	Construct & insert pre-stage event
		LOCAL stagingTransitionTime IS stagingKillRotTime.
		IF vehicleIterator:VALUE["isVirtualStage"] { SET stagingTransitionTime TO 2. }
		LOCAL stagingEvent IS LEXICON(
			"time", stageActivationTime - stagingTransitionTime,
			"type", "_prestage",
			"isVirtual", vehicleIterator:VALUE["isVirtualStage"],
			"message", vehicleIterator:VALUE["name"]
		).
		insertEvent(stagingEvent).
		//	Construct & insert staging event
		LOCAL stagingEvent IS LEXICON(
			"time", stageActivationTime,
			"type", "_upfgstage",
			"isVirtual", vehicleIterator:VALUE["isVirtualStage"],
			"message", vehicleIterator:VALUE["name"]
		).
		insertEvent(stagingEvent).
		//	Compute next stage time
		SET stageActivationTime TO stageActivationTime + getStageDelays(vehicleIterator:VALUE) + vehicleIterator:VALUE["maxT"].
	}
}

//	Executes a scheduled sequence/staging event.
FUNCTION eventHandler {
	//	Clock-based mechanics that is uniform across calls (first call is just like any other) and fully deterministic.
	//	First we check if we have any events left to execute, and if so - what time should we execute it at.
	//	Finally, we check whether it's time to handle it, and if so - dispatch to specific handling subroutine.
	//	Expects global variables:
	//	"sequence" as list
	//	"vehicle" as list
	//	"liftoffTime" as timespan
	//	"stagingInProgress" as bool
	//	"steeringRoll" as scalar
	//	"throttleSetting" as scalar
	//	"throttleDisplay" as scalar
	//	"upfgStage" as scalar
	//	"eventPointer" as scalar
	LOCAL nextEventPointer IS eventPointer + 1.
	IF nextEventPointer >= sequence:LENGTH {
		RETURN.	//	No more events in the sequence
	}
	LOCAL nextEventTime IS liftoffTime:SECONDS + sequence[nextEventPointer]["time"].
	IF TIME:SECONDS < nextEventTime {
		RETURN.	//	Not yet time to handle this one.
	}

	//	If we got this far, means it's time to handle the event
	LOCAL event IS sequence[nextEventPointer].
	LOCAL eType IS event["type"].
	IF      eType = "print" OR eType = "p" { }
	ELSE IF eType = "stage" OR eType = "s" {
		STAGE.
	}
	ELSE IF eType = "jettison" OR eType = "j" {
		STAGE.
	}
	ELSE IF eType = "throttle" OR eType = "t" {
		userEvent_throttle(event).
	}
	ELSE IF eType = "shutdown" OR eType = "u" {
		userEvent_shutdown(event).
	}
	ELSE IF eType = "roll" OR eType = "r" {
		userEvent_roll(event).
	}
	ELSE IF eType = "delegate" OR eType = "d" {
		userEvent_delegate(event).
	}
	ELSE IF eType = "_prestage" {
		internalEvent_preStage().
	}
	ELSE IF eType = "_upfgstage" {
		internalEvent_staging().
	}
	ELSE {
		pushUIMessage( "Unknown event type (" + eType + ", message='" + event["message"] + "')!", 5, PRIORITY_HIGH ).
	}

	//	Print event message, if requested
	IF event:HASKEY("message") {
		pushUIMessage( event["message"] ).
	}

	//	Mark the event as handled by incrementing the pointer
	SET eventPointer TO eventPointer + 1.

	//	Run again to handle "overlapping" events
	//	This is not infinite recursion, because if there is no overlapping event to handle, the next call will RETURN early.
	eventHandler().
}

//	EVENT HANDLING SUBROUTINES

//	Handle the pre-staging event
FUNCTION internalEvent_preStage {
	//	Switch to staging mode, increment the stage counter and force UPFG reconvergence.
	//	Rationale is not changed: we want to maintain constant attitude while the current stage is still burning,
	//	but at the same time start converging guidance for the subsequent stage.
	SET stagingInProgress TO TRUE.
	SET upfgStage TO upfgStage + 1.
	SET upfgConverged TO FALSE.
	usc_convergeFlags:CLEAR().
}

//	Handle the physical staging event
FUNCTION internalEvent_staging {
	//	Staging event is a potentially complex procedure consisting of several steps.
	//	Instead of breaking it down into multiple individual events (understood as sequence items), we use the
	//	trigger mechanism to handle them. When staging occurs, several triggers are scheduled to handle every
	//	single step of the procedure in a timely manner.
	LOCAL currentTime IS TIME:SECONDS.
	LOCAL event IS vehicle[upfgStage]["staging"].
	LOCAL stageName IS vehicle[upfgStage]["name"].
	LOCAL eventDelay IS 0.	//	Keep track of time between subsequent events.
	IF upfgStage > 0 AND vehicle[upfgStage-1]["shutdownRequired"] {
		SET throttleSetting TO 0.
		SET throttleDisplay TO 0.
	}
	IF event["jettison"] {
		GLOBAL stageJettisonTime IS currentTime + event["waitBeforeJettison"].
		WHEN TIME:SECONDS >= stageJettisonTime THEN {
			STAGE.
			pushUIMessage(stageName + " - separation").
		}
		SET eventDelay TO eventDelay + event["waitBeforeJettison"].
	}
	IF event["ignition"] {
		IF event["ullage"] = "rcs" {
			GLOBAL ullageIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= ullageIgnitionTime THEN {
				RCS ON.
				SET SHIP:CONTROL:FORE TO 1.0.
				pushUIMessage(stageName + " - RCS ullage on").
			}
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["ullageBurnDuration"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {
				STAGE.
				updateStageEndTime().
				SET stagingInProgress TO FALSE.
				pushUIMessage(stageName + " - ignition").
			}
			SET eventDelay TO eventDelay + event["ullageBurnDuration"].
			GLOBAL ullageShutdownTime IS currentTime + eventDelay + event["postUllageBurn"].
			WHEN TIME:SECONDS >= ullageShutdownTime THEN {
				SET SHIP:CONTROL:FORE TO 0.0.
				RCS OFF.
				pushUIMessage(stageName + " - RCS ullage off").
			}
		} ELSE IF event["ullage"] = "srb" {
			GLOBAL ullageIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= ullageIgnitionTime THEN {
				STAGE.
				pushUIMessage(stageName + " - SRB ullage ignited").
			}
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["ullageBurnDuration"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {
				STAGE.
				updateStageEndTime().
				SET stagingInProgress TO FALSE.
				pushUIMessage(stageName + " - ignition").
			}
			SET eventDelay TO eventDelay + event["ullageBurnDuration"].
		} ELSE IF event["ullage"] = "none" {
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {
				STAGE.
				updateStageEndTime().
				SET stagingInProgress TO FALSE.
				pushUIMessage(stageName + " - ignition").
			}
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
		} ELSE {
			pushUIMessage( "Unknown event type (" + event["ullage"] + ")!", 5, PRIORITY_HIGH ).
		}
	} ELSE {
		//	If this event does not need ignition, staging is over at this moment
		SET stagingInProgress TO FALSE.
		//	If this was the sustainer stage activation event, we also have to:
		updateStageEndTime().
	}

	//	Print messages for regular stages and constant-acceleration mode activation.
	IF NOT vehicle[upfgStage]["isVirtualStage"] {
		pushUIMessage(stageName + " - activation").
	} ELSE IF vehicle[upfgStage]["mode"] = 2 {
		pushUIMessage("Constant acceleration mode activated.").
	}
}

//	Handle the throttle event
FUNCTION userEvent_throttle {
	//	Throttling is only allowed during the passive guidance phase, as it would ruin burn time predictions used
	//	for guidance and stage timing.
	DECLARE PARAMETER event.	//	Expects a lexicon

	IF upfgStage < 0 {
		IF NOT event:HASKEY("message") {
			IF event["throttle"] < throttleSetting {
				event:ADD("message", "Throttling down to " + 100*event["throttle"] + "%").
			} ELSE {
				event:ADD("message", "Throttling up to " + 100*event["throttle"] + "%").
			}
		}
		SET throttleSetting TO event["throttle"].
		SET throttleDisplay TO throttleSetting.
	} ELSE {
		pushUIMessage( "Throttle ignored in active guidance!", 5, PRIORITY_HIGH ).
	}
}

//	Handle the engine shutdown event
FUNCTION userEvent_shutdown {
	DECLARE PARAMETER event.	//	Expects a lexicon

	//	Find the tagged engines
	LOCAL taggedEngines IS SHIP:PARTSTAGGED(event["engineTag"]).
	IF taggedEngines:LENGTH = 0 {
		pushUIMessage("NO ENGINES TAGGED '" + event["engineTag"] + "' FOUND!", 10, PRIORITY_CRITICAL).
	}
	//	Shut them down
	FOR engine IN taggedEngines {
		engine:SHUTDOWN().
	}
	IF NOT event:HASKEY("message") {
		event:ADD("message", "Shutting down engine(s) tagged '" + event["engineTag"] + "'.").
	}
}

//	Handle the roll event
FUNCTION userEvent_roll {
	DECLARE PARAMETER event.	//	Expects a lexicon

	SET steeringRoll TO event["angle"].
	IF NOT event:HASKEY("message") {
		event:ADD("message", "Rolling to " + steeringRoll + " degrees").
	}
}

//	Handle the delegate event
FUNCTION userEvent_delegate {
	DECLARE PARAMETER event.	//	Expects a lexicon

	LOCAL fun IS event["function"].
	IF fun:ISDEAD() {
		pushUIMessage("DEAD DELEGATE - UNABLE TO CALL", 10, PRIORITY_CRITICAL).
	} ELSE {
		fun:CALL().
	}
}
