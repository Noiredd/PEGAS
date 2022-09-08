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

//	Create countdown print events
FUNCTION spawnCountdownEvents {
	//	Generates print events for a standard countdown sequence.
	//	Expects global variables:
	//	"liftoffTime" as scalar
	//	"sequence" as list

	FUNCTION makeEvent {
		DECLARE PARAMETER timeAfterLiftoff.	//	Expects a scalar
		DECLARE PARAMETER message.			//	Expects a string

		RETURN LEXICON(
			"time", timeAfterLiftoff,
			"type", "print",
			"message", message,
			"isHidden", TRUE
		).
	}

	LOCAL timeToLaunch IS liftoffTime:SECONDS - TIME:SECONDS.
	IF timeToLaunch > 18000 {insertEvent(makeEvent(-18000, "5 hours to launch")).}
	IF timeToLaunch > 3600  {insertEvent(makeEvent(-3600, "1 hour to launch")).}
	IF timeToLaunch > 1800  {insertEvent(makeEvent(-1800, "30 minutes to launch")).}
	IF timeToLaunch > 600   {insertEvent(makeEvent(-600, "10 minutes to launch")).}
	IF timeToLaunch > 300   {insertEvent(makeEvent(-300, "5 minutes to launch")).}
	IF timeToLaunch > 60    {insertEvent(makeEvent(-60, "1 minute to launch")).}
	IF timeToLaunch > 30	{insertEvent(makeEvent(-30, "30 seconds to launch")).}
	insertEvent(makeEvent(-10, "10 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-9, "9 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-8, "8 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-7, "7 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-6, "6 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-5, "5 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-4, "4 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-3, "3 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-2, "2 SECONDS TO LAUNCH")).
	insertEvent(makeEvent(-1, "1 SECONDS TO LAUNCH")).
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
	//	"SETTINGS" as lexicon
	LOCAL stageActivationTime IS controls["upfgActivation"].
	LOCAL vehicleIterator IS vehicle:ITERATOR.
	//	The first active stage is already pre-staged so we only need to create the staging event
	vehicleIterator:NEXT.
	LOCAL stagingEvent IS LEXICON(
		"time", stageActivationTime,
		"type", "_upfgstage",
		"isHidden", vehicleIterator:VALUE["isVirtualStage"] OR vehicleIterator:VALUE["isSustainer"],
		"fpMessage", "STAGE: " + vehicleIterator:VALUE["name"]
	).
	//	Insert it into sequence
	insertEvent(stagingEvent).
	//	But we also want to demark this exact moment as transition to active guidance mode
	LOCAL stagingEvent IS LEXICON(
		"time", stageActivationTime,
		"type", "_activeon",
		"isHidden", FALSE
	).
	insertEvent(stagingEvent).
	//	Compute burnout time for this stage and add to sAT (this involves activation time and burn time)
	SET stageActivationTime TO stageActivationTime + getStageDelays(vehicleIterator:VALUE) + vehicleIterator:VALUE["maxT"].
	//	Loop over remaining stages
	UNTIL NOT vehicleIterator:NEXT {
		//	Construct & insert pre-stage event
		LOCAL stagingTransitionTime IS SETTINGS["stagingKillRotTime"].
		IF vehicleIterator:VALUE["isVirtualStage"] { SET stagingTransitionTime TO 2. }
		LOCAL stagingEvent IS LEXICON(
			"time", stageActivationTime - stagingTransitionTime,
			"type", "_prestage",
			"isHidden", TRUE
		).
		insertEvent(stagingEvent).
		//	Construct & insert staging event
		LOCAL stagingEvent IS LEXICON(
			"time", stageActivationTime,
			"type", "_upfgstage",
			"isHidden", vehicleIterator:VALUE["isVirtualStage"],
			"fpMessage", "STAGE: " + vehicleIterator:VALUE["name"]
		).
		//	Toggle display and add info to the constant-acceleration stages (they don't have a corresponding event to show instead)
		IF vehicleIterator:VALUE["virtualStageType"] = "virtual (const-acc)" {
			SET stagingEvent["isHidden"] TO FALSE.
			SET stagingEvent["fpMessage"] TO "Constant acceleration mode".
		}
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
	//	"prestageHold" as bool
	//	"poststageHold" as bool
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
	ELSE IF eType = "action" OR eType = "a" {
		userEvent_action(event).
	}
	ELSE IF eType = "_activeon" {
		internalEvent_activeModeOn().
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

//	Handle transition to active guidance mode
FUNCTION internalEvent_activeModeOn {
	SET activeGuidanceMode TO TRUE.
	pushUIMessage("UPFG activated!").
}

//	Handle the pre-staging event
FUNCTION internalEvent_preStage {
	//	Switch to staging mode, increment the stage counter and force UPFG reconvergence.
	//	Rationale is not changed: we want to maintain constant attitude while the current stage is still burning,
	//	but at the same time start converging guidance for the subsequent stage.
	SET stagingInProgress TO TRUE.
	SET prestageHold TO TRUE.
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
	//	First, clear the flag informing of the previous stage end-of-burn attitude hold
	SET prestageHold TO FALSE.
	//	Gather all necessary information
	LOCAL currentTime IS TIME:SECONDS.
	LOCAL event IS vehicle[upfgStage]["staging"].
	LOCAL stageName IS vehicle[upfgStage]["name"].
	LOCAL eventDelay IS 0.	//	Keep track of time between subsequent events.
	//	If this stage needs a postStageEvent, set the hold flag immediately
	IF event["postStageEvent"] {
		SET poststageHold TO TRUE.
	}
	//	Now we can get to work
	IF upfgStage > 0 AND vehicle[upfgStage-1]["shutdownRequired"] {
		SET throttleSetting TO 0.
		SET throttleDisplay TO 0.
	}
	IF event["jettison"] {
		LOCAL stageJettisonTime IS currentTime + event["waitBeforeJettison"].
		WHEN TIME:SECONDS >= stageJettisonTime THEN {
			STAGE.
			pushUIMessage(stageName + " - separation").
		}
		SET eventDelay TO eventDelay + event["waitBeforeJettison"].
	}
	IF event["ignition"] {
		IF event["ullage"] = "rcs" {
			LOCAL ullageIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= ullageIgnitionTime THEN {
				RCS ON.
				SET SHIP:CONTROL:FORE TO 1.0.
				pushUIMessage(stageName + " - RCS ullage on").
			}
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
			LOCAL engineIgnitionTime IS currentTime + eventDelay + event["ullageBurnDuration"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {
				internalEvent_staging_activation().
			}
			SET eventDelay TO eventDelay + event["ullageBurnDuration"].
			LOCAL ullageShutdownTime IS currentTime + eventDelay + event["postUllageBurn"].
			WHEN TIME:SECONDS >= ullageShutdownTime THEN {
				SET SHIP:CONTROL:FORE TO 0.0.
				RCS OFF.
				pushUIMessage(stageName + " - RCS ullage off").
			}
		} ELSE IF event["ullage"] = "srb" {
			LOCAL ullageIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= ullageIgnitionTime THEN {
				STAGE.
				pushUIMessage(stageName + " - SRB ullage ignited").
			}
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
			LOCAL engineIgnitionTime IS currentTime + eventDelay + event["ullageBurnDuration"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {
				internalEvent_staging_activation().
			}
			SET eventDelay TO eventDelay + event["ullageBurnDuration"].
		} ELSE IF event["ullage"] = "none" {
			LOCAL engineIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {
				internalEvent_staging_activation().
			}
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
		} ELSE {
			//	This should no longer be possible as we check it prior to liftoff (setVehicle)
			pushUIMessage( "Unknown ullage mode (" + event["ullage"] + ")!", 5, PRIORITY_HIGH ).
		}
	} ELSE {
		//	If this event does not need ignition, staging is over at this moment
		internalEvent_staging_activation(FALSE, FALSE).
	}
	//	However, we might still need to execute the post-staging event
	IF event["postStageEvent"] {
		LOCAL hasExtraHold IS event:HASKEY("waitAfterPostStage").
		LOCAL postStagingEventTime IS currentTime + eventDelay + event["waitBeforePostStage"].
		WHEN TIME:SECONDS >= postStagingEventTime THEN {
			STAGE.
			IF NOT hasExtraHold {
				SET poststageHold TO FALSE.
			}
			pushUIMessage(stageName + " - post-jettison").
		}
		SET eventDelay TO eventDelay + event["waitBeforePostStage"].
		//	If after the separation we need to wait some extra time before releasing the attitude hold
		IF hasExtraHold {
			LOCAL postStagingEventTime IS currentTime + eventDelay + event["waitAfterPostStage"].
			WHEN TIME:SECONDS >= postStagingEventTime THEN {
				SET poststageHold TO FALSE.
			}
			SET eventDelay TO eventDelay + event["waitAfterPostStage"].
		}
	}

	//	Print messages for regular stages and constant-acceleration mode activation.
	IF NOT vehicle[upfgStage]["isVirtualStage"] {
		pushUIMessage("Staging sequence commencing...").
	} ELSE IF vehicle[upfgStage]["mode"] = 2 {
		pushUIMessage("Constant acceleration mode activated.").
	}
}

//	Subroutine for the staging handler - logical stage activation
FUNCTION internalEvent_staging_activation {
	//	This handles the "start" of a stage - be it an individual physical stage with an engine to be ignited,
	//	or a sustainer type stage that's already on.
	//	Expects global variables:
	//	"vehicle" as list
	//	"upfgStage" as scalar
	//	"stagingInProgress" as boolean
	DECLARE PARAMETER needsIgnite IS TRUE.	//	Expects a boolean
	DECLARE PARAMETER printMessage IS TRUE.	//	Expects a boolean

	//	Set throttle once for the entire burn
	LOCAL desiredThrottle IS vehicle[upfgStage]["throttle"].
	LOCAL throttleLimit IS vehicle[upfgStage]["minThrottle"].
	SET throttleSetting TO (desiredThrottle - throttleLimit) / (1 - throttleLimit).
	SET throttleDisplay TO desiredThrottle.
	//	Ignite if necessary
	IF needsIgnite {
		STAGE.
	}
	//	Technical stuff
	updateStageEndTime().
	SET stagingInProgress TO FALSE.
	//	Print message if requested
	IF printMessage {
		pushUIMessage(vehicle[upfgStage]["name"] + " - ignition").
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
		event:ADD("message", "Shutting down engine(s): '" + event["engineTag"] + "'.").
	}
}

//	Handle the roll event
FUNCTION userEvent_roll {
	DECLARE PARAMETER event.	//	Expects a lexicon

	SET steeringRoll TO event["angle"].
	IF NOT event:HASKEY("message") {
		event:ADD("message", "Rolling to " + steeringRoll + " degrees.").
	}
}

//	Handle the delegate event
FUNCTION userEvent_delegate {
	DECLARE PARAMETER event.	//	Expects a lexicon

	LOCAL fun IS event["function"].
	IF fun:ISDEAD() {
		pushUIMessage("DEAD DELEGATE - UNABLE TO CALL!", 10, PRIORITY_CRITICAL).
	} ELSE {
		fun:CALL().
	}
}

//	Handle the action group event
FUNCTION userEvent_action {
	DECLARE PARAMETER event.	//	Expects a lexicon

	LOCAL action IS event["action"]:TOUPPER.
	IF action = "RCS" {
		TOGGLE RCS.
	}
	ELSE IF action = "LIGHTS" {
		TOGGLE LIGHTS.
	}
	ELSE IF action = "BRAKES" {
		TOGGLE BRAKES.
	}
	ELSE IF action = "GEAR" {
		TOGGLE GEAR.
	}
	ELSE IF action = "AG1" {
		TOGGLE AG1.
	}
	ELSE IF action = "AG2" {
		TOGGLE AG2.
	}
	ELSE IF action = "AG3" {
		TOGGLE AG3.
	}
	ELSE IF action = "AG4" {
		TOGGLE AG4.
	}
	ELSE IF action = "AG5" {
		TOGGLE AG5.
	}
	ELSE IF action = "AG6" {
		TOGGLE AG6.
	}
	ELSE IF action = "AG7" {
		TOGGLE AG7.
	}
	ELSE IF action = "AG8" {
		TOGGLE AG8.
	}
	ELSE IF action = "AG9" {
		TOGGLE AG9.
	}
	ELSE IF action = "AG10" {
		TOGGLE AG10.
	}
	ELSE {
		pushUIMessage("Unsupported action group '" + action + "'!", 10, PRIORITY_HIGH).
	}
}
