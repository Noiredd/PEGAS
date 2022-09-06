GLOBAL _PEGAS_VERSION_ IS "v1.3-alpha".

//	Check if all necessary variables have been defined, exit early otherwise.
RUN pegas_precheck.

//	Load settings and libraries.
RUN pegas_settings.
IF cserVersion = "new" {
	RUN pegas_cser_new.
} ELSE {
	RUN pegas_cser.
}
RUN pegas_events.
RUN pegas_upfg.
RUN pegas_util.
RUN pegas_misc.
RUN pegas_comm.
RUN pegas_addons.

//	The following is absolutely necessary to run UPFG fast enough.
SET CONFIG:IPU TO kOS_IPU.

//	Initialize global flags and constants
GLOBAL upfgStage IS -1.				//	System initializes at passive guidance
GLOBAL stageEndTime IS TIME.		//	For Tgo calculation during active guidance (global time)
GLOBAL eventPointer IS -1.			//	Index of the last executed event (-1 means none yet)
GLOBAL throttleSetting IS 1.		//	This is what actually controls the throttle,
GLOBAL throttleDisplay IS 1.		//	and this is what to display on the GUI - see throttleControl() for details.
GLOBAL steeringVector IS LOOKDIRUP(SHIP:FACING:FOREVECTOR, SHIP:FACING:TOPVECTOR).
GLOBAL steeringRoll IS 0.
GLOBAL activeGuidanceMode IS FALSE.	//	Set to TRUE at UPFG activation time (as defined in controls)
GLOBAL upfgConverged IS FALSE.		//	See upfgSteeringControl comments
GLOBAL upfgEngaged IS FALSE.		//	See upfgSteeringControl comments
GLOBAL stagingInProgress IS FALSE.	//	See upfgSteeringControl comments
GLOBAL prestageHold IS FALSE.		//	See upfgSteeringControl comments

//	Load user addons
scanAddons().


//	PREFLIGHT ACTIVITIES
//	Click "control from here" on a part that runs the system.
//	Helpful when your payload is not perfectly rigidly attached, and you're not sure whether it controls the vessel or not.
CORE:PART:CONTROLFROM().
//	Update mission struct and set up UPFG target
missionSetup().
SET upfgTarget TO targetSetup().
//	Calculate time to launch
SET currentTime TO TIME.
SET timeToOrbitIntercept TO orbitInterceptTime().
GLOBAL liftoffTime IS currentTime + timeToOrbitIntercept - controls["launchTimeAdvance"].
IF timeToOrbitIntercept < controls["launchTimeAdvance"] {
	SET liftoffTime TO liftoffTime + SHIP:BODY:ROTATIONPERIOD.
}
//	Calculate launch azimuth if not specified
IF NOT mission:HASKEY("launchAzimuth") {
	mission:ADD("launchAzimuth", launchAzimuth()).
}
//	Read initial roll angle (to be executed during the pitchover maneuver)
IF controls:HASKEY("initialRoll") {
	SET steeringRoll TO controls["initialRoll"].
}
//	Set up the system for flight
setVehicle().			//	Complete vehicle definition (as given by user)
spawnCountdownEvents().
buildFlightPlan(TRUE).	//	Generate the printable events before drawing the UI
callHooks("init").		//	System initialized, run hooks


//	PEGAS TAKES CONTROL OF THE MISSION
createUI().
//	Prepare control for vertical ascent
LOCK THROTTLE TO throttleSetting.
LOCK STEERING TO steeringVector.
SET ascentFlag TO 0.	//	0 = vertical, 1 = pitching over, 2 = notify about holding prograde, 3 = just hold prograde
//	Main loop - wait on launch pad, lift-off and passive guidance
UNTIL ABORT {
	//	User hooks
	callHooks("passivePre").
	//	Event handling
	eventHandler().
	//	Communication system handling
	commsHandler().
	//	Control handling
	IF ascentFlag = 0 {
		//	The vehicle is going straight up for given amount of time
		IF TIME:SECONDS >= liftoffTime:SECONDS + controls["verticalAscentTime"] {
			//	Then it changes attitude for an initial pitchover "kick"
			SET steeringVector TO aimAndRoll(HEADING(mission["launchAzimuth"],90-controls["pitchOverAngle"]):VECTOR, steeringRoll).
			SET ascentFlag TO 1.
			pushUIMessage( "Pitching over by " + ROUND(controls["pitchOverAngle"],1) + " degrees." ).
		}
	}
	ELSE IF ascentFlag = 1 {
		//	It keeps this attitude until velocity vector matches it closely
		IF TIME:SECONDS < liftoffTime:SECONDS + controls["verticalAscentTime"] + 3 {
			//	Delay this check for the first few seconds to allow the vehicle to pitch away from current prograde
		} ELSE {
			//	Attitude must be recalculated at every iteration though
			SET velocityAngle TO VANG(SHIP:UP:VECTOR, SHIP:VELOCITY:SURFACE).
			IF controls["pitchOverAngle"] - velocityAngle < 0.1 {
				SET ascentFlag TO 2.
			}
		}
		//	As a safety check - do not stay deadlocked in this state for too long (might be unnecessary).
		IF TIME:SECONDS >= liftoffTime:SECONDS + controls["verticalAscentTime"] + pitchOverTimeLimit {
			SET ascentFlag TO 2.
			pushUIMessage( "Pitchover time limit exceeded!", 5, PRIORITY_HIGH ).
		}
	}
	ELSE IF ascentFlag = 2 {
		//	Enter the minimal angle of attack phase. This case is different only in that we push a transition message.
		SET steeringVector TO minAoASteering(steeringRoll).
		pushUIMessage( "Holding prograde at " + ROUND(mission["launchAzimuth"],1) + " deg azimuth." ).
		SET ascentFlag TO 3.
	}
	ELSE {
		//	Maintain minimal AoA trajectory
		SET steeringVector TO minAoASteering(steeringRoll).
	}
	//	The passive guidance loop ends a few seconds before actual ignition of the first UPFG-controlled stage.
	//	This is to give UPFG time to converge. Actual ignition occurs via stagingEvents.
	IF TIME:SECONDS >= liftoffTime:SECONDS + controls["upfgActivation"] - upfgConvergenceDelay {
		pushUIMessage( "Initiating UPFG!" ).
		BREAK.
	}
	//	UI - recalculate UPFG target solely for printing relative angle
	SET upfgTarget["normal"] TO targetNormal(mission["inclination"], mission["LAN"]).
	refreshUI().
	//	User hooks
	callHooks("passivePost").
	WAIT 0.
}


//	ACTIVE GUIDANCE
createUI().
//	Initialize UPFG and all the structures it requires
initializeVehicleForUPFG().
SET upfgState TO acquireState().
SET upfgInternal TO setupUPFG().
//	Reassemble the flight plan after vehicle initialization
buildFlightPlan(FALSE).
//	Call user hooks
callHooks("activeInit").
//	Main loop - iterate UPFG (respective function controls attitude directly)
UNTIL ABORT {
	//	User hooks
	callHooks("activePre").
	//	Event handling
	eventHandler().
	//	Communication system handling
	commsHandler().
	//	Update UPFG target and vehicle state
	SET upfgTarget["normal"] TO targetNormal(mission["inclination"], mission["LAN"]).
	SET upfgState TO acquireState().
	//	Iterate UPFG and preserve its state
	SET upfgInternal TO upfgSteeringControl(vehicle, upfgStage, upfgTarget, upfgState, upfgInternal).
	//	Manage throttle, with the exception of initial portion of guided flight (where we're technically still flying the first stage).
	IF activeGuidanceMode { throttleControl(). }
	//	Transition to the attitude hold mode for the final seconds of the flight
	IF upfgConverged AND upfgInternal["tgo"] < upfgFinalizationTime { BREAK. }
	//	UI
	refreshUI().
	//	User hooks
	callHooks("activePost").
	WAIT 0.
}
//	Final orbital insertion loop
pushUIMessage( "Holding attitude for burn finalization!" ).
SET previousTime TO TIME:SECONDS.
UNTIL ABORT {
	LOCAL finalizeDT IS TIME:SECONDS - previousTime.
	SET previousTime TO TIME:SECONDS.
	SET upfgInternal["tgo"] TO upfgInternal["tgo"] - finalizeDT.
	//	Exit the loop before entering the next refresh cycle.
	//	We can't do "tgo < 0" as then we still didn't break if the previous loop tgo was 0.01 or so.
	IF upfgInternal["tgo"] < finalizeDT { BREAK. }
	refreshUI().
	WAIT 0.
}


//	EXIT
UNLOCK STEERING.
UNLOCK THROTTLE.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
WAIT 0.
missionValidation().
refreshUI().
//	Execute the final hooks
callHooks("final").
