//	If no boot file was loaded, this check will immediately crash PEGAS, saving time that would otherwise be wasted on loading libraries.
IF NOT (DEFINED vehicle) OR NOT (DEFINED sequence) OR NOT (DEFINED controls) OR NOT (DEFINED mission) {
	PRINT "".
	PRINT "No boot file loaded! Crashing...".
	PRINT "".
	SET _ TO sequence.
	SET _ TO controls.
	SET _ TO vehicle.
	SET _ TO mission.
}

//	The following is absolutely necessary to run UPFG fast enough.
SET CONFIG:IPU TO 500.

//	Equivalent to clicking "control from here" on a part that runs the system.
//	Helpful when your payload is not perfectly rigidly attached, and you're not sure whether it controls the vessel or not.
CORE:PART:CONTROLFROM().

//	Set up constants.
GLOBAL g0 IS 9.8067.					//	PEGAS will launch from any planet or moon - "g0" is a standard constant for thrust computation and shall not be changed!
GLOBAL pitchOverTimeLimit IS 20.		//	In atmospheric part of ascent, when the vehicle pitches over, the wait for velocity vector to align will be forcibly broken after that many second.
GLOBAL upfgConvergenceDelay IS 5.		//	seconds before "upfgActivation" that we switch into loop 2 to give UPFG time to converge
GLOBAL upfgFinalizationTime IS 5.		//	When time-to-go gets below that, keep attitude stable and simply count down time to cutoff.
GLOBAL stagingKillRotTime IS 5.			//	That many seconds before staging, updating attitude commands will be forbidden, allowing clean separation.
GLOBAL upfgConvergenceCriterion IS 0.1.	//	Maximum difference between consecutive UPFG T-go predictions that allow accepting the solution.
GLOBAL upfgGoodSolutionCriterion IS 15.	//	Maximum angle between guidance vectors calculated by UPFG between stages that allow accepting the solution.

//	Load libraries.
RUN pegas_cser.
RUN pegas_upfg.
RUN pegas_util.
RUN pegas_misc.

//	Initialize global flags
GLOBAL upfgStage IS -1.				//	Seems wrong (we use "vehicle[upfgStage]") but first run of stageEventHandler increments this automatically
GLOBAL stageEventFlag IS FALSE.
GLOBAL systemEvents IS LIST().
GLOBAL systemEventPointer IS -1.	//	Same deal as with "upfgStage"
GLOBAL systemEventFlag IS FALSE.
GLOBAL userEventPointer IS -1.		//	As above
GLOBAL userEventFlag IS FALSE.
GLOBAL throttleSetting IS 1.		//	This is what actually controls the throttle,
GLOBAL throttleDisplay IS 1.		//	and this is what to display on the GUI - see throttleControl() for details.
GLOBAL steeringVector IS LOOKDIRUP(SHIP:FACING:FOREVECTOR, SHIP:FACING:TOPVECTOR).
GLOBAL steeringRoll IS 0.
GLOBAL upfgConverged IS FALSE.
GLOBAL stagingInProgress IS FALSE.


//	PREFLIGHT ACTIVITIES
//	Update mission struct and set up UPFG target
missionSetup().
SET upfgTarget TO targetSetup().
//	Calculate time to launch
SET currentTime TO TIME.
SET timeToOrbitIntercept TO orbitInterceptTime().
GLOBAL liftoffTime IS currentTime + timeToOrbitIntercept - controls["launchTimeAdvance"].
IF timeToOrbitIntercept < controls["launchTimeAdvance"] { SET liftoffTime TO liftoffTime + SHIP:BODY:ROTATIONPERIOD. }
//	Calculate launch azimuth if not specified
IF NOT mission:HASKEY("launchAzimuth") {
	mission:ADD("launchAzimuth", launchAzimuth()).
}
//	Read initial roll angle (to be executed during the pitchover maneuver)
IF controls:HASKEY("initialRoll") {
	SET steeringRoll TO controls["initialRoll"].
}
//	Set up the system for flight
setSystemEvents().		//	Set up countdown messages
setUserEvents().		//	Initialize vehicle sequence
setVehicle().			//	Complete vehicle definition (as given by user)


//	PEGAS TAKES CONTROL OF THE MISSION
createUI().
//	Prepare control for vertical ascent
LOCK THROTTLE TO throttleSetting.
LOCK STEERING TO steeringVector.
SET ascentFlag TO 0.	//	0 = vertical, 1 = pitching over, 2 = notify about holding prograde, 3 = just hold prograde
//	Main loop - wait on launch pad, lift-off and passive guidance
UNTIL ABORT {
	//	Sequence handling
	IF systemEventFlag = TRUE { systemEventHandler(). }
	IF   userEventFlag = TRUE {   userEventHandler(). }
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
		//	We cannot blindly hold prograde though, because this will provide no azimuth control
		//	Much better option is to read current velocity angle and aim for that, but correct for azimuth
		SET velocityAngle TO 90-VANG(SHIP:UP:VECTOR, SHIP:VELOCITY:SURFACE).
		SET steeringVector TO aimAndRoll(HEADING(mission["launchAzimuth"],velocityAngle):VECTOR, steeringRoll).
		//	There are two almost identical cases, in the first we set the initial message, in the next we just keep attitude.
		pushUIMessage( "Holding prograde at " + ROUND(mission["launchAzimuth"],1) + " deg azimuth." ).
		SET ascentFlag TO 3.
	}
	ELSE {
		SET velocityAngle TO 90-VANG(SHIP:UP:VECTOR, SHIP:VELOCITY:SURFACE).
		SET steeringVector TO aimAndRoll(HEADING(mission["launchAzimuth"],velocityAngle):VECTOR, steeringRoll).
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
	WAIT 0.
}


//	ACTIVE GUIDANCE
createUI().
//	Initialize UPFG
initializeVehicle().
SET upfgState TO acquireState().
SET upfgInternal TO setupUPFG().
//	Main loop - iterate UPFG (respective function controls attitude directly)
UNTIL ABORT {
	//	Sequence handling
	IF systemEventFlag = TRUE { systemEventHandler(). }
	IF   userEventFlag = TRUE {   userEventHandler(). }
	IF  stageEventFlag = TRUE {  stageEventHandler(). }
	//	Update UPFG target and vehicle state
	SET upfgTarget["normal"] TO targetNormal(mission["inclination"], mission["LAN"]).
	SET upfgState TO acquireState().
	//	Iterate UPFG and preserve its state
	SET upfgInternal TO upfgSteeringControl(vehicle, upfgStage, upfgTarget, upfgState, upfgInternal).
	//	Manage throttle, with the exception of initial portion of guided flight (where we're technically still flying the first stage).
	IF upfgStage >= 0 { throttleControl(). }
	//	For the final seconds of the flight, just hold attitude and wait.
	IF upfgConverged AND upfgInternal["tgo"] < upfgFinalizationTime { BREAK. }
	//	UI
	refreshUI().
	WAIT 0.
}
//	Final orbital insertion loop
pushUIMessage( "Holding attitude for burn finalization!" ).
SET previousTime TO TIME:SECONDS.
UNTIL ABORT {
	LOCAL finalizeDT IS TIME:SECONDS - previousTime.
	SET previousTime TO TIME:SECONDS.
	SET upfgInternal["tgo"] TO upfgInternal["tgo"] - finalizeDT.
	IF upfgInternal["tgo"] < finalizeDT { BREAK. }	//	Exit loop before entering the next refresh cycle
													//	We could have done "tgo < 0" but this would mean that the previous loop tgo was 0.01 yet we still didn't break
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