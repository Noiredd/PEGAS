//	Utility library.

//	INTERNAL FUNCTIONS

//	Rodrigues vector rotation formula
FUNCTION rodrigues {
	DECLARE PARAMETER inVector.	//	Expects a vector
	DECLARE PARAMETER axis.		//	Expects a vector
	DECLARE PARAMETER angle.	//	Expects a scalar
	
	SET axis TO axis:NORMALIZED.
	
	LOCAL outVector IS inVector*COS(angle).
	SET outVector TO outVector + VCRS(axis, inVector)*SIN(angle).
	SET outVector TO outVector + axis*VDOT(axis, inVector)*(1-COS(angle)).
	
	RETURN outVector.
}.

//	KSP-MATLAB-KSP vector conversion
FUNCTION vecYZ {
	DECLARE PARAMETER input.	//	input vector
	LOCAL output IS V(input:X, input:Z, input:Y).
	RETURN output.
}.

//	Engine combination parameters
FUNCTION getThrust {
	DECLARE PARAMETER engines.	//	Expects a list of lexicons
	
	LOCAL n IS engines:LENGTH.
	LOCAL F IS 0.
	LOCAL dm IS 0.
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		LOCAL isp IS engines[i]["isp"].
		LOCAL dm_ IS engines[i]["flow"].
		SET dm TO dm + dm_.
		SET F TO F + isp*dm_*g0.
	}
	SET isp TO F/(dm*g0).
	
	RETURN LIST(F, dm, isp).
}.

//	TARGETING FUNCTIONS

//	Generate a PEGAS-compatible target struct from user-specified one
FUNCTION targetSetup {
	//	Expects a global variable "mission" as lexicon
	
	//	Fix target definition if the burnout altitude is wrong or not given
	IF mission:HASKEY("altitude") {
		IF mission["altitude"] < mission["periapsis"] OR mission["altitude"] > mission["apoapsis"] {
			SET mission["altitude"] TO mission["periapsis"].
		}
	} ELSE {
		mission:ADD("altitude", mission["periapsis"]).
	}
	
	//	Fix LAN to between 0-360 degrees
	IF mission["LAN"] < 0 { SET mission["LAN"] TO mission["LAN"] + 360. }
	
	//	Override plane definition if a map target was selected
	IF HASTARGET {
		SET mission["inclination"] TO TARGET:ORBIT:INCLINATION.
		SET mission["LAN"] TO TARGET:ORBIT:LAN.
	}
	
	//	Calculate velocity and flight path angle at given criterion using vis-viva equation and conservation of specific relative angular momentum
	LOCAL pe IS mission["periapsis"]*1000 + SHIP:BODY:RADIUS.
	LOCAL ap IS mission["apoapsis"]*1000 + SHIP:BODY:RADIUS.
	LOCAL targetAltitude IS mission["altitude"]*1000 + SHIP:BODY:RADIUS.
	LOCAL sma IS (pe+ap) / 2.							//	semi-major axis
	LOCAL vpe IS SQRT(SHIP:BODY:MU * (2/pe - 1/sma)).	//	velocity at periapsis
	LOCAL srm IS pe * vpe.								//	specific relative angular momentum
	LOCAL targetVelocity IS SQRT(SHIP:BODY:MU * (2/targetAltitude - 1/sma)).
	LOCAL flightPathAngle IS ARCCOS( srm/(targetVelocity*targetAltitude) ).
	
	RETURN LEXICON(
				"radius", targetAltitude,
				"velocity", targetVelocity,
				"angle", flightPathAngle,
				"normal", V(0,0,0)				//	temporarily unset - due to KSP's silly coordinate system this needs to be recalculated every time step, so we will not bother with it for now
				).
}.

//	Time to next northerly launch opportunity
FUNCTION orbitInterceptTime {
	//	Expects a global variable "mission" as lexicon
	LOCAL targetInc IS mission["inclination"].
	LOCAL targetLan IS mission["lan"].
	
	//	First find the ascending node of an orbit of the given inclination, passing right over the vehicle now.
	LOCAL b IS ARCSIN(TAN(90-targetInc)*(TAN(SHIP:GEOPOSITION:LAT))).			//	From Napier's spherical triangle mnemonics
	LOCAL currentNode IS VXCL(V(0,1,0), -SHIP:ORBIT:BODY:POSITION):NORMALIZED.
	SET currentNode TO rodrigues(currentNode, V(0,1,0), b).
	//	Then find the ascending node of the target orbit.
	LOCAL targetNode IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -targetLan).
	//	Finally find the angle between them, minding rotation direction.
	LOCAL nodeDelta IS VANG(currentNode, targetNode).
	LOCAL deltaDir IS VDOT(V(0,1,0), VCRS(targetNode, currentNode)).
	IF deltaDir < 0 { SET nodeDelta TO 360 - nodeDelta. }
	LOCAL deltaTime IS SHIP:ORBIT:BODY:ROTATIONPERIOD * nodeDelta/360.
	
	RETURN deltaTime.
}.

//	Launch azimuth to a given orbit
FUNCTION launchAzimuth {
	//	Expects global variables "upfgTarget" and "mission" as lexicons
	
	LOCAL targetInc IS mission["inclination"].
	LOCAL targetAlt IS upfgTarget["radius"].
	LOCAL targetVel IS upfgTarget["velocity"].
	LOCAL siteLat IS SHIP:GEOPOSITION:LAT.
	IF targetInc < siteLat { pushUIMessage( "Target inclination below launch site latitude!" ). }
	
	LOCAL Binertial IS ARCSIN( COS(targetInc)/COS(siteLat) ).
	//LOCAL Vorbit IS SQRT( SHIP:ORBIT:BODY:MU/(SHIP:BODY:RADIUS+targetAlt*1000) ).		//	This is a normal calculation for a circular orbit
	LOCAL Vorbit IS targetVel*COS(upfgTarget["angle"]).									//	But we already have our desired velocity, however we must correct for the flight path angle (only the tangential component matters here)
	LOCAL Vbody IS (2*CONSTANT:PI*SHIP:BODY:RADIUS/SHIP:BODY:ROTATIONPERIOD)*COS(siteLat).
	LOCAL VrotX IS Vorbit*SIN(Binertial)-Vbody.
	LOCAL VrotY IS Vorbit*COS(Binertial).
	LOCAL azimuth IS ARCTAN2(VrotY, VrotX).
	
	RETURN 90-azimuth.	//	In MATLAB an azimuth of 0 is due east, while in KSP it's due north. This returned value is steering-ready.
}.

//	UPFG HANDLING FUNCTIONS

//	Creates and initializes UPFG internal struct
FUNCTION setupUPFG {
	//	Expects global variables "mission", "upfgState" and upfgTarget as lexicons

	LOCAL curR IS upfgState["radius"].
	LOCAL curV IS upfgState["velocity"].

	SET upfgTarget["normal"] TO targetNormal(mission["inclination"], mission["LAN"]).
	LOCAL desR IS rodrigues(curR, -upfgTarget["normal"], 20):NORMALIZED * upfgTarget["radius"].
	LOCAL tgoV IS upfgTarget["velocity"] * VCRS(-upfgTarget["normal"], desR):NORMALIZED - curV.

	//vecdraw(v(0,0,0),vecyz(curR):normalized,rgb(1,0,0),"R",10.0,true,0.1).
	//vecdraw(v(0,0,0),vecyz(desR):normalized,rgb(1,1,0),"dR",10.0,true,0.1).
	//vecdraw(v(0,0,0),VCRS(-upfgTarget["normal"], desR):NORMALIZED,rgb(1,0,1),"dV",10.0,true,0.1).
	//vecdraw(v(0,0,0),vecyz(tgoV):normalized,rgb(0,1,1),"Vgo",10.0,true,0.1).
	//vecdraw(v(0,0,0),vecyz(upfgTarget["normal"]:normalized),rgb(1,1,1),"iy",10.0,true,0.1).
	RETURN LEXICON(
		"cser", LEXICON("dtcp",0, "xcp",0, "A",0, "D",0, "E",0),
		"rbias", V(0, 0, 0),
		"rd", desR,
		"rgrav", -SHIP:ORBIT:BODY:MU/2 * curR / curR:MAG^3,
		"tb", 0,
		"time", upfgState["time"],
		"tgo", 0,
		"v", curV,
		"vgo", tgoV
	).
}.

//	Acquire vehicle position data
FUNCTION acquireState {
	//	Expects a global variable "liftoffTime" as scalar
	
	RETURN LEXICON(
		"time", TIME:SECONDS - liftoffTime:SECONDS,
		"mass", SHIP:MASS*1000,
		"radius", vecYZ(SHIP:ORBIT:BODY:POSITION) * -1,
		"velocity", vecYZ(SHIP:ORBIT:VELOCITY:ORBIT)
	).
}.

//	Target plane normal vector in MATLAB coordinates, UPFG compatible direction
FUNCTION targetNormal {
	DECLARE PARAMETER targetInc.	//	Expects a scalar
	DECLARE PARAMETER targetLan.	//	Expects a scalar
	
	//	First create a vector pointing to the highest point in orbit by rotating the prime vector by a right angle.
	LOCAL highPoint IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), 90-targetLan).
	//	Then create a temporary axis of rotation (short form for 90 deg rotation).
	LOCAL rotAxis IS V(-highPoint:Z, highPoint:Y, highPoint:X).
	//	Finally rotate about this axis by a right angle to produce normal vector.
	LOCAL normalVec IS rodrigues(highPoint, rotAxis, 90-targetInc).
	
	RETURN -vecYZ(normalVec).
}.

//	EVENT HANDLING FUNCTIONS

//	Setup system events, currently only countdown messages
FUNCTION setSystemEvents {
	//	Local function - countdown event generator
	FUNCTION makeEvent {
		DECLARE PARAMETER timeAfterLiftoff.	//	Expects a scalar
		DECLARE PARAMETER eventMessage.		//	Expects a string
		
		RETURN LEXICON("time", timeAfterLiftoff, "type", "dummy", "message", eventMessage, "data", LIST()).
	}.
	
	//	Expects a global variable "liftoffTime" as scalar and "systemEvents" as list
	LOCAL timeToLaunch IS liftoffTime:SECONDS - TIME:SECONDS.
	
	//	Prepare events table
	IF timeToLaunch > 18000 { systemEvents:ADD(makeEvent(-18000,"5 hours to launch")). }
	IF timeToLaunch > 3600  { systemEvents:ADD(makeEvent(-3600,"1 hour to launch")). }
	IF timeToLaunch > 1800  { systemEvents:ADD(makeEvent(-1800,"30 minutes to launch")). }
	IF timeToLaunch > 600   { systemEvents:ADD(makeEvent(-600,"10 minutes to launch")). }
	IF timeToLaunch > 300   { systemEvents:ADD(makeEvent(-300,"5 minutes to launch")). }
	IF timeToLaunch > 60    { systemEvents:ADD(makeEvent(-60,"1 minute to launch")). }
	IF timeToLaunch > 30	{ systemEvents:ADD(makeEvent(-30,"30 seconds to launch")). }
	systemEvents:ADD(makeEvent(-10,"10 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-9,"9 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-8,"8 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-7,"7 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-6,"6 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-5,"5 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-4,"4 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-3,"3 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-2,"2 SECONDS TO LAUNCH")).
	systemEvents:ADD(makeEvent(-1,"1 SECONDS TO LAUNCH")).
	
	//	Initialize the first event
	systemEventHandler().
}.

//	Setup user events (vehicle sequence)
FUNCTION setUserEvents {
	//	Just a wrapper to a handler which automatically does the setup on its first run.
	userEventHandler().
}.

//	Setup vehicle: UPFG info struct and default throttle
//	* sets up MODE for each stage (required for UPFG)
//	* calculates engine fuel mass flow if thrust value was given instead
//	* adjusts maxT/engine flow for given throttle OR sets up default throttle to 1.0
//	* creates a new virtual stage for acceleration-limited mode
//	* initializes the UPFG staging sequence
FUNCTION setVehicle {
	//	Expects a global variable "vehicle" as list of lexicons and "controls" as lexicon
	
	FROM { LOCAL i IS 0. } UNTIL i = vehicle:LENGTH STEP { SET i TO i+1. } DO {
		//	Throttle setup
		IF NOT vehicle[i]:HASKEY("throttle") {
			//	Add entry for consistency
			vehicle[i]:ADD("throttle", 1.0).
		}
		ELSE {
			//	Adjust max burn time and engine flow rate; calculate mass flow if necessary
			SET vehicle[i]["maxT"] TO vehicle[i]["maxT"] / vehicle[i]["throttle"].
			FOR e IN vehicle[i]["engines"] {
				IF NOT e:HASKEY("flow") { e:ADD("flow", e["thrust"] / (e["isp"]*g0)). }
				SET e["flow"] TO e["flow"] * vehicle[i]["throttle"].
			}
		}
		//	Mode setup
		vehicle[i]:ADD("mode", 1).
		//	Acceleration limit setup
		IF vehicle[i]:HASKEY("gLim") {
			IF vehicle[i]["gLim"]>0 {	// due to KSP's lazy condition checking this should not throw errors
				//	Calculate when will the acceleration limit be exceeded
				LOCAL fdmisp IS getThrust(vehicle[i]["engines"]).	//	needs pegas_upfg
				LOCAL Fthrust IS fdmisp[0].
				LOCAL massFlow IS fdmisp[1].
				LOCAL totalIsp IS fdmisp[2].
				LOCAL accLimTime IS (vehicle[i]["m0"] - Fthrust/vehicle[i]["gLim"]/g0) / massFlow.
				//	If this time is greater than the stage's max burn time - we're good. Otherwise, the limit must be enforced
				IF accLimTime < vehicle[i]["maxT"] {
					//	Create a new stage
					LOCAL gLimStage IS LEXICON("mode", 2, "gLim", vehicle[i]["gLim"], "engines", vehicle[i]["engines"]).
					//	Inherit default throttle from the original stage
					gLimStage:ADD("throttle", vehicle[i]["throttle"]).
					//	Supply it with a staging information
					gLimStage:ADD("staging", LEXICON("message", "Constant acceleration mode", "jettison", FALSE, "ignition", FALSE)).
					//	Calculate its initial mass
					LOCAL burnedFuelMass IS massFlow * accLimTime.
					gLimStage:ADD("m0", vehicle[i]["m0"] - burnedFuelMass).
					//	Calculate its burn time assuming constant acceleration
					LOCAL totalStageFuel IS massFlow * vehicle[i]["maxT"].
					LOCAL remainingFuel IS totalStageFuel - burnedFuelMass.
					gLimStage:ADD("maxT", totalIsp/vehicle[i]["gLim"] * LN( gLimStage["m0"]/(gLimStage["m0"]-remainingFuel) )).
					//	Insert it into the list and increment i so that we don't process it next
					vehicle:INSERT(i+1, gLimStage).
					//	Adjust the current stage's burn time
					SET vehicle[i]["maxT"] TO accLimTime.
					SET vehicle[i]["gLim"] TO 0.
					SET i TO i+1.
				}
			}
		}
		ELSE { vehicle[i]:ADD("gLim", 0). }	//	UPFG requires this field
	}
	stageEventHandler(TRUE).	//	Schedule ignition of the first UPFG-controlled stage.
	
	IF NOT controls:HASKEY("initialThrottle") { controls:ADD("initialThrottle", 1.0). }
}.

//	Executes a system event. Currently only supports message printing.
FUNCTION systemEventHandler {
	//	Local function needed here, so we can safely exit the handler on first run without excessive nesting
	FUNCTION setNextEvent {
		SET systemEventPointer TO systemEventPointer + 1.
		IF systemEventPointer < systemEvents:LENGTH {
			WHEN TIME:SECONDS >= liftoffTime:SECONDS + systemEvents[systemEventPointer]["time"] THEN { SET systemEventFlag TO TRUE. }
		}
	}.
	
	//	Expects global variables "liftoffTime" as TimeSpan, "systemEvents" as list, "systemEventFlag" as bool and "systemEventPointer" as scalar.
	//	First call initializes and exits without messaging
	IF systemEventPointer = -1 {	//	This var is initialized at -1, so meeting this condition is only possible on first run.
		setNextEvent().
		RETURN.
	}
	
	//	Handle event
	pushUIMessage( systemEvents[systemEventPointer]["message"], 3 ).
	
	//	Reset event flag
	SET systemEventFlag TO FALSE.
	
	//	Create new event
	setNextEvent().
}.

//	Executes a user (sequence) event.
FUNCTION userEventHandler {
	//	Mechanism is very similar to systemEventHandler
	FUNCTION setNextEvent {
		SET userEventPointer TO userEventPointer + 1.
		IF userEventPointer < sequence:LENGTH {
			WHEN TIME:SECONDS >= liftoffTime:SECONDS + sequence[userEventPointer]["time"] THEN { SET userEventFlag TO TRUE. }
		}
	}.
	
	//	Expects global variables "liftoffTime" as scalar, "sequence" as list, "userEventFlag" as bool and "userEventPointer" as scalar.
	//	First call initializes and exits without doing anything
	IF userEventPointer = -1 {
		setNextEvent().
		RETURN.
	}
	
	//	Handle event
	LOCAL eType IS sequence[userEventPointer]["type"].
	IF      eType = "print" OR eType = "p" { }
	ELSE IF eType = "stage" OR eType = "s" { STAGE. }
	ELSE IF eType = "throttle" OR eType = "t" { SET throttleSetting TO sequence[userEventPointer]["throttle"]. }
	ELSE { pushUIMessage( "Unknown event type (" + eType + ")!" ). }
	pushUIMessage( sequence[userEventPointer]["message"] ).
	
	//	Reset event flag
	SET userEventFlag TO FALSE.
	
	//	Create new event
	setNextEvent().
}.

//	Executes an automatic staging event. Might spawn additional triggers.
FUNCTION stageEventHandler {
	//	Structure is very similar to systemEventHandler, but the pointer only gets incremented when the event is executed.
	//	Reason for this is, i-th event should activate i-th stage, not i+1-th.
	FUNCTION setNextEvent {
		DECLARE PARAMETER eventDelay IS 0.	//	Expects a scalar. Meaning: if this stage ignites in "eventDelay" seconds from now, the next should ignite in "eventDelay"+"maxT" from now.
		IF upfgStage < vehicle:LENGTH {
			GLOBAL nextStageTime IS TIME:SECONDS + eventDelay + vehicle[upfgStage]["maxT"].
			WHEN TIME:SECONDS >= nextStageTime THEN { SET stageEventFlag TO TRUE. }
		}
	}.
	
	//	Expects global variables "liftOffTime" as scalar, "vehicle" as list, "controls" as lexicon, "stageEventFlag" and "upfgConverged" as bool.
	//	Additionally expects a global variable "upfgStage" as scalar but creates it during the first run.
	DECLARE PARAMETER firstRun IS FALSE.	//	Only run with "TRUE" from setup
	
	//	First call initializes and exits
	IF firstRun {
		GLOBAL upfgStage IS -1.		//	We need that at zero but setNextEvent increments this automatically
		GLOBAL stageEventFlag IS FALSE.
		//	First event cannot be initialized with setNextEvent because it directly reads vehicle[upfgStage] which is -1 at first.
		GLOBAL nextStageTime IS liftOffTime:SECONDS + controls["upfgActivation"].
		WHEN TIME:SECONDS >= nextStageTime THEN { SET stageEventFlag TO TRUE. }
		RETURN.
	}
	
	//	Handle event
	SET upfgStage TO upfgStage + 1.	//	Advance UPFG struct pointer
	SET upfgConverged TO FALSE.		//	UPFG might glitch slightly during staging, so mark it as not ready to guide.
	LOCAL event IS vehicle[upfgStage]["staging"].
	LOCAL currentTime IS TIME:SECONDS.
	LOCAL eventDelay IS 0.			//	Many things occur sequentially - this keeps track of the time between subsequent events.
	IF event["jettison"] {
		GLOBAL stageJettisonTime IS currentTime + event["waitBeforeJettison"].
		WHEN TIME:SECONDS >= stageJettisonTime THEN { STAGE. }
		SET eventDelay TO eventDelay + event["waitBeforeJettison"].
	}
	IF event["ignition"] {
		IF event["ullage"] = "rcs" {
			GLOBAL ullageIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= ullageIgnitionTime THEN { RCS ON. SET SHIP:CONTROL:FORE TO 1.0. }
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["ullageBurnDuration"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN { STAGE. }
			SET eventDelay TO eventDelay + event["ullageBurnDuration"].
			GLOBAL ullageShutdownTime IS currentTime + eventDelay + event["postUllageBurn"].
			WHEN TIME:SECONDS >= ullageShutdownTime THEN { SET SHIP:CONTROL:FORE TO 0.0. RCS OFF. }
		} ELSE IF event["ullage"] = "srb" {
			GLOBAL ullageIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= ullageIgnitionTime THEN { STAGE. }
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["ullageBurnDuration"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN { STAGE. }
			SET eventDelay TO eventDelay + event["ullageBurnDuration"].
		} ELSE IF event["ullage"] = "none" {
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN { STAGE. }
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
		} ELSE { pushUIMessage( "Unknown event type (" + event["ullage"] + ")!" ). }
	}
	pushUIMessage( event["message"] ).
	
	//	Reset event flag
	SET stageEventFlag TO FALSE.
	
	//	Create new event
	setNextEvent(eventDelay).
}.

//	OTHER TECHNICAL FUNCTIONS

//	Decides whether UPFG has converged basing on variations in guidance vector
FUNCTION convergenceHandler {
	DECLARE PARAMETER upfgResults.	//	Expects a lexicon
	
	//	Expects a global variables "upfgConverged" as bool and a global variable "previousIF" as vector which it creates if undefined.
	
	IF NOT (DEFINED previousIF) {
		GLOBAL previousIF IS V(1,0,0).
	}
	
	LOCAL vectorDifference IS (previousIF-upfgResults["vector"]):MAG.
	IF upfgConverged = FALSE AND vectorDifference<0.01 {
		SET upfgConverged TO TRUE.
		pushUIMessage( "UPFG has converged!" ).
	}
	SET previousIF TO upfgResults["vector"].
}.

//	Throttle controller
FUNCTION throttleControl {
	//	Expects global variables "vehicle" as list and "upfgStage" as scalar.
	//	Also expects a global variable "throttleSetting" as scalar but creates it on its first run.
	
	IF NOT (DEFINED throttleSetting) {
		GLOBAL throttleSetting IS 0.
		LOCK THROTTLE TO throttleSetting.
	}
	
	LOCAL thisStage IS MAX(upfgStage, 0).	//	At first, when loop 2 has only just kicked in, upfgStage is at -1.
	IF vehicle[thisStage]["mode"] = 1 {
		SET throttleSetting TO vehicle[thisStage]["throttle"].
	}
	ELSE IF vehicle[thisStage]["mode"] = 2 {
		LOCAL currentThrust_ IS getThrust(vehicle[thisStage]["engines"]).	//	requires pegas_upfg
		LOCAL currentThrust IS currentThrust_[0].
		SET throttleSetting TO vehicle[thisStage]["throttle"]*(SHIP:MASS*1000*vehicle[thisStage]["gLim"]*g0) / (currentThrust).
	}
	ELSE { pushUIMessage( "throttleControl stage error (stage=" + thisStage + ", mode=" + vehicle[thisStage]["mode"] + ")!" ). }.
}.
