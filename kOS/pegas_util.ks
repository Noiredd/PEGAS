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
}

//	Returns a kOS direction for given aim vector and roll angle
FUNCTION aimAndRoll {
	DECLARE PARAMETER aimVec.	//	Expects a vector
	DECLARE PARAMETER rollAng.	//	Expects a scalar
	
	LOCAL rollVector IS rodrigues(UP:VECTOR, aimVec, -rollAng).
	RETURN LOOKDIRUP(aimVec, rollVector).
}

//	KSP-MATLAB-KSP vector conversion
FUNCTION vecYZ {
	DECLARE PARAMETER input.	//	Expects a vector
	LOCAL output IS V(input:X, input:Z, input:Y).
	RETURN output.
}

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
}

//	Robust calculation of constant acceleration burn time
FUNCTION constAccBurnTime {
	//	Takes minimum engine throttle into account:
	//	continuous throttling down to maintain acceleration makes fuel flow an inverse exponential function of time
	//	but at some point the minimum throttle constraint can be violated - from then, stage will continue to burn
	//	at constant thrust. This means that the stage will burn out faster than expected.
	DECLARE PARAMETER _stage.	//	Expects a lexicon containing at least partially formed logical stage.
								//	This has to contain the following keys:
								//	"massFuel", "massTotal", "engines", "gLim" and "minThrottle".
	
	//	Unpack the structure
	LOCAL engineData IS getThrust(_stage["engines"]).
	LOCAL isp IS engineData[2].
	LOCAL baseFlow IS engineData[1].
	LOCAL mass IS _stage["massTotal"].
	LOCAL fuel IS _stage["massFuel"].
	LOCAL gLim IS _stage["gLim"].
	LOCAL tMin IS _stage["minThrottle"].
	//	Find maximum burn time
	LOCAL maxBurnTime IS isp/gLim * LN( mass/(mass-fuel) ).
	//	If there is no throttling limit - we will always be able to throttle a bit more down.
	//	With no possible constraints to violate, we can just return this theoretical time.
	IF tMin = 0 { RETURN maxBurnTime. }
	//	Otherwise - find time of constraint violation
	LOCAL violationTime IS -isp/gLim * LN(tMin).
	//	If this time is lower than the time we want to burn - we need to act.
	LOCAL constThrustTime IS 0.	//	Declare now, so that we can have a single return statement.
	IF violationTime < maxBurnTime {
		//	First we calculate mass of the fuel burned until violation
		LOCAL burnedFuel IS mass*(1 - CONSTANT:E^(-gLim/isp * violationTime)).
		//	Then, time it will take to burn the rest on constant minimum throttle
		SET constThrustTime TO (fuel - burnedFuel) / (baseFlow * tMin).
	}
	RETURN maxBurnTime + constThrustTime.
}

//	TARGETING FUNCTIONS

//	Update keys in the mission lexicon
FUNCTION missionSetup {
	//	Expects global variables "mission" and "controls" as lexicons

	//	Fix target definition if the burnout altitude is wrong or not given
	IF mission:HASKEY("altitude") {
		IF mission["altitude"] < mission["periapsis"] OR mission["altitude"] > mission["apoapsis"] {
			SET mission["altitude"] TO mission["periapsis"].
		}
	} ELSE {
		mission:ADD("altitude", mission["periapsis"]).
	}
	
	//	Override plane definition if a map target was selected
	IF HASTARGET {
		SET mission["inclination"] TO TARGET:ORBIT:INCLINATION.
		SET mission["LAN"] TO TARGET:ORBIT:LAN.
	}
	
	//	Set default launch direction
	IF NOT mission:HASKEY("direction") {
		mission:ADD("direction", "nearest").
	}
	
	//	Set inclination to launch site latitude, or fix the existing to (-180)-180 degrees range
	IF mission:HASKEY("inclination") {
		UNTIL mission["inclination"] > -180 { SET mission["inclination"] TO mission["inclination"] + 360. }
		UNTIL mission["inclination"] < 180 { SET mission["inclination"] TO mission["inclination"] - 360. }
	} ELSE {
		mission:ADD("inclination", ABS(SHIP:GEOPOSITION:LAT)).
	}
	
	//	Calculate LAN for the "right now" launch, or fix the existing to 0-360 degrees range
	IF mission:HASKEY("LAN") {
		UNTIL mission["LAN"] > 0 { SET mission["LAN"] TO mission["LAN"] + 360. }
		IF mission["LAN"] > 360 { SET mission["LAN"] TO MOD(mission["LAN"], 360). }
	} ELSE {
		//	Calculate what LAN would an orbit passing right above the launch site right now have,
		//	correct for launchTimeAdvance and add some time for the countdown, and set up the new LAN.
		IF mission["direction"] = "nearest" { SET mission["direction"] TO "north". }
		LOCAL currentNode IS nodeVector(mission["inclination"], mission["direction"]).
		LOCAL currentLan IS VANG(currentNode, SOLARPRIMEVECTOR).
		IF VDOT(V(0,1,0), VCRS(currentNode, SOLARPRIMEVECTOR)) < 0 { SET currentLan TO 360 - currentLan. }
		SET mission["LAN"] TO currentLan + (controls["launchTimeAdvance"] + 30)/SHIP:ORBIT:BODY:ROTATIONPERIOD*360.
	}
}

//	Generate a PEGAS-compatible target struct from user-specified one
FUNCTION targetSetup {
	//	Expects a global variable "mission" as lexicon
	
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
}

//	Ascending node vector of the orbit passing right over the launch site
FUNCTION nodeVector {
	DECLARE PARAMETER inc.				//	Inclination of the desired orbit. Expects a scalar.
	DECLARE PARAMETER dir IS "north".	//	Launch direction. Expects a string, either "north" or "south".
	
	//	From right spherical triangle composed of inclination, latitude and "b",
	//	which is angular difference between the desired node vector and projection
	//	of the vector pointing at the launch site onto the equatorial plane.
	LOCAL b IS TAN(90-inc)*TAN(SHIP:GEOPOSITION:LAT).
	SET b TO ARCSIN( MIN(MAX(-1, b), 1) ).
	LOCAL longitudeVector IS VXCL(V(0,1,0), -SHIP:ORBIT:BODY:POSITION):NORMALIZED.
	IF dir = "north" {
		RETURN rodrigues(longitudeVector, V(0,1,0), b).
	} ELSE IF dir = "south" {
		//	This can be easily derived from spherical triangle if one draws a half
		//	of an orbit, from node to node. It is obvious that distance from node to
		//	peak equals 90 degrees, and from that the following results.
		RETURN rodrigues(longitudeVector, V(0,1,0), 180-b).
	} ELSE {
		pushUIMessage("Unknown launch direction. Trying north.", 5, PRIORITY_HIGH).
		RETURN nodeVector(inc, "north").
	}
}

//	Time to next launch opportunity in given direction
FUNCTION orbitInterceptTime {
	DECLARE PARAMETER launchDir IS mission["direction"].	//	Passing as parameter for recursive calls.
	
	//	Expects a global variable "mission" as lexicon
	LOCAL targetInc IS mission["inclination"].
	LOCAL targetLan IS mission["lan"].
	
	//	For "nearest" launch opportunity:
	IF launchDir = "nearest" {
		LOCAL timeToNortherly IS orbitInterceptTime("north").
		LOCAL timeToSoutherly IS orbitInterceptTime("south").
		IF timeToSoutherly < timeToNortherly {
			SET mission["direction"] TO "south".
			RETURN timeToSoutherly.
		} ELSE {
			SET mission["direction"] TO "north".
			RETURN timeToNortherly.
		}
	} ELSE {
		//	Tind the ascending node vector of an orbit of the desired inclination,
		//	that passes above the launch site right now.
		SET currentNode TO nodeVector(targetInc, launchDir).
		//	Then find the ascending node vector of the target orbit.
		LOCAL targetNode IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -targetLan).
		//	Find the angle between them, minding rotation direction, and return as time.
		LOCAL nodeDelta IS VANG(currentNode, targetNode).
		LOCAL deltaDir IS VDOT(V(0,1,0), VCRS(targetNode, currentNode)).
		IF deltaDir < 0 { SET nodeDelta TO 360 - nodeDelta. }
		LOCAL deltaTime IS SHIP:ORBIT:BODY:ROTATIONPERIOD * nodeDelta/360.
		
		RETURN deltaTime.
	}
}

//	Launch azimuth to a given orbit
FUNCTION launchAzimuth {
	//	Expects global variables "upfgTarget" and "mission" as lexicons
	
	LOCAL targetInc IS mission["inclination"].
	LOCAL targetAlt IS upfgTarget["radius"].
	LOCAL targetVel IS upfgTarget["velocity"].
	LOCAL siteLat IS SHIP:GEOPOSITION:LAT.
	IF targetInc < siteLat { pushUIMessage( "Target inclination below launch site!", 5, PRIORITY_HIGH ). }
	
	LOCAL Binertial IS COS(targetInc)/COS(siteLat).
	IF Binertial < -1 { SET Binertial TO -1. }
	IF Binertial > 1 { SET Binertial TO 1. }
	SET Binertial TO ARCSIN(Binertial).		//	In case of an attempt at launch to a lower inclination than reachable
	//LOCAL Vorbit IS SQRT( SHIP:ORBIT:BODY:MU/(SHIP:BODY:RADIUS+targetAlt*1000) ).		//	This is a normal calculation for a circular orbit
	LOCAL Vorbit IS targetVel*COS(upfgTarget["angle"]).									//	But we already have our desired velocity, however we must correct for the flight path angle (only the tangential component matters here)
	LOCAL Vbody IS (2*CONSTANT:PI*SHIP:BODY:RADIUS/SHIP:BODY:ROTATIONPERIOD)*COS(siteLat).
	LOCAL VrotX IS Vorbit*SIN(Binertial)-Vbody.
	LOCAL VrotY IS Vorbit*COS(Binertial).
	LOCAL azimuth IS ARCTAN2(VrotY, VrotX).
	
	//	In MATLAB an azimuth of 0 is due east, while in KSP it's due north.
	//	Return the valid value depending on the launch direction:
	IF mission["direction"] = "north" {
		RETURN 90-azimuth.
	} ELSE IF mission["direction"] = "south" {
		RETURN 90+azimuth.
	} ELSE {
		pushUImessage("Unknown launch direction. Trying north.", 5, PRIORITY_HIGH).
		RETURN 90-azimuth.
	}
}

//	Verifies parameters of the attained orbit
FUNCTION missionValidation {
	FUNCTION difference {
		DECLARE PARAMETER input.		//	Expects scalar
		DECLARE PARAMETER reference.	//	Expects scalar
		DECLARE PARAMETER threshold.	//	Expects scalar
		
		IF ABS(input-reference)<threshold { RETURN TRUE. } ELSE { RETURN FALSE. }
	}
	FUNCTION errorMessage {
		DECLARE PARAMETER input.		//	Expects scalar
		DECLARE PARAMETER reference.	//	Expects scalar
		//	Apoapse/periapse will be rounded to no decimal places, angles rounded to 2.
		LOCAL smartRounding IS 0.
		LOCAL inputAsString IS "" + ROUND(input,0).
		IF inputAsString:LENGTH <= 3 {
			SET smartRounding TO 2.
		}
		LOCAL output IS "" + ROUND(input,smartRounding) + " vs " + ROUND(reference,smartRounding) + " (".
		IF input<reference { SET output TO output + ROUND(input-reference,smartRounding). }
		ELSE { SET output TO output + "+" + ROUND(input-reference,smartRounding). }
		RETURN output + ")".
	}
	//	Expects global variable "mission" as lexicon.
	
	//	Some local variables for tracking mission success/partial success/failure
	LOCAL success IS TRUE.
	LOCAL failure IS FALSE.
	LOCAL apsisSuccessThreshold IS 10000.
	LOCAL apsisFailureThreshold IS 50000.
	LOCAL angleSuccessThreshold IS 0.1.
	LOCAL angleFailureThreshold IS 1.
	
	//	Check every condition
	IF NOT difference(SHIP:ORBIT:PERIAPSIS, mission["periapsis"]*1000, apsisSuccessThreshold) {
		SET success TO FALSE.
		IF NOT difference(SHIP:ORBIT:PERIAPSIS, mission["periapsis"]*1000, apsisFailureThreshold) {
			SET failure TO TRUE.
		}
		PRINT "Periapsis:   " + errorMessage(SHIP:ORBIT:PERIAPSIS, mission["periapsis"]*1000).
	}
	IF NOT difference(SHIP:ORBIT:APOAPSIS, mission["apoapsis"]*1000, apsisSuccessThreshold) {
		SET success TO FALSE.
		IF NOT difference(SHIP:ORBIT:APOAPSIS, mission["apoapsis"]*1000, apsisFailureThreshold) {
			SET failure TO TRUE.
		}
		PRINT "Apoapsis:    " + errorMessage(SHIP:ORBIT:APOAPSIS, mission["apoapsis"]*1000).
	}
	IF NOT difference(SHIP:ORBIT:INCLINATION, mission["inclination"], angleSuccessThreshold) {
		SET success TO FALSE.
		IF NOT difference(SHIP:ORBIT:INCLINATION, mission["inclination"], angleFailureThreshold) {
			SET failure TO TRUE.
		}
		PRINT "Inclination: " + errorMessage(SHIP:ORBIT:INCLINATION, mission["inclination"]).
	}
	IF NOT difference(SHIP:ORBIT:LAN, mission["LAN"], angleSuccessThreshold) {
		SET success TO FALSE.
		IF NOT difference(SHIP:ORBIT:LAN, mission["LAN"], angleFailureThreshold) {
			SET failure TO TRUE.
		}
		PRINT "Long. of AN: " + errorMessage(SHIP:ORBIT:LAN, mission["LAN"]).
	}
	
	//	If at least one condition is not a success - we only have a partial. If at least one condition
	//	is a failure - we have a failure.
	IF failure {
		pushUIMessage( "Mission failure!", 3, PRIORITY_HIGH ).
	} ELSE {
		IF NOT success {
			pushUIMessage( "Partial success.", 3, PRIORITY_HIGH ).
		} ELSE {
			pushUIMessage( "Mission successful!", 3, PRIORITY_HIGH ).
		}
	}
}

//	UPFG HANDLING FUNCTIONS

//	Creates and initializes UPFG internal struct
FUNCTION setupUPFG {
	//	Expects global variables "mission", "upfgState" and "upfgTarget" as lexicons.

	LOCAL curR IS upfgState["radius"].
	LOCAL curV IS upfgState["velocity"].

	SET upfgTarget["normal"] TO targetNormal(mission["inclination"], mission["LAN"]).
	LOCAL desR IS rodrigues(curR, -upfgTarget["normal"], 20):NORMALIZED * upfgTarget["radius"].
	LOCAL tgoV IS upfgTarget["velocity"] * VCRS(-upfgTarget["normal"], desR):NORMALIZED - curV.

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
}

//	Acquire vehicle position data
FUNCTION acquireState {
	//	Expects a global variable "liftoffTime" as scalar
	
	RETURN LEXICON(
		"time", TIME:SECONDS - liftoffTime:SECONDS,
		"mass", SHIP:MASS*1000,
		"radius", vecYZ(SHIP:ORBIT:BODY:POSITION) * -1,
		"velocity", vecYZ(SHIP:ORBIT:VELOCITY:ORBIT)
	).
}

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
}

//	EVENT HANDLING FUNCTIONS

//	Setup system events, currently only countdown messages
FUNCTION setSystemEvents {
	//	Local function - countdown event generator
	FUNCTION makeEvent {
		DECLARE PARAMETER timeAfterLiftoff.	//	Expects a scalar
		DECLARE PARAMETER eventMessage.		//	Expects a string
		
		RETURN LEXICON("time", timeAfterLiftoff, "type", "dummy", "message", eventMessage, "data", LIST()).
	}
	
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
}

//	Setup user events (vehicle sequence)
FUNCTION setUserEvents {
	//	Just a wrapper to a handler which automatically does the setup on its first run.
	userEventHandler().
}

//	Setup vehicle: transform user input to UPFG-compatible struct
FUNCTION setVehicle {
	//	Calculates missing mass inputs (user gives any 2 of 3: total, dry, fuel mass)
	//	Adds payload mass to the mass of each stage
	//	Sets up defaults: acceleration limit (none, 0.0), throttle (1.0), and UPFG MODE
	//	Calculates engine fuel mass flow (if thrust value was given instead) and adjusts for given throttle
	//	Calculates max stage burn time
	
	//	Expects a global variable "vehicle" as list of lexicons and "controls" and "mission" as lexicon.

	LOCAL i IS 0.
	FOR v IN vehicle {
		//	Mass calculations
		IF v:HASKEY("massTotal") AND v:HASKEY("massDry")		{ v:ADD("massFuel",  v["massTotal"]-v["massDry"]).	}
		ELSE IF v:HASKEY("massTotal") AND v:HASKEY("massFuel")	{ v:ADD("massDry",   v["massTotal"]-v["massFuel"]).	}
		ELSE IF v:HASKEY("massFuel") AND v:HASKEY("massDry")	{ v:ADD("massTotal", v["massFuel"] +v["massDry"]).	}
		ELSE { PRINT "Vehicle is ill-defined: missing mass keys in stage " + i. }
		IF mission:HASKEY("payload") {
			SET v["massTotal"] TO v["massTotal"] + mission["payload"].
			SET v["massDry"] TO v["massDry"] + mission["payload"].
		}
		//	Default fields: gLim, minThrottle, throttle, mode
		IF NOT v:HASKEY("gLim")			{ v:ADD("gLim", 0). }
		IF NOT v:HASKEY("minThrottle")	{ v:ADD("minThrottle", 0). }
		//	In case user accidentally entered throttle as percentage instead of a fraction
		ELSE IF v["minThrottle"] > 1.0	{ SET v["minThrottle"] TO v["minThrottle"] / 100.0. }
		IF NOT v:HASKEY("throttle")		{ v:ADD("throttle", 1). }
		ELSE IF v["throttle"] > 1.0		{ SET v["throttle"] TO v["throttle"] / 100.0. }
		v:ADD("mode", 1).
		//	Engine update
		FOR e IN v["engines"] {
			IF NOT e:HASKEY("flow") { e:ADD("flow", e["thrust"] / (e["isp"]*g0) * v["throttle"]). }
		}
		//	Add the shutdown flag - it is optional, but functions rely on its presence
		IF NOT v:HASKEY("shutdownRequired") { v:ADD("shutdownRequired", FALSE). }
		//	Calculate max burn time
		LOCAL combinedEngines IS getThrust(v["engines"]).
		v:ADD("maxT", v["massFuel"] / combinedEngines[1]).
		//	Increment loop counter
		SET i TO i+1.
	}
}

//	Recalculates stage mass parameters basing on the measured mass
FUNCTION recalculateVehicleMass {
	//	Vehicles basing on sustainer cores start the active guidance phase with a stage whose mass is not known at system
	//	initialization - instead, only dry mass of a stage is known, and mass of the fuel needs to be calculated at the
	//	moment of stage activation.
	
	//	Expects global variables "vehicle" as lexicon and "nextStageTime" as scalar.
	DECLARE PARAMETER stageID.
	DECLARE PARAMETER timeAhead IS 0.		//	If we need to know what will the mass be at some time from "now", passing
											//	a positive scalar here will cause subtraction of mass burned by the engines.
	DECLARE PARAMETER recalcNext IS FALSE.	//	TRUE will cause recalculation of the constant-acceleration stage (if present).
	DECLARE PARAMETER updateEvent IS FALSE.	//	TRUE will cause the "nextStageTime" to shift by the change in stage burn times.
	
	LOCAL combinedEngines IS getThrust(vehicle[stageID]["engines"]).
	SET vehicle[stageID]["massTotal"] TO SHIP:MASS*1000 - combinedEngines[1]*timeAhead.
	SET vehicle[stageID]["massFuel"]  TO vehicle[stageID]["massTotal"] - vehicle[stageID]["massDry"].
	LOCAL oldMaxT IS vehicle[stageID]["maxT"].
	SET vehicle[stageID]["maxT"] TO vehicle[stageID]["massFuel"] / combinedEngines[1].
	//	If the stage is followed by a constant acceleration stage - it has to be recalculated as well
	IF recalcNext AND stageID < vehicle:LENGTH - 1 AND vehicle[stageID+1]["mode"] = 2 {
		//	This is almost the same as initializeVehicle code, with one difference: if the (physical) stage requires
		//	explicit shutting down of its engines, this information needs to be retrieved.
		//back to the stage cause the creator will use it to create the new stage, we'll delete it later
		SET vehicle[stageID]["shutdownRequired"] TO vehicle[stageID+1]["shutdownRequired"].
		//	Remove the old stage
		vehicle:REMOVE(stageID+1).
		//	Repeat the steps from initializeVehicle
		LOCAL accLimTime IS accLimitViolationTime(vehicle[stageID]).
		IF accLimTime > 0 AND accLimTime < vehicle[stageID]["maxT"] {
			LOCAL gLimStage IS createAccelerationLimitedStage(vehicle[stageID], accLimTime).
			vehicle:INSERT(stageID+1, gLimStage).
			SET vehicle[stageID]["maxT"] TO accLimTime.
			SET vehicle[stageID]["shutdownRequired"] TO FALSE.
		}
	}
	IF updateEvent {
		SET nextStageTime TO nextStageTime - oldMaxT + vehicle[stageID]["maxT"].
	}
}

//	Calculates the time after which a given stage would exceed its acceleration limit
FUNCTION accLimitViolationTime {
	DECLARE PARAMETER baseStage.	//	Expects a lexicon
	
	IF baseStage["gLim"] = 0 { RETURN -1. }
	
	LOCAL fdmisp IS getThrust(baseStage["engines"]).
	RETURN (baseStage["massTotal"] - fdmisp[0]/baseStage["gLim"]/g0) / fdmisp[1].
}

//	Basing on an existing stage, builds a new virtual stage to handle acceleration limits.
FUNCTION createAccelerationLimitedStage {
	DECLARE PARAMETER baseStage.	//	Expects a lexicon
	DECLARE PARAMETER accLimTime.	//	Time after which the given stage exceeds the limit. Expects a scalar
	
	//	Create a new stage
	LOCAL gLimStage IS LEXICON("mode", 2, "name", "Constant acceleration", "gLim", baseStage["gLim"], "engines", baseStage["engines"]).
	//	Default throttle is irrelevant since it will be dynamically calculated anyway
	gLimStage:ADD("throttle", 1.0).
	//	But we need to inherit the minimum throttle limit from the previous stage
	gLimStage:ADD("minThrottle", baseStage["minThrottle"]).
	//	Copy the shutdown flag
	gLimStage:ADD("shutdownRequired", baseStage["shutdownRequired"]).
	//	Supply it with a staging information
	gLimStage:ADD("staging", LEXICON("jettison", FALSE, "ignition", FALSE)).
	//	Calculate its initial mass
	LOCAL fdmisp IS getThrust(baseStage["engines"]).
	LOCAL burnedFuelMass IS fdmisp[1] * accLimTime.
	gLimStage:ADD("massTotal", baseStage["massTotal"] - burnedFuelMass).
	//	Finish the structure so that we can run the burn time calculation
	gLimStage:ADD("massFuel", baseStage["massFuel"] - burnedFuelMass).
	gLimStage:ADD("massDry", gLimStage["massTotal"] - gLimStage["massFuel"]).
	gLimStage:ADD("maxT", constAccBurnTime(gLimStage)).
	
	RETURN gLimStage.
}

//	Handles definition of the physical vehicle (initial mass of the first actively guided stage, acceleration limits) and
//	initializes the automatic staging sequence.
FUNCTION initializeVehicle {
	//	The first actively guided stage can be a whole new stage (think: Saturn V, S-II), or a sustainer stage that continues
	//	a burn started at liftoff (Atlas V, STS). In the former case, all information is known at liftoff and no updates are
	//	necessary. For the latter, the amount of fuel remaining in the tank is only known at the moment of ignition of the
	//	stage (due to uncertainty in engine spool-up at ignition, and potentially changing time of activation of UPFG). Thus,
	//	the stage - and potentially also its derived const-acc stage - can only be initialized in flight. And this is what the
	//	following function is supposed to do.

	//	Expects a global variable "vehicle" as list of lexicons, "upfgConvergenceDelay" as scalar
	
	LOCAL currentTime IS TIME:SECONDS.
	
	//	If a stage has a staging sequence defined, this means it is a Saturn-like stage which needs no update. Otherwise,
	//	it is a sustainer stage and only its initial (and, hence, dry) mass is known. Actual mass needs to be calculated.
	IF NOT vehicle[0]["staging"]["ignition"] {
		//	We need to know the real mass of the vehicle, but we're doing this "upfgConvergenceDelay" seconds before this
		//	information will be used:
		recalculateVehicleMass(0, upfgConvergenceDelay).
	}
	//	Acceleration limits are handled in the following loop
	FROM { LOCAL i IS 0. } UNTIL i = vehicle:LENGTH STEP { SET i TO i+1. } DO {
		IF vehicle[i]["gLim"]>0 {
			//	Calculate when will the acceleration limit be exceeded
			LOCAL accLimTime IS accLimitViolationTime(vehicle[i]).
			//	If this time is greater than the stage's max burn time - we're good.
			//	Otherwise, we create a virtual stage for the acceleration-limited flight and reduce the burn time of
			//	the violating stage.
			IF accLimTime > 0 AND accLimTime < vehicle[i]["maxT"] {
				LOCAL gLimStage IS createAccelerationLimitedStage(vehicle[i], accLimTime).
				//	Insert it into the list and increment i so that we don't process it next
				vehicle:INSERT(i+1, gLimStage).
				//	Adjust the current stage's burn time
				SET vehicle[i]["maxT"] TO accLimTime.
				//	And remember that it cannot shutdown before the virtual staging
				SET vehicle[i]["shutdownRequired"] TO FALSE.
				//	We want to iterate through all the stages, but we just added one
				SET i TO i+1.
			}
		}
	}
	
	stageEventHandler(currentTime).	//	Schedule ignition of the first UPFG-controlled stage.
}

//	Executes a system event. Currently only supports message printing.
FUNCTION systemEventHandler {
	//	Local function needed here, so we can safely exit the handler on first run without excessive nesting
	FUNCTION setNextEvent {
		SET systemEventPointer TO systemEventPointer + 1.
		IF systemEventPointer < systemEvents:LENGTH {
			WHEN TIME:SECONDS >= liftoffTime:SECONDS + systemEvents[systemEventPointer]["time"] THEN { SET systemEventFlag TO TRUE. }
		}
	}
	
	//	Expects global variables "liftoffTime" as TimeSpan, "systemEvents" as list, "systemEventFlag" as bool and "systemEventPointer" as scalar.
	//	First call initializes and exits without messaging
	IF systemEventPointer = -1 {	//	This var is initialized at -1, so meeting this condition is only possible on first run.
		setNextEvent().
		RETURN.
	}
	
	//	Handle event
	pushUIMessage( systemEvents[systemEventPointer]["message"], 3, PRIORITY_LOW ).
	
	//	Reset event flag
	SET systemEventFlag TO FALSE.
	
	//	Create new event trigger
	setNextEvent().
}

//	Executes a user (sequence) event.
FUNCTION userEventHandler {
	//	Mechanism is very similar to systemEventHandler
	FUNCTION setNextEvent {
		SET userEventPointer TO userEventPointer + 1.
		IF userEventPointer < sequence:LENGTH {
			WHEN TIME:SECONDS >= liftoffTime:SECONDS + sequence[userEventPointer]["time"] THEN { SET userEventFlag TO TRUE. }
		}
	}
	
	//	Expects global variables "sequence" and "vehicle" as list, "userEventFlag" as bool,
	//	"liftoffTime", "steeringRoll", "userEventPointer", "upfgStage" and "nextStageTime" as scalars.
	//	First call initializes and exits without doing anything
	IF userEventPointer = -1 {
		setNextEvent().
		RETURN.
	}
	
	//	Handle event
	LOCAL eType IS sequence[userEventPointer]["type"].
	IF      eType = "print" OR eType = "p" { }
	ELSE IF eType = "stage" OR eType = "s" { STAGE. }
	ELSE IF eType = "jettison" OR eType = "j" {
		//	Jettisoning some mass results in change of vehicle dynamics. The following mechanism allows the system to
		//	deal with this loss, which otherwise would have negative effects on constant-acceleration stages.
		//	The jettisoned mass is subtracted from the current stage's mass (dry and total), and all subsequent stages'
		//	until one that separates the preceding one is found. In case of a const-acc stage, its burn time is also
		//	recalculated. If it so happens that the jettison has occurred during const-acc, changing the burn time is
		//	not enough to ensure safe separation (since the triggers were already set) - in this case, the next stage's
		//	(if there is any) separation delay is increased.
		//	It is theoretically possible that the stage right after the updated constant-acceleration stage will have
		//	no separation nor ignition, and thus no delay can be applied, but since this vehicle configuration is hardly
		//	realistic, this case IS NOT covered here (and will be simply ignored).
		LOCAL dm IS sequence[userEventPointer]["massLost"].
		FROM { LOCAL i IS upfgStage. } UNTIL i = vehicle:LENGTH STEP { SET i TO i+1. } DO {
			//	Reduce mass of this stage
			SET vehicle[i]["massTotal"] TO vehicle[i]["massTotal"] - dm.
			SET vehicle[i]["massDry"] TO vehicle[i]["massDry"] - dm.
			//	Recalculate burn time of const-acc stages
			IF vehicle[i]["mode"] = 2 {
				LOCAL newBurnTime IS constAccBurnTime(vehicle[i]).
				//	If this stage is not being flown - changing the burn time will suffice
				IF i <> upfgStage {
					SET vehicle[i]["maxT"] TO newBurnTime.
				} ELSE IF i+1 < vehicle:LENGTH {
					//	Otherwise, we have to increase a delay on the subsequent stage
					LOCAL addDelay IS newBurnTime - vehicle[i]["maxT"].
					SET nextStageTime TO nextStageTime + addDelay.
				}
			}
			//	Exit the loop if the subsequent stage separates (either via staging or ignition)
			IF (i+1 < vehicle:LENGTH) AND (vehicle[i+1]["staging"]["jettison"] OR vehicle[i+1]["staging"]["ignition"]) { BREAK. }
		}
		//	Finally, stage
		STAGE.
	}
	ELSE IF eType = "throttle" OR eType = "t" {
		//	Throttling is only allowed during the passive guidance phase, as it would ruin burn time predictions used by
		//	UPFG for guidance and stageEvent system for stage timing.
		IF upfgStage < 0 {
			IF NOT sequence[userEventPointer]:HASKEY("message") {
				IF sequence[userEventPointer]["throttle"] < throttleSetting {
					sequence[userEventPointer]:ADD("message", "Throttling down to " + 100*sequence[userEventPointer]["throttle"] + "%").
				} ELSE {
					sequence[userEventPointer]:ADD("message", "Throttling up to " + 100*sequence[userEventPointer]["throttle"] + "%").
				}
			}
			SET throttleSetting TO sequence[userEventPointer]["throttle"].
			SET throttleDisplay TO throttleSetting.
		} ELSE {
			pushUIMessage( "Throttle ignored in active guidance!", 5, PRIORITY_HIGH ).
		}
	}
	ELSE IF eType = "roll" OR eType = "r" {
		SET steeringRoll TO sequence[userEventPointer]["angle"].
		IF NOT sequence[userEventPointer]:HASKEY("message") {
			sequence[userEventPointer]:ADD("message", "Rolling to " + steeringRoll + " degrees").
		}
	}
    ELSE IF eType = "delegate" OR eType = "d" {
		sequence[userEventPointer]["function"]:CALL().
	}
	ELSE { pushUIMessage( "Unknown event type (" + eType + ", message='" + sequence[userEventPointer]["message"] + "')!", 5, PRIORITY_HIGH ). }
	//	Print event message, if requested
	IF sequence[userEventPointer]:HASKEY("message") {
		pushUIMessage( sequence[userEventPointer]["message"] ).
	}
	
	//	Reset event flag
	SET userEventFlag TO FALSE.
	
	//	Create new event trigger
	setNextEvent().
}

//	Executes an automatic staging event. Spawns additional triggers.
FUNCTION stageEventHandler {
	//	Structure is very similar to systemEventHandler, but with a little twist.
	//	Before activating a stage, the vehicle's attitude is held constant. During this period, to save time and ignite the new stage
	//	with UPFG at least closer to convergence, we want to calculate steering for the next stage. Therefore, we decide that the
	//	phrase "current stage" shall mean "the currently guided stage, or the one that will be guided next if this one is almost spent".
	//	Global variable "upfgStage" shall point to this exact stage and must be incremented at the very moment we decide to solve for
	//	the next stage: upon setting the global variable "stagingInProgress".
	FUNCTION setNextEvent {
		DECLARE PARAMETER baseTime IS TIME:SECONDS.	//	Expects a scalar. Meaning: set next stage from this time (allows more precise calculations)
		DECLARE PARAMETER eventDelay IS 0.			//	Expects a scalar. Meaning: if this stage ignites in "eventDelay" seconds from now, the next should ignite in "eventDelay"+"maxT" from now.
		GLOBAL nextStageTime IS baseTime + eventDelay + vehicle[upfgStage]["maxT"].	//	Calculate how long this stage will burn, but don't set an event for the last stage
		IF upfgStage < vehicle:LENGTH-1 {
			WHEN TIME:SECONDS >= nextStageTime THEN { SET stageEventFlag TO TRUE. }
			WHEN TIME:SECONDS >= nextStageTime - stagingKillRotTime THEN {
				SET stagingInProgress TO TRUE.
				SET upfgStage TO upfgStage + 1.
				upfgStagingNotify().	//	Pass information about staging to UPFG handler
			}
		}
	}
	
	//	Expects global variables "liftOffTime" as TimeSpan, "vehicle" as list, "controls" as lexicon, "upfgStage" as scalar and "stageEventFlag" as bool.
	DECLARE PARAMETER currentTime IS TIME:SECONDS.	//	Only passed when run from initializeVehicle
	
	//	First call (we know because upfgStage is still at initial value) only sets up the event for first guided stage.
	IF upfgStage = -1 {
		//	We cannot use setNextEvent because it directly reads vehicle[upfgStage], but we have to do a part of its job
		GLOBAL nextStageTime IS liftOffTime:SECONDS + controls["upfgActivation"].
		WHEN TIME:SECONDS >= nextStageTime THEN { SET stageEventFlag TO TRUE. }
		SET upfgStage TO upfgStage + 1.
		RETURN.
	}
	
	//	Handle event
	LOCAL event IS vehicle[upfgStage]["staging"].
	LOCAL stageName IS vehicle[upfgStage]["name"].
	LOCAL eventDelay IS 0.			//	Many things occur sequentially - this keeps track of the time between subsequent events.
	IF upfgStage > 0 AND vehicle[upfgStage-1]["shutdownRequired"] {
		SET throttleSetting TO 0.
		SET throttleDisplay TO 0.
	}
	IF event["jettison"] {
		GLOBAL stageJettisonTime IS currentTime + event["waitBeforeJettison"].
		//	If we only jettison something but not ignite any new engines, means that this stage is a sustainer-type stage, which
		//	needs additional recalculation of the mass parameters. We store this flag globally here (the trigger must access it).
		GLOBAL needsMassRecalculation IS NOT event["ignition"].
		WHEN TIME:SECONDS >= stageJettisonTime THEN {	STAGE.
														IF needsMassRecalculation { recalculateVehicleMass(upfgStage, 0, TRUE, TRUE). }
														pushUIMessage(stageName + " - separation"). }
		SET eventDelay TO eventDelay + event["waitBeforeJettison"].
	}
	IF event["ignition"] {
		IF event["ullage"] = "rcs" {
			GLOBAL ullageIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= ullageIgnitionTime THEN {	RCS ON. 
															SET SHIP:CONTROL:FORE TO 1.0.
															pushUIMessage(stageName + " - RCS ullage on"). }
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["ullageBurnDuration"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {	STAGE.
															SET stagingInProgress TO FALSE.
															pushUIMessage(stageName + " - ignition"). }
			SET eventDelay TO eventDelay + event["ullageBurnDuration"].
			GLOBAL ullageShutdownTime IS currentTime + eventDelay + event["postUllageBurn"].
			WHEN TIME:SECONDS >= ullageShutdownTime THEN {	SET SHIP:CONTROL:FORE TO 0.0.
															RCS OFF.
															pushUIMessage(stageName + " - RCS ullage off"). }
		} ELSE IF event["ullage"] = "srb" {
			GLOBAL ullageIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= ullageIgnitionTime THEN {	STAGE.
															pushUIMessage(stageName + " - SRB ullage ignited"). }
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["ullageBurnDuration"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {	STAGE.
															SET stagingInProgress TO FALSE.
															pushUIMessage(stageName + " - ignition"). }
			SET eventDelay TO eventDelay + event["ullageBurnDuration"].
		} ELSE IF event["ullage"] = "none" {
			GLOBAL engineIgnitionTime IS currentTime + eventDelay + event["waitBeforeIgnition"].
			WHEN TIME:SECONDS >= engineIgnitionTime THEN {	STAGE.
															SET stagingInProgress TO FALSE.
															pushUIMessage(stageName + " - ignition"). }
			SET eventDelay TO eventDelay + event["waitBeforeIgnition"].
		} ELSE { pushUIMessage( "Unknown event type (" + event["ullage"] + ")!", 5, PRIORITY_HIGH ). }
	} ELSE {
		//	If this event does not need ignition, staging is over at this moment
		SET stagingInProgress TO FALSE.
	}
	pushUIMessage(stageName + " - activation").
	
	//	Reset event flag
	SET stageEventFlag TO FALSE.
	
	//	Create new event trigger
	setNextEvent(currentTime, eventDelay).
}

//	THROTTLE AND STEERING CONTROLS

//	Calculate a steering vector for minimal angle of attack flight (surface-relative)
FUNCTION minAoASteering {
	//	Expects a global variable "mission" as lexicon.
	DECLARE PARAMETER desiredRoll IS 0.	//	Expects a scalar

	//	This is not a "zero AoA steering" by following the current surface velocity vector - we still provide azimuth control
	SET surfVelAngle TO 90 - VANG(SHIP:UP:VECTOR, SHIP:VELOCITY:SURFACE).
	RETURN aimAndRoll(HEADING(mission["launchAzimuth"], surfVelAngle):VECTOR, desiredRoll).
}

//	Interface between stageEventHandler and upfgSteeringControl.
FUNCTION upfgStagingNotify {
	//	Allows stageEventHandler to let upfgSteeringControl know that staging had occured.
	//	Easier to modify this function in case more information needs to be passed rather
	//	than stageEventHandler itself.
	
	//	Expects global variables "upfgConverged" and "usc_stagingNoticed" as bool.
	SET upfgConverged TO FALSE.
	SET usc_stagingNoticed TO FALSE.
}

//	Intelligent wrapper around UPFG that controls steering vector.
FUNCTION upfgSteeringControl {
	//	This function is essentially oblivious to which stage it is guiding (see "stageEventHandler" for more info).
	//	However, it pays attention to UPFG convergence and proceeding staging, ensuring that the vehicle will not
	//	rotate during separation nor will it rotate to an oscillating, unconverged solution.
	FUNCTION resetUPFG {
		//	Reset internal state of the guidance algorithm. Put here as a precaution from early debugging days,
		//	should not be ever called in normal operation (but if it gets called, it's likely to fix UPFG going
		//	crazy).
		//	Important thing to do is to remember fuel burned in the stage before resetting (or set it to zero if
		//	we're in a pre-convergence phase).
		LOCAL tb IS 0.
		IF NOT stagingInProgress { SET tb TO upfgOutput[0]["tb"]. }
		SET upfgOutput[0] TO setupUPFG().
		SET upfgOutput[0]["tb"] TO tb.
		SET usc_convergeFlags TO LIST().
		SET usc_lastGoodVector TO V(1,0,0).
		SET upfgConverged TO FALSE.
		pushUIMessage( "UPFG reset", 5, PRIORITY_CRITICAL ).
	}
	
	//	Expects global variables "upfgConverged", "upfgEverConverged" and "stagingInProgress" as bool,
	//	"steeringVector" as vector, "upfgConvergenceCriterion", "upfgGoodSolutionCriterion" and "steeringRoll"
	//	as scalars.
	//	Owns global variables "usc_lastGoodVector" as vector, "usc_convergeFlags" as list, "usc_stagingNoticed" as bool and 
	//	"usc_lastIterationTime" as scalar.
	DECLARE PARAMETER vehicle.		//	Expects a list of lexicon
	DECLARE PARAMETER upfgStage.	//	Expects a scalar
	DECLARE PARAMETER upfgTarget.	//	Expects a lexicon
	DECLARE PARAMETER upfgState.	//	Expects a lexicon
	DECLARE PARAMETER upfgInternal.	//	Expects a lexicon
	
	//	First run marked by undefined globals
	IF NOT (DEFINED usc_lastGoodVector) {
		GLOBAL usc_lastGoodVector IS V(1,0,0).
		GLOBAL usc_convergeFlags IS LIST().
		GLOBAL usc_stagingNoticed IS FALSE.
		GLOBAL usc_lastIterationTime IS upfgState["time"].
	}
	
	//	Run UPFG
	LOCAL currentIterationTime IS upfgState["time"].
	LOCAL lastTgo IS upfgInternal["tgo"].
	LOCAL currentVehicle IS vehicle:SUBLIST(upfgStage,vehicle:LENGTH-upfgStage).
	LOCAL upfgOutput IS upfg(currentVehicle, upfgTarget, upfgState, upfgInternal).
	
	//	Convergence check. The rule is that time-to-go as calculated between iterations
	//	should not change significantly more than the time difference between those iterations.
	//	Uses upfgState as timestamp, for equal grounds for comparison.
	//	Requires (a hardcoded) number of consecutive good values before calling it a convergence.
	LOCAL iterationDeltaTime IS currentIterationTime - usc_lastIterationTime.
	IF stagingInProgress {
		//	If the stage hasn't yet been activated, then we're doing a pre-flight convergence.
		//	That means that time effectively doesn't pass for the algorithm - so neither the
		//	iteration takes any time, nor any fuel (measured with remaining time of burn) is
		//	deducted from the stage.
		SET iterationDeltaTime TO 0.
		SET upfgOutput[0]["tb"] TO 0.
	}
	SET usc_lastIterationTime TO currentIterationTime.
	LOCAL expectedTgo IS lastTgo - iterationDeltaTime.
	SET lastTgo TO upfgOutput[1]["tgo"].
	IF ABS(expectedTgo-upfgOutput[1]["tgo"]) < upfgConvergenceCriterion {
		IF usc_lastGoodVector <> V(1,0,0) {
			IF VANG(upfgOutput[1]["vector"], usc_lastGoodVector) < upfgGoodSolutionCriterion {
				usc_convergeFlags:ADD(TRUE).
			} ELSE {
				IF NOT stagingInProgress {
					resetUPFG().
				}
			}
		} ELSE {
			usc_convergeFlags:ADD(TRUE).
		}
	} ELSE { SET usc_convergeFlags TO LIST(). }
	//	If we have enough number of consecutive good results - we're converged.
	IF usc_convergeFlags:LENGTH = 2 {
		SET upfgConverged TO TRUE.
		SET upfgEverConverged TO TRUE.
		SET usc_convergeFlags TO LIST(TRUE, TRUE).
	}
	//	Check if we can steer
	IF upfgConverged AND NOT stagingInProgress {
		SET steeringVector TO aimAndRoll(vecYZ(upfgOutput[1]["vector"]), steeringRoll).
		SET usc_lastGoodVector TO upfgOutput[1]["vector"].
	} ELSE IF NOT upfgEverConverged {
		//	Remain in the min-AoA mode if this is the first run of UPFG
		SET steeringVector TO minAoASteering(steeringRoll).
	}
	RETURN upfgOutput[0].
}

//	Throttle controller
FUNCTION throttleControl {
	//	Expects global variables "vehicle" as list, "upfgStage", "throttleSetting" and "throttleDisplay" as scalars and "stagingInProgress" as bool.
	
	//	If we're guiding a stage nominally, it's simple. But if the stage is about to change into the next one,
	//	value of "upfgStage" is already incremented. In this case we shouldn't use the next stage values (this
	//	would ruin constant-acceleration stages).
	LOCAL whichStage IS upfgStage.
	IF stagingInProgress {
		SET whichStage TO upfgStage - 1.
		IF vehicle[whichStage]["shutdownRequired"] { RETURN. }
	}
	
	IF vehicle[whichStage]["mode"] = 1 {
		SET throttleSetting TO vehicle[whichStage]["throttle"].
		SET throttleDisplay TO throttleSetting.
	}
	ELSE IF vehicle[whichStage]["mode"] = 2 {
		LOCAL nominalThrust_ IS getThrust(vehicle[whichStage]["engines"]).
		LOCAL nominalThrust IS nominalThrust_[0].
		LOCAL throttleLimit IS vehicle[whichStage]["minThrottle"].
		LOCAL desiredThrottle IS SHIP:MASS*1000*vehicle[whichStage]["gLim"]*g0 / nominalThrust.
		//	Realism Overhaul considers in-game throttle not as absolute, but relative to the allowed throttle range of the engine.
		//	Setting throttle to 0.5 for an engine with throttle range 0.4-1.0 actually results in a 0.7 throttle setting.
		SET throttleSetting TO (desiredThrottle - throttleLimit) / (1 - throttleLimit).
		//	If the algorithm requests a throttle setting lower than minimum limit, we might accidentally shutdown.
		SET throttleSetting TO MAX(throttleSetting, 0.01).
		//	For the GUI printout however, we want to see the final throttle value.
		SET throttleDisplay TO desiredThrottle.
	}
	ELSE { pushUIMessage( "throttleControl stage error (stage=" + upfgStage + "(" + whichStage + "), mode=" + vehicle[whichStage]["mode"] + ")!", 5, PRIORITY_CRITICAL ). }.
}
