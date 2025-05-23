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
		SET F TO F + isp*dm_*CONSTANT:g0.
	}
	SET isp TO F/(dm*CONSTANT:g0).

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
	LOCAL total IS _stage["massTotal"].
	LOCAL fuel IS _stage["massFuel"].
	LOCAL gLim IS _stage["gLim"].
	LOCAL tMin IS _stage["minThrottle"].
	//	Find maximum burn time
	LOCAL maxBurnTime IS isp/gLim * LN( total/(total-fuel) ).
	//	If there is no throttling limit - we will always be able to throttle a bit more down.
	//	With no possible constraints to violate, we can just return this theoretical time.
	IF tMin = 0 { RETURN maxBurnTime. }
	//	Otherwise - find time of constraint violation
	LOCAL violationTime IS -isp/gLim * LN(tMin).
	//	If this time is lower than the time we want to burn - we need to act.
	LOCAL constThrustTime IS 0.	//	Declare now, so that we can have a single return statement.
	IF violationTime < maxBurnTime {
		//	First we calculate mass of the fuel burned until violation
		LOCAL burnedFuel IS total*(1 - CONSTANT:E^(-gLim/isp * violationTime)).
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
		SET mission["LAN"] TO currentLan + (controls["launchTimeAdvance"] + 32)/SHIP:ORBIT:BODY:ROTATIONPERIOD*360.
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
		"normal", V(0,0,0)	//	temporarily unset - due to KSP's silly coordinate system this needs to be recalculated every time step, so we will not bother with it for now
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
	//	Expects a global variable "liftoffTime" as timespan

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

//	VEHICLE DATA PREPARATION FOR UPFG

//	Check whether all required control keys are present
FUNCTION checkControls {
	//	Check configuration and crash if there are errors
	LOCAL errorsFound IS FALSE.

	//	Check if the mandatory keys are present
	IF NOT (controls:HASKEY("launchTimeAdvance") AND controls:HASKEY("upfgActivation")) {
		SET errorsFound TO TRUE.
		PRINT "Mandatory control keys missing!".
	}

	//	Check if passive guidance is configured correctly
	LOCAL hasPOA IS controls:HASKEY("pitchOverAngle").
	LOCAL hasVAT IS controls:HASKEY("verticalAscentTime").
	IF controls:HASKEY("pitchProgram") {
		IF hasPOA OR hasVAT {
			SET errorsFound TO TRUE.
			PRINT "Passive guidance misconfigured: multiple keys defined!".
		}
		//	Check if the pitch program has the correct structure
		IF NOT (controls["pitchProgram"]:HASKEY("pitch") AND controls["pitchProgram"]:HASKEY("altitude")) {
			SET errorsFound TO TRUE.
			PRINT "Passive guidance misconfigured: bad program definition!".
		} ELSE {
			IF controls["pitchProgram"]["pitch"]:LENGTH < 2 {
				SET errorsFound TO TRUE.
				PRINT "Passive guidance misconfigured: not enough control points!".
			}
			IF controls["pitchProgram"]["pitch"]:LENGTH <> controls["pitchProgram"]["altitude"]:LENGTH {
				SET errorsFound TO TRUE.
				PRINT "Passive guidance misconfigured: pitch-altitude mismatch!".
			}
		}
	} ELSE {
		IF NOT (hasPOA AND hasVAT) {
			SET errorsFound TO TRUE.
			PRINT "Passive guidance misconfigured: missing keys!".
		}
	}

	//	Crash the system if known errors were found
	IF errorsFound {
		PRINT "ERRORS IN CONTROL CONFIGURATION FOUND".
		PRINT "For your convenience, PEGAS will now crash.".
		PRINT " ".
		PRINT " ".
		PRINT " ".
		PRINT " ".
		PRINT " ".
		SET _ TO __DELIBERATE_CRASH__.
	}
}

//	Setup vehicle: transform user input to UPFG-compatible struct
FUNCTION setVehicle {
	//	Calculates missing mass inputs (user gives any 2 of 3: total, dry, fuel mass)
	//	Adds payload mass to the mass of each stage
	//	Sets up defaults: acceleration limit (none, 0.0), throttle (1.0), and UPFG MODE
	//	Calculates engine fuel mass flow (if thrust value was given instead) and adjusts for given throttle
	//	Calculates max stage burn time

	//	Expects a global variable "vehicle" as list of lexicons and "controls" and "mission" as lexicon.

	LOCAL errorsFound IS FALSE.
	LOCAL i IS 0.
	FOR vst IN vehicle {
		//	Mass calculations
		IF vst:HASKEY("massTotal") AND vst:HASKEY("massDry")		{ vst:ADD("massFuel",  vst["massTotal"]-vst["massDry"]).	}
		ELSE IF vst:HASKEY("massTotal") AND vst:HASKEY("massFuel")	{ vst:ADD("massDry",   vst["massTotal"]-vst["massFuel"]).	}
		ELSE IF vst:HASKEY("massFuel") AND vst:HASKEY("massDry")	{ vst:ADD("massTotal", vst["massFuel"] +vst["massDry"]).	}
		ELSE {
			PRINT "Vehicle error: missing mass keys in stage " + i.
			SET errorsFound TO TRUE.
		}
		IF mission:HASKEY("payload") {
			SET vst["massTotal"] TO vst["massTotal"] + mission["payload"].
			SET vst["massDry"] TO vst["massDry"] + mission["payload"].
		}
		//	Default fields: gLim, minThrottle, throttle, mode
		IF NOT vst:HASKEY("gLim")			{ vst:ADD("gLim", 0). }
		IF NOT vst:HASKEY("minThrottle")	{ vst:ADD("minThrottle", 0). }
		//	In case user accidentally entered throttle as percentage instead of a fraction
		ELSE IF vst["minThrottle"] > 1.0	{ SET vst["minThrottle"] TO vst["minThrottle"] / 100.0. }
		IF NOT vst:HASKEY("throttle")		{ vst:ADD("throttle", 1). }
		ELSE IF vst["throttle"] > 1.0		{ SET vst["throttle"] TO vst["throttle"] / 100.0. }
		vst:ADD("mode", 1).
		//	Engine update
		FOR e IN vst["engines"] {
			IF NOT e:HASKEY("flow") { e:ADD("flow", e["thrust"] / (e["isp"]*CONSTANT:g0) * vst["throttle"]). }
			IF NOT e:HASKEY("tag") { e:ADD("tag", ""). }
		}
		//	Check if the staging configuration has the correct flags
		IF NOT vst:HASKEY("staging") {
			PRINT "Vehicle error: undefined staging for stage " + i.
			SET errorsFound TO TRUE.
		} ELSE {
			IF NOT vst["staging"]:HASKEY("jettison") OR NOT vst["staging"]:HASKEY("ignition") {
				PRINT "Vehicle error: misconfigured staging for stage " + i.
				SET errorsFound TO TRUE.
			} ELSE {
				IF vst["staging"]["jettison"] AND (NOT vst["staging"]:HASKEY("waitBeforeJettison")) {
					PRINT "Vehicle error: 'waitBeforeJettison' missing".
					SET errorsFound TO TRUE.
				}
				IF vst["staging"]["ignition"] AND (NOT vst["staging"]:HASKEY("waitBeforeIgnition")) {
					PRINT "Vehicle error: 'waitBeforeIgnition' missing".
					SET errorsFound TO TRUE.
				}
				IF vst["staging"]["ignition"] AND (NOT vst["staging"]:HASKEY("ullage")) {
					PRINT "Vehicle error: 'ullage' missing".
					SET errorsFound TO TRUE.
				} ELSE IF vst["staging"]:HASKEY("ullage") {
					IF LIST("none", "srb", "rcs", "hot"):FIND(vst["staging"]["ullage"]) < 0 {
						PRINT "Vehicle error: unknown ullage mode".
						SET errorsFound TO TRUE.
					} ELSE IF LIST("none", "hot"):FIND(vst["staging"]["ullage"]) < 0 AND NOT vst["staging"]:HASKEY("ullageBurnDuration") {
						PRINT "Vehicle error: 'ullageBurnDuration' missing".
						SET errorsFound TO TRUE.
					} ELSE IF vst["staging"]["ullage"] = "rcs" AND NOT vst["staging"]:HASKEY("postUllageBurn") {
						PRINT "Vehicle error: 'postUllageBurn' missing".
						SET errorsFound TO TRUE.
					}
				}
			}
			IF NOT vst["staging"]:HASKEY("postStageEvent") {
				vst["staging"]:ADD("postStageEvent", FALSE).
			} ELSE {
				IF vst["staging"]["postStageEvent"] AND (NOT vst["staging"]:HASKEY("waitBeforePostStage")) {
					PRINT "Vehicle error: 'waitBeforePostStage' missing".
					SET errorsFound TO TRUE.
				}
			}
		}
		//	Add the shutdown flag - it is optional, but functions rely on its presence
		IF NOT vst:HASKEY("shutdownRequired") { vst:ADD("shutdownRequired", FALSE). }
		//	Calculate max burn time
		LOCAL combinedEngines IS getThrust(vst["engines"]).
		vst:ADD("maxT", vst["massFuel"] / combinedEngines[1]).
		//	Internal flags
		vst:ADD("followedByVirtual", FALSE).
		vst:ADD("isVirtualStage", FALSE).
		vst:ADD("virtualStageType", "regular").
		vst:ADD("isSustainer", FALSE).	//	Only for display purposes: see refreshUI, initializeVehicleForUPFG
		//	Increment loop counter
		SET i TO i+1.
	}

	//	Crash the system if known errors were found
	IF errorsFound {
		PRINT "ERRORS IN VEHICLE CONFIGURATION FOUND".
		PRINT "For your convenience, PEGAS will now crash.".
		PRINT " ".
		PRINT " ".
		PRINT " ".
		PRINT " ".
		PRINT " ".
		SET _ TO __DELIBERATE_CRASH__.
	}
}

//	Calculate the sum of all delays before the actual ignition of a given stage
FUNCTION getStageDelays {
	DECLARE PARAMETER thisStage.    //	Expects a lexicon.

	LOCAL staging IS thisStage["staging"].
	LOCAL totalDelays IS 0.
	IF staging:HASKEY("waitBeforeJettison") {
		SET totalDelays TO totalDelays + staging["waitBeforeJettison"].
	}
	IF staging:HASKEY("waitBeforeIgnition") {
		SET totalDelays TO totalDelays + staging["waitBeforeIgnition"].
	}
	IF staging:HASKEY("ullageBurnDuration") {
		SET totalDelays TO totalDelays + staging["ullageBurnDuration"].
	}

	RETURN totalDelays.
}

//	Find the first event of (one of) a given type occurring after a given time
FUNCTION firstEventSince {
	//	By default returns only "jettison" and "shutdown" events. Pass empty list to return all events.
	//	Returns an event of a dummy type "none" with very large time in case no proper events were found.
	DECLARE PARAMETER startTime.
	DECLARE PARAMETER filterOnly IS LIST("jettison", "j", "shutdown", "u").

	IF filterOnly:LENGTH > 0 {
		//	Filtered search
		FOR event IN sequence {
			IF filterOnly:FIND(event["type"]) >= 0 AND event["time"] > startTime {
				RETURN event.
			}
		}
	} ELSE {
		//	Unfiltered search
		FOR event IN sequence {
			IF event["time"] > startTime {
				RETURN event.
			}
		}
	}
	RETURN LEXICON("type", "none", "time", 1e9). // no more events
}

//	Handler for the constant acceleration mode switch
FUNCTION init4upfg_constantAcceleration {
	//	Expects and modifies a global variable "vehicle" of type list.
	DECLARE PARAMETER stageID.			//	Index of a stage to split (integer)
	DECLARE PARAMETER accLimTime.		//	Time at which to perform the split, relative to stageStartTime (scalar)

	LOCAL thrustFlowIsp IS getThrust(vehicle[stageID]["engines"]).
	//	Start with the original stage to inherit all of the basic parameters
	LOCAL gLimStage IS vehicle[stageID]:COPY().
	//	Set the constant-acceleration mode and disable staging
	SET gLimStage["mode"] TO 2.
	SET gLimStage["staging"] TO LEXICON("jettison", FALSE, "ignition", FALSE, "postStageEvent", FALSE).
	//	Calculate its initial mass and burn time
	LOCAL burnedFuelMass IS thrustFlowIsp[1] * accLimTime.
	SET gLimStage["massTotal"] TO gLimStage["massTotal"] - burnedFuelMass.
	SET gLimStage["massFuel"] TO gLimStage["massFuel"] - burnedFuelMass.
	SET gLimStage["maxT"] TO constAccBurnTime(gLimStage).
	//	Insert it into the list
	SET gLimStage["isSustainer"] TO FALSE.
	SET gLimStage["isVirtualStage"] TO TRUE.
	SET gLimStage["virtualStageType"] TO "virtual (const-acc)".
	vehicle:INSERT(stageID + 1, gLimStage).
	//	Adjust the burn time of the current stage
	SET vehicle[stageID]["maxT"] TO accLimTime.
	//	And remember that it cannot shutdown before the virtual staging
	SET vehicle[stageID]["shutdownRequired"] TO FALSE.
	SET vehicle[stageID]["followedByVirtual"] TO TRUE.
}

//	Handler for jettison virtual stage switch
FUNCTION init4upfg_jettison {
	//	Expects and modifies a global variable "vehicle" of type list.
	DECLARE PARAMETER stageID.			//	Index of a stage to split (integer)
	DECLARE PARAMETER stageStartTime.	//	Activation time of that stage (scalar)
	DECLARE PARAMETER event.			//	The exact event that causes the virtual stage (lexicon)

	LOCAL thrustFlowIsp IS getThrust(vehicle[stageID]["engines"]).
	//	First, calculate when does the event happen relative to the stage
	LOCAL startToJettison IS event["time"] - stageStartTime.
	//	Then, how much fuel is consumed prior to the event (beware: this might happen during a const-acc stage too)
	LOCAL fuelBurnedUntil IS CHOOSE
		thrustFlowIsp[1] * startToJettison IF vehicle[stageID]["mode"] = 1
		ELSE vehicle[stageID]["massTotal"]*(1 - CONSTANT:E^(-vehicle[stageID]["gLim"]/thrustFlowIsp[2] * startToJettison)).
	//	Now, create the "after" stage and insert it into the vehicle description
	SET afterStage TO vehicle[stageID]:COPY().
	SET afterStage["massFuel"] TO afterStage["massFuel"] - fuelBurnedUntil.
	SET afterStage["massDry"] TO afterStage["massDry"] - event["massLost"].
	SET afterStage["massTotal"] TO afterStage["massFuel"] + afterStage["massDry"].
	//	Burn time of this new stage is also dependent on the mode:
	SET afterStage["maxT"] TO CHOOSE
		afterStage["maxT"] - startToJettison IF afterStage["mode"] = 1
		ELSE constAccBurnTime(afterStage).
	//	CRUCIAL: this new stage is already ignited, so we must not try to start it again
	SET afterStage["staging"] TO LEXICON("jettison", FALSE, "ignition", FALSE, "postStageEvent", FALSE).
	//	Mark the new stage as virtual and label it
	SET afterStage["isSustainer"] TO FALSE.
	SET afterStage["isVirtualStage"] TO TRUE.
	SET afterStage["virtualStageType"] TO CHOOSE
		"virtual (post-jettison)" IF afterStage["mode"] = 1
		ELSE "virtual (post-jet. const)".
	vehicle:INSERT(stageID + 1, afterStage).
	//	Finally, update the original stage
	SET vehicle[stageID]["massFuel"] TO fuelBurnedUntil.
	SET vehicle[stageID]["massDry"] TO vehicle[stageID]["massTotal"] - vehicle[stageID]["massFuel"].
	SET vehicle[stageID]["maxT"] TO startToJettison.
	SET vehicle[stageID]["shutdownRequired"] TO FALSE.	//	If this is needed, it's on the new stage
	SET vehicle[stageID]["followedByVirtual"] TO TRUE.
	//	As a safety precaution, label the event as processed
	SET event["_processed"] TO TRUE.
}

//	Handler for engine shutdown virtual stage switch
FUNCTION init4upfg_shutdown {
	//	Expects and modifies a global variable "vehicle" of type list.
	DECLARE PARAMETER stageID.			//	Index of a stage to split (integer)
	DECLARE PARAMETER stageStartTime.	//	Activation time of that stage (scalar)
	DECLARE PARAMETER event.			//	The exact event that causes the virtual stage (lexicon)

	LOCAL thrustFlowIsp IS getThrust(vehicle[stageID]["engines"]).
	//	First, calculate when does the event happen relative to the stage
	LOCAL startToJettison IS event["time"] - stageStartTime.
	//	Then, how much fuel is consumed prior to the event (beware: this might happen during a const-acc stage too)
	LOCAL fuelBurnedUntil IS CHOOSE
		thrustFlowIsp[1] * startToJettison IF vehicle[stageID]["mode"] = 1
		ELSE vehicle[stageID]["massTotal"]*(1 - CONSTANT:E^(-vehicle[stageID]["gLim"]/thrustFlowIsp[2] * startToJettison)).
	//	Create the "after" stage. Note that this might have been a const-acc stage and a shutdown might have brought
	//	it back below the gLimit, in which case we should return to mode 1 (max thrust).
	SET afterStage TO vehicle[stageID]:COPY().
	//	Start with the basic information
	SET afterStage["massFuel"] TO afterStage["massFuel"] - fuelBurnedUntil.
	SET afterStage["massTotal"] TO afterStage["massTotal"] - fuelBurnedUntil.
	//	Go through the engines and separate these that aren't being shut down
	SET remainingEngines TO LIST().
	FOR engine IN vehicle[stageID]["engines"] {
		IF engine["tag"] <> event["engineTag"] {
			remainingEngines:ADD(engine).
		}
	}
	SET afterStage["engines"] TO remainingEngines.
	LOCAL newThrustFlowIsp IS getThrust(afterStage["engines"]).
	//	Handle the potential gLim situation
	IF afterStage["mode"] = 2 {
		LOCAL initialG IS newThrustFlowIsp[0] / (afterStage["massTotal"] * CONSTANT:g0).
		IF initialG < afterStage["gLim"] {
			SET afterStage["mode"] TO 1.
			//	Switching back to mode 2 at the right time will be handled by the main function
		}
	}
	//	Calculate the new burn time
	SET afterStage["maxT"] TO CHOOSE
		afterStage["massFuel"] / newThrustFlowIsp[1] IF afterStage["mode"] = 1
		ELSE constAccBurnTime(afterStage).
	//	CRUCIAL: this new stage is already ignited, so we must not try to start it again
	SET afterStage["staging"] TO LEXICON("jettison", FALSE, "ignition", FALSE, "postStageEvent", FALSE).
	//	Mark the new stage as a virtual one and label it
	SET afterStage["isSustainer"] TO FALSE.
	SET afterStage["isVirtualStage"] TO TRUE.
	SET afterStage["virtualStageType"] TO CHOOSE
		"virtual (post-shutdown)" IF afterStage["mode"] = 1
		ELSE "virtual (post-shut const)".
	vehicle:INSERT(stageID + 1, afterStage).
	//	Finally, update the original stage
	SET vehicle[stageID]["massFuel"] TO fuelBurnedUntil.
	SET vehicle[stageID]["massDry"] TO vehicle[stageID]["massTotal"] - vehicle[stageID]["massFuel"].
	SET vehicle[stageID]["maxT"] TO startToJettison.
	SET vehicle[stageID]["shutdownRequired"] TO FALSE.	//	If this is needed, it's on the new stage
	SET vehicle[stageID]["followedByVirtual"] TO TRUE.
	//	As a safety precaution, label the event as processed
	SET event["_processed"] TO TRUE.
}

//	Update the vehicle definition to account for sequence events and acceleration limits
FUNCTION initializeVehicleForUPFG {
	//	This function transforms the vehicle definition in the following way:
	//	* sustainer-type first stage is updated basing on measured vehicle data (since the exact amount of fuel in this
	//	  type of stage is not known until after the launch),
	//	* shutdown and jettison events are handled by inserting virtual stages into the vehicle,
	//	* constant acceleration stages are handled by inserting virtual stages into the vehicle.

	//	Expects global variables:
	//	"vehicle" as list of lexicons
	//	"sequence" as list of lexicons
	//	"controls" as lexicon
	//	"SETTINGS" as lexicon
	//	"upfgStage" as scalar
	//	"stagingInProgress" as bool
	//	"prestageHold" as bool

	//	If a stage has an ignition command in its staging sequence, this means it is a Saturn-like stage (i.e. a spent stage
	//	for atmospheric flight is jettisoned, and the active guidance is engaged for a new stage) and it needs no update.
	//	Otherwise it is a sustainer stage (Shuttle-like) and only its initial (and, hence, dry) mass is known. Actual mass
	//	needs to be measured and burn time calculated.
	IF NOT vehicle[0]["staging"]["ignition"] {
		//	We need to know what the real mass of the vehicle will be when UPFG is activated. Since this function is called
		//	a known amount of time prior to that (defined in SETTINGS["upfgConvergenceDelay"]), we can calculate that.
		LOCAL combinedEngines IS getThrust(vehicle[0]["engines"]).
		SET vehicle[0]["massTotal"] TO SHIP:MASS*1000 - combinedEngines[1]*SETTINGS["upfgConvergenceDelay"].
		SET vehicle[0]["massFuel"] TO vehicle[0]["massTotal"] - vehicle[0]["massDry"].
		SET vehicle[0]["maxT"] TO vehicle[0]["massFuel"] / combinedEngines[1].
		SET vehicle[0]["isSustainer"] TO TRUE.
	}

	//	The vehicle update algorithm is as follows:
	//	1. Iterate over the vehicle (initially the list only contains regular stages).
	//	2. For each stage, check if it's a gLim stage and when would the violation happen.
	//	3. Check what's the earliest vehicle-changing event (i.e. jettison OR shutdown) for this stage.
	//	4. Handle whatever comes earlier: event or gLim violation. This means inserting a virtual stage for the given
	//	situation using one of the dedicated handler functions ("init4upfg_xxx"). Iterate on if nothing needs to be done.
	LOCAL stageStartTime IS controls["upfgActivation"].
	FROM { LOCAL i IS 0. } UNTIL i = vehicle:LENGTH STEP { SET i TO i + 1. } DO {
		LOCAL dontHandleEvents IS FALSE.	//	a CONTINUE keyword would be great
		//	Get the basic information about the stage
		SET stageStartTime TO stageStartTime + getStageDelays(vehicle[i]).
		LOCAL stageEndTime IS stageStartTime + vehicle[i]["maxT"].
		LOCAL thrustFlowIsp IS getThrust(vehicle[i]["engines"]).
		//	Fetch the next event that has a chance to modify the vehicle
		LOCAL nextEvent IS firstEventSince(stageStartTime).
		IF nextEvent["type"] = "none" {
			SET dontHandleEvents TO TRUE.	//	There are no more events to handle
		}
		//	For acceleration-limited stages, check what happens earlier: limit violation or a sequence event
		IF vehicle[i]["gLim"] > 0 AND vehicle[i]["mode"] = 1 {
			LOCAL gLimitExceeded IS (vehicle[i]["massTotal"] - thrustFlowIsp[0]/vehicle[i]["gLim"]/CONSTANT:g0) / thrustFlowIsp[1].
			IF stageStartTime + gLimitExceeded < stageEndTime AND stageStartTime + gLimitExceeded < nextEvent["time"] {
				init4upfg_constantAcceleration(i, gLimitExceeded).
				SET dontHandleEvents TO TRUE.
			}
		}
		//	Now is the time to handle the event, if found any
		IF NOT dontHandleEvents AND nextEvent["time"] < stageEndTime {
			IF nextEvent["type"] = "jettison" {
				init4upfg_jettison(i, stageStartTime, nextEvent).
			} ELSE IF nextEvent["type"] = "shutdown" {
				init4upfg_shutdown(i, stageStartTime, nextEvent).
			}
		}
		//	No need to do any special increments or direct handling of a potential subsequent events happening for this stage;
		//	simply iterating forward to the next stage (=potentially virtual stage produced in this iteration) lets us do that.
		//	The only thing we have to take care of is incrementing the stageStartTime AFTER everything was handled (after all,
		//	maxT could have been altered).
		SET stageStartTime TO stageStartTime + vehicle[i]["maxT"].
	}

	//	Safety check: if there are any events happening during the active phase but we did not account for them, it's a problem.
	//	It is possible that, due to user error or a complex vehicle configuration, a sequence event ended up happening DURING
	//	the staging sequence (i.e. after stageEndTime of every stage, but before stageStartTime of any stage). This is a serious
	//	issue which cannot be automatically resolved; best we can do is catch it, produce an error log, and emit a message.
	LOCAL errorsFound IS LIST().
	LOCAL i IS 0.
	FOR event IN sequence {
		IF event["time"] > controls["upfgActivation"] {
			IF (event["type"] = "jettison" OR event["type"] = "j" OR event["type"] = "shutdown" OR event["type"] = "u") {
				IF NOT event:HASKEY("_processed") OR event["_processed"] = FALSE {
					errorsFound:ADD(i).
				}
			}
		}
		SET i TO i + 1.
	}
	IF errorsFound:LENGTH > 0 {
		IF EXISTS("sequence_errors.log") { DELETEPATH("sequence_errors.log"). }
		LOG vehicle TO "sequence_errors.log".
		LOG sequence TO "sequence_errors.log".
		LOG "Offending events are: " TO "sequence_errors.log".
		LOG errorsFound TO "sequence_errors.log".
		pushUIMessage("ERROR: Some events were not processed!", 10, PRIORITY_HIGH).
	}
	//	Finally, we are ready to spawn events for all the stages
	spawnStagingEvents().
	//	The vehicle is ready so we can start the actual preconvergence for the first actively guided stage
	SET upfgStage TO 0.
	SET stagingInProgress TO TRUE.
	SET prestageHold TO TRUE.
}

//	Utility to keep track of actively guided stage burnouts
FUNCTION updateStageEndTime {
	//	The staging event calls this when a stage is activated, to calculate when the stage will run out of fuel.
	//	This is useful, because we can use this value to estimate a true Tgo for that stage. Doubly so, as we can
	//	easily calculate the total burn time for the entire physical stage. The value might be off by 1-2 seconds
	//	because of the engine spool-up time (#wontfix).
	//	Expects global variables:
	//	"vehicle" as list
	//	"upfgStage" as scalar
	LOCAL stageBurnTime IS 0.
	//	Calculate the complete burn time including the subsequent virtual stages.
	LOCAL i IS 0.
	FOR stg in vehicle {
		//	Forward to the current stage
		IF i < upfgStage {} ELSE {
			SET stageBurnTime TO stageBurnTime + stg["maxT"].
			IF NOT stg["followedByVirtual"] {
				BREAK.
			}
		}
		SET i TO i + 1.
	}
	//	Since this stage has been activated just now, this is when it will burn out:
	SET stageEndTime TO TIME:SECONDS + stageBurnTime.
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

//	Intelligent wrapper around UPFG that controls the steering vector.
FUNCTION upfgSteeringControl {
	//	This function controls the entire process of active guidance by handling three tasks:
	//	* calling UPFG,
	//	* checking whether guidance has converged,
	//	* checking vehicle status to engage or disengage steering.
	//	Convergence check is necessary to ensure the vehicle does not rotate towards an incomplete, possibly
	//	oscillating solution. Details as described below, but the status handling part needs some explanation.
	//	This function can be called in three different situations: primarily of course in the nominal part of
	//	the flight, where an actively guided stage is being actively guided. But two edge cases need to be
	//	considered: during the final portion of the passive guidance mode, and between two actively guided
	//	stages. In those cases UPFG is being pre-converged, that is: guidance is being calculated for the stage
	//	that is *about to* be activated. The easiest way to understand the logic is by understanding the flags:
	//	- activeGuidanceMode: set by a dedicated event upon activation of the first actively guided stage
	//	- stagingInProgress: set by internalEvent_preStage, means that the current *physical* stage is about to
	//	  be spent OR that the staging procedure for the subsequent stage is in progress; in either case
	//	  upfgState has already been incremented and we're preconverging guidance for the next one;
	//	  this flag is cleared when the proper stage ignites
	//	- prestageHold: set by internalEvent_preStage and cleared by internalEvent_staging, means that the
	//	  current physical stage is about to be spent but the staging procedure has not yet started; this flag
	//	  is mostly used for status display
	//	- poststageHold: conditionally set&cleared by internalEvent_staging (only in case the stage has a
	//	  "postStageEvent" configured), enforces attitude hold until after the post-stage event has been
	//	  executed (e.g. Saturn-like interstage jettison)
	//	- upfgConverged: UPFG has achieved a stable guidance
	//	- upfgEngaged: UPFG has converged and all vehicle status flags permit engaging the guidance; this flag
	//	  can be understood as "nominal active flight mode" and is only set or cleared in this function
	//	Final words regarding the upfgStage variable: this is the index of the "currently guided stage", i.e.
	//	the one for which UPFG is calculating the solution currently. This can mean the "currently flying
	//	stage" if the vehicle is in the nominal flight, but it can also mean the "soon-to-be activated stage"
	//	if the vehicle is about to transition between stages and guidance has to be preconverged for the
	//	subsequent stage. Consult events module for details, particularly spawnStagingEvents and the internal
	//	event handlers.

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

	//	Expects global variables:
	//	"activeGuidanceMode" as bool
	//	"upfgConverged" as bool
	//	"upfgEngaged" as bool
	//	"stagingInProgress" as bool
	//	"steeringVector" as vector
	//	"steeringRoll" as scalar
	//	"liftoffTime" as timespan
	//	"vehicle" as list
	//	"controls" as lexicon
	//	"SETTINGS" as lexicon
	//	Owns global variables:
	//	"usc_lastGoodVector" as vector
	//	"usc_convergeFlags" as list
	//	"usc_lastIterationTime" as scalar
	//	"usc_lastSeenStage" as integer
	//	"usc_currentVehicle" as list
	DECLARE PARAMETER vehicle.		//	Expects a list of lexicon
	DECLARE PARAMETER upfgStage.	//	Expects a scalar
	DECLARE PARAMETER upfgTarget.	//	Expects a lexicon
	DECLARE PARAMETER upfgState.	//	Expects a lexicon
	DECLARE PARAMETER upfgInternal.	//	Expects a lexicon

	//	First run marked by undefined globals
	IF NOT (DEFINED usc_lastGoodVector) {
		GLOBAL usc_lastGoodVector IS V(1,0,0).
		GLOBAL usc_convergeFlags IS LIST().
		GLOBAL usc_lastIterationTime IS upfgState["time"].
		GLOBAL usc_lastSeenStage IS upfgStage.
		GLOBAL usc_currentVehicle IS vehicle:COPY().
	}

	//	Run UPFG
	LOCAL currentIterationTime IS upfgState["time"].
	LOCAL lastTgo IS upfgInternal["tgo"].
	IF usc_lastSeenStage <> upfgStage {
		//	Only copy the vehicle info when the stage changed
		SET usc_currentVehicle TO vehicle:SUBLIST(upfgStage, vehicle:LENGTH).
		SET usc_lastSeenStage TO upfgStage.
	}
	LOCAL upfgOutput IS upfg(usc_currentVehicle, upfgTarget, upfgState, upfgInternal).

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
	IF ABS(expectedTgo-upfgOutput[1]["tgo"]) < SETTINGS["upfgConvergenceCriterion"] {
		IF usc_lastGoodVector <> V(1,0,0) {
			IF VANG(upfgOutput[1]["vector"], usc_lastGoodVector) < SETTINGS["upfgGoodSolutionCriterion"] {
				usc_convergeFlags:ADD(TRUE).
			} ELSE {
				IF NOT stagingInProgress {
					resetUPFG().
				}
			}
		} ELSE {
			usc_convergeFlags:ADD(TRUE).
		}
	} ELSE { usc_convergeFlags:CLEAR(). }
	//	If we have enough consecutive good results - we're converged.
	IF usc_convergeFlags:LENGTH > 2 {
		SET upfgConverged TO TRUE.
		usc_convergeFlags:CLEAR().	//	No need to gather any more flags and make the list grow indefinitely
	}
	//	Check if we can steer
	SET upfgEngaged TO FALSE.	//	If everything is good, it will be overridden right away
	IF activeGuidanceMode {
		IF stagingInProgress OR poststageHold {
			//	Do nothing, maintain constant attidude (the last good steering vector)
		}
		ELSE IF upfgConverged {
			//	Only now we're good to go
			SET steeringVector TO aimAndRoll(vecYZ(upfgOutput[1]["vector"]), steeringRoll).
			SET usc_lastGoodVector TO upfgOutput[1]["vector"].
			SET upfgEngaged TO TRUE.
		}
	} ELSE {
		// This case triggers when we're in the initial pre-convergence mode.
		atmosphericSteeringControl(steeringRoll).	// Defer to atmospheric mode
	}
	RETURN upfgOutput[0].
}

//	Throttle controller
FUNCTION throttleControl {
	//	This handles the constant acceleration throttle control. For constant thrust stages, throttle is set
	//	at stage activation time (see internalEvent_staging_activation).
	//	Expects global variables "vehicle" as list, "upfgStage", "throttleSetting" and "throttleDisplay" as scalars and "stagingInProgress" as bool.

	//	If we're guiding a stage nominally, it's simple. But if we're in between stages, specifically in the
	//	preconvergence mode, we need to look at the *previous* stage data in order to calculate throttle, not
	//	the current (in the upfgStage sense) one.
	//	Extra care must be taken in two situations: if this is the first active stage (avoid a negative index
	//	access), and if we're past the stage burnout time - it is possible that the staging handler purposely
	//	shut down its engines by throttling to 0, so we must not reignite them.
	LOCAL whichStage IS upfgStage.
	IF stagingInProgress {
		IF whichStage = 0 {
			//	The first actively guided stage cannot possibly be in constant acceleration - exit early
			RETURN.
		}
		IF NOT prestageHold {
			//	We are no longer at the end of previous stage, so no need to control throttle - exit early
			RETURN.
		}
		//	Otherwise, this might possibly be a constant acceleration stage near engine cut-off,
		//	but upfgStage has already been incremented:
		SET whichStage TO upfgStage - 1.
	}

	//	No matter which way have we arrived at here (stagingInProgress or not), check for a constant
	//	acceleration stage and handle it accordingly.
	IF vehicle[whichStage]["mode"] = 2 {
		LOCAL nominalThrust_ IS getThrust(vehicle[whichStage]["engines"]).
		LOCAL nominalThrust IS nominalThrust_[0].
		LOCAL throttleLimit IS vehicle[whichStage]["minThrottle"].
		LOCAL desiredThrottle IS SHIP:MASS*1000*vehicle[whichStage]["gLim"]*CONSTANT:g0 / nominalThrust.
		//	Realism Overhaul considers in-game throttle not as absolute, but relative to the allowed throttle range of the engine.
		//	Setting throttle to 0.5 for an engine with throttle range 0.4-1.0 actually results in a 0.7 throttle setting.
		SET throttleSetting TO (desiredThrottle - throttleLimit) / (1 - throttleLimit).
		//	If the algorithm requests a throttle setting lower than minimum limit, we might accidentally shutdown.
		SET throttleSetting TO MAX(throttleSetting, 0.01).
		//	For the GUI printout however, we want to see the final throttle value.
		SET throttleDisplay TO desiredThrottle.
	}
}

//	Return all currently activated engines as a list.
FUNCTION getActiveEngines {
	LOCAL activeEngines IS LIST().
	LIST ENGINES IN allEngines.
	FOR engine IN allEngines {
		IF engine:AVAILABLETHRUST > 0 { activeEngines:ADD(engine). }
	}
	RETURN activeEngines.
}

//	Loss of thrust detection system
FUNCTION thrustWatchdog {
	//	Called regularly, will perform a check whether the engines that are supposed to be burning, are in fact
	//	burning. "Supposed to" means that the check will not be performed prior to liftoff nor during staging
	//	(and shortly after staging, to avoid false loss-of-thrust detection when the engines are merely just
	//	spooling up to full thrust). In case of loss of thrust, mission abort is triggered.
	//	To reduce call overhead, list of active engines is prepared and cached whenever a staging event occurs.
	//	Note: do NOT call this in terminal phase of the flight (i.e. during attitude hold and countdown to Tgo).
	//	Watchdog can be disabled by having "disableThrustWatchdog" set to TRUE in "controls".
	//	Expects global variables:
	//	"controls" as lexicon
	//	"liftOffTime" as timespan
	//	"upfgStage" as integer
	//	"vehicle" as list
	//	Owns global variables: "twb_activeEngines", "twb_waitAfterStaging", "twb_timeOfStaging".

	//	Exit if the watchdog is disabled by the user
	IF controls:HASKEY("disableThrustWatchdog") AND controls["disableThrustWatchdog"] { RETURN. }

	//	Disabled when we're waiting on the launchpad
	IF TIME:SECONDS < liftoffTime:SECONDS { RETURN. }

	//	First real run (cannot happen any earlier because we need to catch the active engines)
	IF NOT (DEFINED twb_activeEngines) {
		GLOBAL twb_activeEngines IS getActiveEngines().
		GLOBAL twb_waitAfterStaging IS FALSE.
		GLOBAL twb_timeOfStaging IS 0.
	}

	//	Don't check if we're between stages and it's okay to not have thrust
	IF stagingInProgress AND (NOT vehicle[upfgStage]["isVirtualStage"]) AND (NOT vehicle[upfgStage]["isSustainer"]) {
		SET twb_waitAfterStaging TO TRUE.
		SET twb_timeOfStaging TO TIME:SECONDS.
		RETURN.
	}

	//	If we just exited from staging, wait a second or two to let the engines spool up *and then* cache them
	IF twb_waitAfterStaging {
		IF TIME:SECONDS < twb_timeOfStaging + 2 { RETURN. }
		SET twb_waitAfterStaging TO FALSE.
		SET twb_activeEngines TO getActiveEngines().
	}

	//	Check if we've got thrust
	LOCAL sumThrust IS 0.
	FOR engine IN twb_activeEngines {
		SET sumThrust TO sumThrust + engine:THRUST.
	}

	//	We're comparing floats so better to use an epsilon... nobody will be flying an ion engine, right?
	IF sumThrust < 0.001 {
		pushUIMessage("LOSS OF THRUST DETECTED. ABORTING!", 10, PRIORITY_CRITICAL).
		TOGGLE ABORT.
	}
}

//	Secondary MECO criterion
FUNCTION angularMomentumWatchdog {
	//	Usually (main loop in pegas.ks) we determine the final MECO from Tgo reported by UPFG (upfgInternal).
	//	However, in certain edge cases, target orbit might be achieved before we transition into burn finalization.
	//	To safeguard against this, we will periodically check vehicle's specific orbital angular momentum
	//	and compare against the expected value computed from target orbit parameters.
	//	Expects global variables:
	//	"target" as lexicon
	//	Owns global variables: "amwd_target".
	//	Returns TRUE if we should MECO, i.e. orbital angular momentum target is achieved, FALSE otherwise.

	//	First run: set up the angular momentum target value
	IF NOT (DEFINED amwd_target) {
		LOCAL _target_ap IS SHIP:ORBIT:BODY:RADIUS + 1000 * mission["apoapsis"].
		LOCAL _target_pe IS SHIP:ORBIT:BODY:RADIUS + 1000 * mission["periapsis"].
		LOCAL _target_sma IS (_target_ap + _target_pe) / 2.
		LOCAL _target_ecc IS (_target_ap - _target_pe) / (_target_ap + _target_pe).
		GLOBAL amwd_target IS SQRT(_target_sma * (1 - _target_ecc ^ 2)).
	}

	//	Don't check if we're between stages (angular momentum should not change during staging, but we will wait
	//	just in case, to avoid messing up with the procedure).
	IF stagingInProgress { RETURN. }

	//	Compute the current angular momentum
	LOCAL current_am IS SQRT(SHIP:ORBIT:SEMIMAJORAXIS * (1 - SHIP:ORBIT:ECCENTRICITY ^ 2)).

	RETURN current_am >= amwd_target.
}
