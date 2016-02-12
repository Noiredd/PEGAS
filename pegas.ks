//	pegas.ks
//	PEGAS - Powered Explicit Guidance Ascent System

//	This program should be called by a loader that has copied the necessary libraries and was itself called by a proper boot file which
//	has created and initialized the following global variables:
//		P_vName	STRING			vehicle name
//		P_seq	LIST of FLOAT	time table for a flight sequence, with following fields:
//								0	ignition (suggested a negative number - represents time for engines to reach full thrust before releasing)
//										important: ignition and clamp release HAVE to be two separate STAGE events! (maybe todo: staging standards? LEXICON?)
//								1	booster jettison (moment of jettisoning any side-mounted boosters, or 0 if none are present)
//								2	fairings jettison (as with boosters)
//								3	first stage cutoff
//								4	first stage separation
//								5	ullage motor ignition
//								6	second stage ignition
//								7	PEG activation (time for ullage motors to burn out and main engine to reach full thrust)
//								8	second stage max burn time (NOT related to T=0, rather to stage ignition time)
//		P_pt	LIST of FLOAT	time part of a first stage control program
//		P_pp	LIST of FLOAT	pitch part of a first stage control program
//		P_umode	INT				ullage motor for second stage: 0 - via stageable solid motors, >0 - via RCS
//								in case of RCS ullage, this also states for how long should the thrusters fire

//	INITIALIZATION
//	Safety check, thanks to 0.18 update
IF NOT( DEFINED P_vName AND DEFINED P_seq AND DEFINED P_pt AND DEFINED P_pp AND DEFINED P_umode ) {
	PRINT "No proper vehicle boot file was loaded. Aborting.".
	SHUTDOWN.
}
//	Load libraries
RUN pegas_nav.
RUN pegas_lib.
//	Load settings
RUN pegas_set.

//	TODO: Nice splashscreen before launch maybe? Not before a full 3D case is solved and precise orbit (Ap, Pe, inclination) targeting is possible.
CLEARSCREEN.

//	COUNTDOWN
IF S_countdown < ABS(P_seq[0]) { SET S_countdown TO CEILING(ABS(P_seq[0])). }.
//	Prepare events list for the countdown
LOCAL cdEvents IS LIST().	//	countdown events (new countdown message every second, engine ignition)
LOCAL cdIgnite IS 0.		//	indicates which element of the list corresponds to ignition, not messages
FROM { LOCAL i IS S_countdown. } UNTIL i < 0 STEP { SET i TO i-1. } DO {
	cdEvents:ADD(i).
	IF CEILING(ABS(P_seq[0])) = i {
		cdEvents:ADD(ABS(P_seq[0])).	//	add engine ignition to event list and mark its position in the countdown sequence
		SET cdIgnite TO S_countdown-i+1.
	}.
}.
//	Countdown loop
LOCAL cdT0 IS TIME:SECONDS + S_countdown + 0.2.	//	+0.2 because we don't really worry about release timing accuracy... yet
LOCAL cdTN IS TIME:SECONDS.
LOCAL cdId IS 0.		//	current position in the countdown events list
UNTIL FALSE {
	//	check how much time to countdown end
	SET cdTN TO cdT0 - TIME:SECONDS.
	//	if we're past the timestamp of the current event
	IF cdTN < cdEvents[cdId] {
		//	if we're at the ignition event - stage
		IF cdId = cdIgnite {
			PRINT "IGNITION!".
			LOCK THROTTLE TO 1.
			STAGE.	//	Be aware that this should not be allowed to happen too quickly, as the stage might not be ready right after loading the vehicle and physics
		}
		//	else means to just print a message
		ELSE {
			CLEARSCREEN.
			PRINT "Countdown... " + cdEvents[cdId].
		}.
		//	increment position on the events list
		SET cdId TO cdId+1.
		//	and if we're out of events - exit the loop
		IF cdId = cdEvents:LENGTH { BREAK. }.
	}.
	WAIT 0.
}.
//	Wow that was a rather large piece of code for just a countdown with pre-launch ignition... gotta figure out a nicer way to do that.

//	LAUNCH
//	Prepare log file name (always, no matter the setting - this is not equal to creating the file)
LOCAL P_logfn IS P_vName + ROUND(TIME:SECONDS) + ".log".
//	Make sure we can stage first (KSP introduces a short "cooldown" period between two stagings)
UNTIL STAGE:READY {
	PRINT "WARNING: STAGE WAS NOT READY!".
	WAIT 0.
}.
STAGE.
//	Take flight control immediately
GLOBAL P_t0 IS TIME:SECONDS.//	T=0
GLOBAL P_time IS 0.			//	current time (T+)
GLOBAL P_pitch IS 90.		//	current pitch command
GLOBAL P_steer IS LOOKDIRUP(HEADING(90,P_pitch):VECTOR, SHIP:FACING:TOPVECTOR).
SAS OFF.
LOCK STEERING TO P_steer.	//	Locking to a vector instead of expression, because kOS might not necessarily choose the closest route
							//	to do the rotation, which means ship's FACING vector would continuously change during rotation. If the
							//	steering target was recalculated on every physics step, these small changes could potentially accumulate.
addMessage("LIFTOFF!").		//	Instead of printing

//	OPEN LOOP PITCH CONTROL
//	Define state variables
GLOBAL P_alt IS 0.			//	current altitude as measured from Earth's centre
GLOBAL P_vy IS 0.			//	current vertical velocity
GLOBAL P_vx IS 0.			//	current horizontal velocity
GLOBAL P_angle IS 0.		//	current flight angle
GLOBAL P_acc IS 0.			//	current acceleration (m/s^2)
GLOBAL P_ve IS 0.			//	current exhaust velocity (Isp*g0)
GLOBAL P_engines IS LIST().	//	list of all engines on board - could as well be refined to only active engines, but this is not necessary
LOCAL F_boosters IS 0.		//	boosters have not been jettisoned
LOCAL F_fairings IS 0.		//	neither were fairings
LIST ENGINES IN P_engines.	//	only update this after staging events
							//	TODO: maybe implement some better handling of engines than "list all of them and see which ones have fire coming out of the nozzle"
//	Run main loop
UNTIL FALSE {
	SET P_time TO TIME:SECONDS - P_t0.
	getState(P_engines).	//	update state variables
	//	recalculate steering vector
	SET P_pitch TO 90-getPitch(MIN(P_time,P_seq[3]-S_krTime)).	//	stop pitching at the last moments before cutoff - rocket must not rotate for safe separation
	SET P_steer TO LOOKDIRUP(HEADING(90,P_pitch):VECTOR, SHIP:FACING:TOPVECTOR).	//	TODO: FIXED AZIMUTH MUST BE MOVED TO SETTINGS
	//	data outputs
	dataViz().
	IF S_logging = 1 { dataLog(P_logfn). }.
	//	sequence control: should the boosters be jettisoned?
	IF P_seq[1]>0 {
		//	the following assumes boosters will not be jettisoned milliseconds into the flight, and the stage will be ready in any reallistic scenario
		IF P_time>P_seq[1] AND F_boosters = 0 {
			addMessage("BOOSTERS JETTISONED!").
			SET F_boosters TO 1.
			STAGE.
		}.
	}.
	//	sequence control: should the fairings be jettisoned?
	IF P_seq[2]>0 {
		IF P_time>P_seq[2] AND F_fairings = 0 {
			addMessage("FAIRINGS JETTISONED!").
			SET F_fairings TO 1.
			STAGE.
		}.
	}.
	//	TODO: consider replacing constant checks with a single WHEN for both events
	//	exit check
	IF P_time>P_seq[3] BREAK.
	WAIT 0.2.
}.
addMessage("CUTOFF!").
dataViz().	//	force display message
WAIT 0.

//	STAGING
//	Separation
UNTIL FALSE {
	SET P_time TO TIME:SECONDS - P_t0.
	//	the following assumes enough time has been given the stage to become ready (no check for readiness occurs)
	IF P_time>P_seq[4] {
		addMessage("SEPARATING STAGE 1!").
		dataViz().
		STAGE.
		BREAK.
	}.
	WAIT 0.
}.
//	Ullage
UNTIL FALSE {
	SET P_time TO TIME:SECONDS - P_t0.
	//	Wait until the right time
	IF P_time>P_seq[5] {
		IF P_umode = 0 {
			//	This handles SRB ullage
			UNTIL STAGE:READY {
				//	It's bad if this executes
				PRINT "(waiting for ullage SRMs)".
				WAIT 0.
			}.
			addMessage("IGNITING ULLAGE BOOSTERS!").
			dataViz().
			STAGE.
			BREAK.
		} ELSE IF P_umode > 0 {
			//	This handles RCS ullage
			addMessage("IGNITING ULLAGE THRUSTERS!").
			dataViz().
			RCS ON.
			SET SHIP:CONTROL:FORE TO 0.9.	//	Not full throttle, to allow some room for attitude-keeping during long RCS push
			//	Set a trigger to disable thrusters
			WHEN P_time>P_seq[5]+P_umode THEN { SET SHIP:CONTROL:FORE TO 0.0. RCS OFF. }.
			BREAK.
		}.
	}.
	WAIT 0.
}.
//	Ignition
UNTIL FALSE {
	SET P_time TO TIME:SECONDS - P_t0.
	//	Wait...
	IF P_time>P_seq[6] {
		//	...wait...
		UNTIL STAGE:READY {
			PRINT "(waiting for main engine)".
			WAIT 0.
		}.
		//	...fire!
		addMessage("STAGE 2 MAIN ENGINE IGNITION!").
		dataViz().
		STAGE.
		BREAK.
	}.
	WAIT 0.
}.

//	POWERED EXPLICIT GUIDANCE
//	Let the engine "spool up", as declared in vehicle-specific sequence table.
UNTIL P_time>P_seq[7] {
	SET P_time TO TIME:SECONDS - P_t0.
	WAIT 0.
}.
//	Prepare variables
GLOBAL P_target IS SHIP:OBT:BODY:RADIUS + S_target*1000.
GLOBAL P_mu IS SHIP:OBT:BODY:MU.
LIST ENGINES IN P_engines.		//	refresh the list after jettisoning first stage
getState(P_engines).			//	get fresh vehicle state after staging sequence
GLOBAL A IS 0.
GLOBAL B IS 0.
GLOBAL C IS 0.
GLOBAL T IS P_seq[8].			//	initialize time to cutoff guess from maximum stage burn time (nice first guess for PEG init)
GLOBAL P_tpeg IS P_time.		//	this will hold the time of last PEG calculation
GLOBAL P_converged IS -3.		//	guidance has converged flag
//	PEG init
peg(0, P_alt, P_vx, P_vy, P_target, P_acc, P_ve, P_mu, A, B, T).
WAIT 0.
//	Main loop
UNTIL FALSE {
	SET P_time TO TIME:SECONDS - P_t0.
	getState(P_engines).
	LOCAL last IS P_time-P_tpeg.//	time since last PEG calculation
	//	only run PEG every S_pegCycle seconds
	IF last>=S_pegCycle {
		LOCAL p IS peg(S_pegCycle, P_alt, P_vx, P_vy, P_target, P_acc, P_ve, P_mu, A, B, T).
		//	Convergence check: on ideal guidance, time to cutoff should decline every cycle by S_pegCycle
		//	if a new T estimate differs from the expected one by less than 2% we consider guidance as converged
		//	Remember that when the calculation occurs, time is already T-cycle, so the next value should be one more cycle shorter!
		IF ABS( (T-2*S_pegCycle)/p[3] - 1 ) < 2/100 {
			//	we need at least 3 converged samples before we declare full convergence
			IF P_converged < 0 {
				SET P_converged TO P_converged + 1.
			} ELSE IF P_converged = 0 {
				addMessage("GUIDANCE HAS CONVERGED!").
				SET P_converged TO 1.
			}.
		}.
		SET A TO p[0].
		SET B TO p[1].
		SET C TO p[2].
		SET T TO p[3].
		SET P_tpeg TO P_time.
		SET last TO 0.	//	TODO: hmmm, what if PEG was run but already near the cutoff and didn't recalculate A&B?
	}.
	//	translate output to pitch control setpoint
	SET P_pitch TO A - B*last + C.	//	yes, all docs have a '+' sign here - this seems to work much better though (proven by flight data log analysis)
	SET P_pitch TO MAX(-1, MIN(P_pitch, 1)).	//	clamp result into arcus sine domain (though it's VERY bad if this is ever needed)
	SET P_pitch TO ARCSIN(P_pitch).
	IF P_converged = 1 {
		//	First few outputs from PEG (before convergence) are usually weird.
		//	It's better to just follow the old vector, so we don't risk rotating the craft to some extreme attitude.
		SET P_steer TO LOOKDIRUP(HEADING(90,P_pitch):VECTOR, SHIP:FACING:TOPVECTOR).
	}.
	//	data output
	dataViz().
	IF S_logging = 1 { dataLog(P_logfn). }.
	//	PEG-controlled cutoff: T-last = time from now to cutoff; if this is smaller than some arbitrary nonzero value - cut off
	//	in the future, this might account for engines that do not shut down immediately (and hence produce some additional delta-v
	//	after they're supposed to be off, which results in a predictable imperfection of the target orbit)
	IF (T-last < 0.1) {
		BREAK.	//	all the work is handled outside the function
	}.
	WAIT 0.
}.
//	Cutoff
addMessage("MAIN ENGINE CUTOFF!").
dataViz().
FOR e IN P_engines {
	IF e:IGNITION AND e:ALLOWSHUTDOWN
		e:SHUTDOWN.
}.
UNLOCK THROTTLE.
UNLOCK STEERING.
SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
WAIT 0.

//	SUMMARY
//	TODO: Add a little wait here, in case for those slow-shutting-off engines
getState(LIST()).	//	call on empty list, no engine is active anyway
dataViz().
PRINT "Time taken: " + ROUND(P_time,1) + " seconds".
PRINT "Orbit established: " + ROUND(APOAPSIS/1000,1) + "x" + ROUND(PERIAPSIS/1000,1) + "km".
PRINT "Orbit targeted: " + S_target + "km".
PRINT "Apsis error: " + ROUND(APOAPSIS/10/S_target-100,2) + "%, " + ROUND(PERIAPSIS/10/S_target-100,2) + "%".