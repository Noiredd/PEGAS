//	pegas_lib.ks
//	All the necessary functions to run launch and guidance.
//	Variable names with a prefix "P_" are assumed to be declared GLOBAL.

//	addMessage	pushes a new message for temporary display in dataViz
//				requires passing a (presumed non-empty) parameter of type STRING
FUNCTION addMessage {
	DECLARE PARAMETER msg.
	
	//	Mechanism for this is as follows: a new message is defined as a global variable,
	//	and its Time To Live is declared in another. Function dataViz looks for these variables
	//	and displays the message if current flight time is not greater than the TTL.
	IF NOT( DEFINED P_vizMsg ) GLOBAL P_vizMsg IS "".
	IF NOT( DEFINED P_vizTTL ) GLOBAL P_vizTTL IS 0.
	//	Every new message overrides the previous one.
	SET P_vizMsg TO msg.
	SET P_vizTTL TO P_time + S_msgTTL.	//	time to live stored in settings
}.

//	dataViz		outputs all flight information to the screen
//				requires flight sequence given by P_seq defined as GLOBAL and of type LIST
//				capable of displaying custom messages, set up by addMessage as a GLOBAL STRING with a FLOAT time to live
FUNCTION dataViz {
	CLEARSCREEN.
	PRINT "PEGAS - Powered Explicit Guidance Ascent System".
	PRINT " ".
	PRINT P_vName.
	PRINT " ".
	PRINT "T + " + ROUND(P_time,1) + "s".
	PRINT " ".
	PRINT "Target:   " + S_target + "km".
	PRINT "Altitude: " + ROUND((P_alt-SHIP:OBT:BODY:RADIUS)/1000,1) + "km".
	PRINT "Apoapsis: " + ROUND(APOAPSIS/1000,1) + "km".
	PRINT " ".
	PRINT "Velocity: " + ROUND(SQRT(P_vy^2+P_vx^2),1) + "m/s".
	PRINT "      Vy: " + ROUND(P_vy,1) + "m/s".
	PRINT "      Vx: " + ROUND(P_vx,1) + "m/s".
	PRINT "   Angle: " + ROUND(P_angle,1) + "deg".
	PRINT " ".
	PRINT "Setpitch: " + ROUND(P_pitch,1) + "deg".
	PRINT " ".
	PRINT "Acceler.: " + ROUND(P_acc,2) + "m/s^2".
	PRINT "     TWR: " + ROUND(P_acc/9.80665,2).
	PRINT " ".
	//	Custom message section
	IF DEFINED P_vizMsg {
		IF P_time < P_vizTTL {
			//	display only if its still alive
			PRINT "STATUS:   " + P_vizMsg.
			RETURN.
		}.
	}.
	//	If message was displayed, function has already ended.
	//	Otherwise, phase-specific status will be printed
	IF P_time < P_seq[3] {
		//	Phase: first stage (open loop control)
		PRINT "STATUS:   " + "First stage cutoff in " + ROUND(P_seq[3]-P_time,1) + " seconds.".
	} ELSE IF P_time > P_seq[6] {
		//	Phase: second stage (closed loop control)
		PRINT "STATUS:   " + "Second stage cutoff in " + ROUND(P_tpeg+T-P_time,1) + " seconds.".
	}.
}.

//	dataLog		outputs all flight information to the log file
//				requires passing a filename parameter of type STRING
FUNCTION dataLog {
	DECLARE PARAMETER filename.
	
	//	Start file with a header line.
	IF NOT( DEFINED P_dataLogHeaderFlag) {
		GLOBAL P_dataLogHeaderFlag IS 1.
		LOG "Time	Altitude	Vy	Vx	Angle(flt)	Acceleration	Pitch(cmd)" TO filename.
	}.
	
	LOG P_time + " " + P_alt + " " + P_vy + " " + P_vx + " " + P_angle + " " + P_acc + " " + P_pitch + " " TO filename.
}.

//	getPitch	calculate pitch setpoint for a given time t and pitch program P_pt, P_pp
//				requires program given by P_pt and P_pp defined as GLOBAL and of type LIST
FUNCTION getPitch {
	DECLARE PARAMETER t.
	
	LOCAL n IS P_pt:LENGTH.
	LOCAL k IS 0.
	
	//	Finds where on the pitch program time table are we now.
	FROM { LOCAL i IS 0. } UNTIL i = n STEP { SET i TO i+1. } DO {
		IF t < P_pt[i] { SET k TO i. BREAK. }.
	}
	IF k = 0 RETURN P_pp[n-1].
	
	//	Linear interpolation from two values we're between.
	LOCAL m IS (P_pp[k]-P_pp[k-1])/(P_pt[k]-P_pt[k-1]).
	LOCAL b IS P_pp[k]-m*P_pt[k].
	
	RETURN m*t+b.
}.

//	getState	obtains current vehicle state
//				contains no engine identification logic, requiring explicitly passing a list of those
//				outputs no data directly - GLOBAL variables are assumed to have been declared
FUNCTION getState {
	DECLARE PARAMETER engines.	//	crucial: must be of type LIST
	
	SET P_alt TO SHIP:OBT:BODY:DISTANCE.
	SET P_vx TO SQRT(SHIP:VELOCITY:ORBIT:SQRMAGNITUDE - SHIP:VERTICALSPEED^2).
	SET P_vy TO SHIP:VERTICALSPEED.
	SET P_angle TO ARCTAN2(P_vx, P_vy).
	//	TODO: fix the above. Vx should be surface-related velocity for atmospheric flight (1st stage) and
	//	TODO: orbital velocity for PEG stage. Same goes for flight angle. Rework this after a 3D case
	LOCAL e_thr IS 0.
	LOCAL e_isp IS 0.
	FOR e IN engines {
		//	sum specific impulses of all active engines:
		//	Isp_sum = sum(Isp_i*thrust_i) / sum(thrust_i)
		//	note that the following does not do division, only two sums
		IF e:IGNITION {
			SET e_thr TO e_thr + e:THRUST.
			SET e_isp TO e_isp + e:ISP*e:THRUST.
		}.
	}.
	SET P_acc TO e_thr/SHIP:MASS.
	
	//	During normal operation the following check should never be needed. It cannot be removed
	//	though, as the function is called one last time after MECO, for a summary.
	IF e_thr = 0 { SET P_ve TO 0. }
	ELSE { SET P_ve TO e_isp*9.80665/e_thr. }.
}.

//	msolve		simple solver for a particular equation system, as encountered in PEG
//				simplified even more - only works for circular target orbit with no yaw control
//				returns steering constants A and B in the form of a list
FUNCTION msolve {
	DECLARE PARAMETER ve.	//	exhaust velocity - constant
	DECLARE PARAMETER tau.	//	maximum hypothetical time of burn (as if the whole vehicle mass was propellant)
	DECLARE PARAMETER oldT.	//	old burn time estimate
	DECLARE PARAMETER vy.	//	current vertical (radial) velocity
	DECLARE PARAMETER gain.	//	altitude to gain (difference between target altitude and current)

	LOCAL b0 IS -ve * LN(1 - oldT/tau).
	LOCAL b1 IS b0*tau - ve*oldT.
	LOCAL c0 IS b0*oldT - b1.
	LOCAL c1 IS c0*tau - ve*oldT^2/2.
	LOCAL z0 IS -vy.
	LOCAL z1 IS gain - vy*oldT.

	LOCAL B IS (z1/c0 - z0/b0) / (c1/c0 - b1/b0).
	LOCAL A IS (z0 - b1*B) / b0.

	RETURN LIST(A,B).		//	steering constants A and B; beware of calling this too close to target - results might diverge
}.
	
//	peg			Powered Explicit Guidance routine
//				requires function msolve for A and B calculation
//				outputs steering constants A, B and C and time to cutoff T in the form of a list
//				passing oldA==oldB==0 causes them to be estimated from scratch (default use: initial run)
FUNCTION peg {
	DECLARE PARAMETER cycle.//	length of the major cycle (time between PEG calculations)
	DECLARE PARAMETER h.	//	current altitude (metres from the body center, not surface)
	DECLARE PARAMETER vx.	//	current horizontal (tangential) velocity
	DECLARE PARAMETER vy.	//	current vertical (radial) velocity
	DECLARE PARAMETER tgt.	//	target altitude (formatted like h)
	DECLARE PARAMETER a0.	//	current vehicle acceleration
	DECLARE PARAMETER ve.	//	exhaust velocity
	DECLARE PARAMETER mu.	//	standard gravitational parameter GM of the orbited body
	DECLARE PARAMETER oldA.	//	previous values of A, B, T
	DECLARE PARAMETER oldB.
	DECLARE PARAMETER oldT.

	LOCAL tau IS ve/a0.
	LOCAL A IS 0.
	LOCAL B IS 0.
	LOCAL C IS 0.
	LOCAL T IS 0.

	//	estimate A and B from oldT if both passed as 0
	IF oldA = 0 AND oldB = 0 {
		LOCAL ab IS msolve(ve, tau, oldT, vy, tgt-h).
		SET oldA TO ab[0].
		SET oldB TO ab[1].
	}

	//	current and target angular momentum
	LOCAL angM IS VCRS(V(h,0,0), V(vy,vx,0)):MAG.
	LOCAL tgtV IS SQRT(mu/tgt).
	LOCAL tgtM IS VCRS(V(tgt,0,0), V(0,tgtV,0)):MAG.
	LOCAL dMom IS tgtM-angM.

	//	steering constant series f_r
	SET C TO (mu/tgt^2 - tgtV^2/tgt) / (a0 / (1-oldT/tau)).
	LOCAL frt IS oldA + oldB*oldT + C.
	SET C TO (mu/h^2 - vx^2/h) / a0.
	LOCAL fr IS oldA + C.
	LOCAL frdot IS (frt-fr)/oldT.

	//	steering constant series f_theta
	LOCAL ft IS 1 - fr^2/2.
	LOCAL ftdot IS -fr*frdot.
	LOCAL ftdd IS -frdot^2/2.

	//	deltaV and updated T
	LOCAL avgR IS (h+tgt)/2.
	LOCAL dv IS dMom/avgR + ve*(oldT-cycle)*(ftdot+ftdd*tau) + ftdd*ve*(oldT-cycle)^2/2.
	SET dv TO dv / (ft + ftdot*tau + ftdd*tau^2).
	SET T TO tau*(1 - CONSTANT():E ^ (-dv/ve)).

	//	calculate A and B
	//	doing that while too close to target values might cause the matrix solver to diverge,
	//	so we restrict execution of that section
	IF T>=S_pegEpsilon {
		LOCAL ab IS msolve(ve, tau, oldT, vy, tgt-h).
		SET A TO ab[0].
		SET B TO ab[1].
	} ELSE {
		SET A TO oldA.
		SET B TO oldB.
	}
	
	//	formal list-type return
	RETURN LIST(A,B,C,T).
}.