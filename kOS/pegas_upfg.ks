//	Unified Powered Flight Guidance

FUNCTION upfg {
	DECLARE PARAMETER vehicle.
	DECLARE PARAMETER target.
	DECLARE PARAMETER state.
	DECLARE PARAMETER previous.

	LOCAL gamma IS target["angle"].
	LOCAL iy IS target["normal"].
	LOCAL rdval IS target["radius"].
	LOCAL vdval IS target["velocity"].
	LOCAL t IS state["time"].
	LOCAL m IS state["mass"].
	LOCAL r IS state["radius"].
	LOCAL v IS state["velocity"].
	LOCAL cser IS previous["cser"].
	LOCAL rbias IS previous["rbias"].
	LOCAL rd IS previous["rd"].
	LOCAL rgrav IS previous["rgrav"].
	LOCAL tp IS previous["time"].
	LOCAL vprev IS previous["v"].
	LOCAL vgo IS previous["vgo"].

	//	1
	LOCAL n IS vehicle:LENGTH.
	LOCAL SM IS LIST().
	LOCAL aL IS LIST().
	LOCAL md IS LIST().
	LOCAL ve IS LIST().
	LOCAL fT IS LIST().
	LOCAL aT IS LIST().
	LOCAL tu IS LIST().
	LOCAL tb IS LIST().
  
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		SM:ADD(vehicle[i]["mode"]).
		aL:ADD(vehicle[i]["gLim"]*g0).
		LOCAL pack IS getThrust(vehicle[i]["engines"]).
		fT:ADD(pack[0]).
		md:ADD(pack[1]).
		ve:ADD(pack[2]*g0).
		aT:ADD(fT[i] / vehicle[i]["massTotal"]).
		tu:ADD(ve[i]/aT[i]).
		tb:ADD(vehicle[i]["maxT"]).
	}
	
	//	2
	LOCAL dt IS t-tp.
	LOCAL dvsensed IS v-vprev.
	LOCAL vgo IS vgo-dvsensed.
	SET tb[0] TO tb[0] - previous["tb"].
	
	//	3
	IF SM[0]=1 {
		SET aT[0] TO fT[0] / m.
	} ELSE IF SM[0]=2 {
		SET aT[0] TO aL[0].
	}
	SET tu[0] TO ve[0] / aT[0].
	LOCAL L IS 0.
	LOCAL Li IS LIST().
	
	FROM { LOCAL i IS 0. } UNTIL i>=n-1 STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			Li:ADD( ve[i]*LN(tu[i]/(tu[i]-tb[i])) ).
		} ELSE IF SM[i]=2 {
			Li:ADD( aL[i]*tb[i] ).
		} ELSE Li:ADD( 0 ).
		SET L TO L + Li[i].
		IF L>vgo:MAG {
			RETURN upfg(vehicle:SUBLIST(0,vehicle:LENGTH-1), target, state, previous).
		}
	}
	Li:ADD(vgo:MAG - L).
	
	LOCAL tgoi IS LIST().
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			SET tb[i] TO tu[i] * (1-CONSTANT:E^(-Li[i]/ve[i])).
		} ELSE IF SM[i]=2 {
			SET tb[i] TO Li[i] / aL[i].
		}
		IF i=0 {
			tgoi:ADD(tb[i]).
		} ELSE {
			tgoi:ADD(tgoi[i-1] + tb[i]).
		}
	}
	
	LOCAL L1 IS Li[0].
	LOCAL tgo IS tgoi[n-1].
	
	//	4
	SET L TO 0.
	LOCAL J IS 0.
	LOCAL S IS 0.
	LOCAL Q IS 0.
	LOCAL H IS 0.
	LOCAL P IS 0.
	LOCAL Ji IS LIST().
	LOCAL Si IS LIST().
	LOCAL Qi IS LIST().
	LOCAL Pi IS LIST().
	LOCAL tgoi1 IS 0.
	
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF i>0 {
			SET tgoi1 TO tgoi[i-1].
		}
		IF SM[i]=1 {
			Ji:ADD( tu[i]*Li[i] - ve[i]*tb[i] ).
			Si:ADD( -Ji[i] + tb[i]*Li[i] ).
			Qi:ADD( Si[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 ).
			Pi:ADD( Qi[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 * (tb[i]/3+tgoi1) ).
		} ELSE IF SM[i]=2 {
			Ji:ADD( 0.5*Li[i]*tb[i] ).
			Si:ADD( Ji[i] ).
			Qi:ADD( Si[i]*(tb[i]/3+tgoi1) ).
			Pi:ADD( (1/6)*Si[i]*(tgoi[i]^2 + 2*tgoi[i]*tgoi1 + 3*tgoi1^2) ).
		}
		
		SET Ji[i] TO Ji[i] + Li[i]*tgoi1.
		SET Si[i] TO Si[i] + L*tb[i].
		SET Qi[i] TO Qi[i] + J*tb[i].
		SET Pi[i] TO Pi[i] + H*tb[i].
		
		SET L TO L+Li[i].
		SET J TO J+Ji[i].
		SET S TO S+Si[i].
		SET Q TO Q+Qi[i].
		SET P TO P+Pi[i].
		SET H TO J*tgoi[i] - Q.
	}
	
	//	5
	LOCAL lambda IS vgo:NORMALIZED.
	IF previous["tgo"]>0 {
		SET rgrav TO (tgo/previous["tgo"])^2 * rgrav.
	}
	LOCAL rgo IS rd - (r + v*tgo + rgrav).
	LOCAL iz IS VCRS(rd,iy):NORMALIZED.
	LOCAL rgoxy IS rgo - VDOT(iz,rgo)*iz.
	LOCAL rgoz IS (S - VDOT(lambda,rgoxy)) / VDOT(lambda,iz).
	SET rgo TO rgoxy + rgoz*iz + rbias.
	LOCAL lambdade IS Q - S*J/L.
	LOCAL lambdadot IS (rgo - S*lambda) / lambdade.
	LOCAL iF_ IS lambda - lambdadot*J/L.
	SET iF_ TO iF_:NORMALIZED.
	LOCAL phi IS VANG(iF_,lambda)*CONSTANT:DEGTORAD.
	LOCAL phidot IS -phi*L/J.
	LOCAL vthrust IS L - 0.5*L*phi^2 - J*phi*phidot - 0.5*H*phidot^2.
	SET vthrust TO vthrust*lambda - (L*phi + J*phidot)*lambdadot:NORMALIZED.
	LOCAL rthrust IS S - 0.5*S*phi^2 - Q*phi*phidot - 0.5*P*phidot^2.
	SET rthrust TO rthrust*lambda - (S*phi + Q*phidot)*lambdadot:NORMALIZED.
	SET vbias TO vgo - vthrust.
	SET rbias TO rgo - rthrust.
	
	//	6
	//	TODO: angle rates
	LOCAL _up IS r:NORMALIZED.
	LOCAL _east IS VCRS(V(0,0,1),_up):NORMALIZED.
	LOCAL pitch IS VANG(iF_,_up).
	LOCAL inplane IS VXCL(_up,iF_).
	LOCAL yaw IS VANG(inplane,_east).
	LOCAL tangent IS VCRS(_up,_east).
	IF VDOT(inplane,tangent)<0 {
		SET yaw TO -yaw.
	}
	
	//	7
	LOCAL rc1 IS r - 0.1*rthrust - (tgo/30)*vthrust.
	LOCAL vc1 IS v + 1.2*rthrust/tgo - 0.1*vthrust.
	LOCAL pack IS cse(rc1, vc1, tgo, cser).
	SET cser TO pack[2].
	SET rgrav TO pack[0] - rc1 - vc1*tgo.
	LOCAL vgrav IS pack[1] - vc1.
	
	//	8
	LOCAL rp IS r + v*tgo + rgrav + rthrust.
	SET rp TO rp - VDOT(rp,iy)*iy.
	LOCAL rd IS rdval*rp:NORMALIZED.
	SET ix TO rd:NORMALIZED.
	SET iz TO VCRS(ix,iy).
	//	emulate matrix-vector multiplication
	LOCAL vv1 IS V(ix:X, iy:X, iZ:X).
	LOCAL vv2 IS V(ix:Y, iy:Y, iZ:Y).
	LOCAL vv3 IS V(ix:Z, iy:Z, iZ:Z).
	LOCAL vop IS V(SIN(gamma), 0, COS(gamma)).
	LOCAL vd IS V(VDOT(vv1,vop), VDOT(vv2,vop), VDOT(vv3,vop))*vdval.
	SET vgo TO vd - v - vgrav + vbias.
	
	//	RETURN - build new internal state instead of overwriting the old one
	LOCAL current IS LEXICON(
		"cser", cser,
		"rbias", rbias,
		"rd", rd,
		"rgrav", rgrav,
		"tb", previous["tb"] + dt,
		"time", t,
		"tgo", tgo,
		"v", v,
		"vgo", vgo
	).
	LOCAL guidance IS LEXICON(
		"vector", iF_,
		"pitch", pitch,
		"yaw", yaw,
		"pitchdot", 0,
		"yawdot", 0,
		"tgo", tgo
	).
	RETURN LIST(current, guidance, dt).
}.
