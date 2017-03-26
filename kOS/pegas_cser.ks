FUNCTION uss {
	DECLARE PARAMETER xarg.
	DECLARE PARAMETER a.
	DECLARE PARAMETER kmax.
	
	LOCAL du1 IS xarg/4.
	LOCAL u1 IS du1.
	LOCAL u1old IS 0.
	LOCAL f7 IS -a * du1^2.
	LOCAL k IS 3.
	
	UNTIL NOT (k<kmax) {
		SET du1 TO f7*du1 / (k*(k-1)).
		SET u1old TO u1.
		SET u1 TO u1+du1.
		IF u1=u1old { BREAK. }
		SET k TO k+2.
	}
	
	RETURN u1.
}.

FUNCTION qcf {
	DECLARE PARAMETER w.
	LOCAL xq IS 0.
	
	IF w<1 {
		SET xq TO 21.04 - 13.04*w.
	} ELSE IF w<4.625 {
		SET xq TO (5/3) * (2*w+5).
	} ELSE IF w<13.846 {
		SET xq TO (10/7) * (w+12).
	} ELSE IF w<44 {
		SET xq TO 0.5 * (w+60).
	} ELSE IF w<100 {
		SET xq TO 0.25 * (w+164).
	} ELSE SET xq TO 70.
	
	LOCAL b IS 0.
	LOCAL y IS (w-1)/(w+1).
	LOCAL j IS FLOOR(xq).
	LOCAL b IS y/(1+(j-1)/(j+2)*(1-b)).
	UNTIL NOT (j>2) {
		SET j TO j-1.
		SET b TO y/(1+(j-1)/(j+2)*(1-b)).
	}
	
	LOCAL Q IS 1/w^2 * (1 + (2-b/2) / (3*w*(w+1))).
	
	RETURN Q.
}.

FUNCTION si {
	DECLARE PARAMETER dterror.
	DECLARE PARAMETER xguess.
	DECLARE PARAMETER dtguess.
	DECLARE PARAMETER xmin.
	DECLARE PARAMETER dtmin.
	DECLARE PARAMETER xmax.
	DECLARE PARAMETER dtmax.
	
	LOCAL etp IS 0.000001.
	LOCAL dxs IS 0.
	
	LOCAL dtminp IS dtguess - dtmin.
	LOCAL dtmaxp IS dtguess - dtmax.
	IF (ABS(dtminp)<etp) OR (ABS(dtmaxp)<etp) {
		SET dxs TO 0.
	} ELSE {
		IF dterror<0 {
			SET dxs TO (xguess-xmax) * (dterror/dtmaxp).
			IF (xguess+dxs)<=xmin {
				SET dxs TO (xguess-xmin) * (dterror/dtminp).
			}
			SET xmax TO xguess.
			SET dtmax TO dtguess.
		} ELSE {
			SET dxs TO (xguess-xmin) * (dterror/dtminp).
			IF (xguess+dxs)>=xmax {
				SET dxs TO (xguess-xmax) * (dterror/dtmaxp).
			}
			SET xmin TO xguess.
			SET dtmin TO dtguess.
		}
	}
	
	RETURN LIST(dxs, xmin, dtmin, xmax, dtmax).
}.
