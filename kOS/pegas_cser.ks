//	Conic State Extrapolation

FUNCTION cse {
	//	LOCAL FUNCTIONS
	//	Bundled here because of kOS's peculiar scoping rules.
	FUNCTION ktti {
		DECLARE PARAMETER xarg.
		DECLARE PARAMETER s0s.
		DECLARE PARAMETER a.
		DECLARE PARAMETER kmax.

		LOCAL u1 IS uss(xarg, a, kmax).

		LOCAL zs IS 2*u1.
		LOCAL E IS 1 - 0.5*a*zs^2.
		LOCAL w IS SQRT( MAX(0.5+E/2, 0) ).
		LOCAL D IS w*zs.
		LOCAL A IS D^2.
		LOCAL B IS 2*(E+s0s*D).

		LOCAL Q IS qcf(w).

		LOCAL t IS D*(B+A*Q).

		RETURN LIST(t, A, D, E).
	}
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
	}
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
	}
	FUNCTION kil {
		DECLARE PARAMETER imax.
		DECLARE PARAMETER dts.
		DECLARE PARAMETER xguess.
		DECLARE PARAMETER dtguess.
		DECLARE PARAMETER xmin.
		DECLARE PARAMETER dtmin.
		DECLARE PARAMETER xmax.
		DECLARE PARAMETER dtmax.
		DECLARE PARAMETER s0s.
		DECLARE PARAMETER a_.
		DECLARE PARAMETER kmax.
		DECLARE PARAMETER A.
		DECLARE PARAMETER D.
		DECLARE PARAMETER E.

		LOCAL etp IS 0.000001.
		LOCAL i IS 1.
		LOCAL dterror IS 0.
		LOCAL dxs IS 0.
		LOCAL xold IS 0.
		LOCAL dtold IS 0.

		UNTIL NOT (i<imax) {
			SET dterror TO dts-dtguess.

			IF ABS(dterror)<etp {
				BREAK.
			}

			LOCAL pack IS si(dterror, xguess, dtguess, xmin, dtmin, xmax, dtmax).
			SET dxs TO pack[0].
			SET xmin TO pack[1].
			SET dtmin TO pack[2].
			SET xmax TO pack[3].
			SET dtmax TO pack[4].
			pack:CLEAR().

			SET xold TO xguess.
			SET xguess TO xguess+dxs.

			IF xguess=xold { BREAK. }

			SET dtold TO dtguess.

			SET pack TO ktti(xguess, s0s, a_, kmax).
			SET dtguess TO pack[0].
			SET A TO pack[1].
			SET D TO pack[2].
			SET E TO pack[3].

			IF dtguess=dtold { BREAK. }

			SET i TO i+1.
		}

		RETURN LIST(xguess, dtguess, A, D, E).
	}
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
	}
	//END OF LOCAL FUNCTIONS

	DECLARE PARAMETER r0.	//	Expects a vector
	DECLARE PARAMETER v0.	//	Expects a vector
	DECLARE PARAMETER dt.	//	Expects a scalar
	DECLARE PARAMETER last.	//	Expects a lexicon with fields: dtcp, xcp, A, D, E

	LOCAL dtcp IS 0.
	IF last["dtcp"]=0 {
		SET dtcp TO dt.
	} ELSE {
		SET dtcp TO last["dtcp"].
	}
	LOCAL xcp IS last["xcp"].
	LOCAL x IS xcp.
	LOCAL A IS last["A"].
	LOCAL D IS last["D"].
	LOCAL E IS last["E"].

	LOCAL kmax IS 10.
	LOCAL imax IS 10.

	LOCAL f0 IS 0.
	IF dt>=0 {
		SET f0 TO 1.
	} ELSE {
		SET f0 TO -1.
	}

	LOCAL n IS 0.
	LOCAL r0m IS r0:MAG.

	LOCAL f1 IS f0*SQRT(r0m/SHIP:ORBIT:BODY:MU).
	LOCAL f2 IS 1/f1.
	LOCAL f3 IS f2/r0m.
	LOCAL f4 IS f1*r0m.
	LOCAL f5 IS f0/SQRT(r0m).
	LOCAL f6 IS f0*SQRT(r0m).

	LOCAL ir0 IS r0/r0m.
	LOCAL v0s IS f1*v0.
	LOCAL sigma0s IS VDOT(ir0,v0s).
	LOCAL b0 IS v0s:SQRMAGNITUDE - 1.
	LOCAL alphas IS 1-b0.

	LOCAL xguess IS f5*x.
	LOCAL xlast IS f5*xcp.
	LOCAL xmin IS 0.
	LOCAL dts IS f3*dt.
	LOCAL dtlast IS f3*dtcp.
	LOCAL dtmin IS 0.
	LOCAL dtmax IS 0.
	LOCAL xP IS 0.
	LOCAL Ps IS 0.

	LOCAL xmax IS 2*CONSTANT:PI / SQRT(ABS(alphas)).

	IF alphas>0 {
		SET dtmax TO xmax/alphas.
		SET xP TO xmax.
		SET Ps TO dtmax.
		UNTIL NOT (dts>=Ps) {
			SET n TO n+1.
			SET dts TO dts-Ps.
			SET dtlast TO dtlast-Ps.
			SET xguess TO xguess-xP.
			SET xlast TO xlast-xP.
		}
	} ELSE {
		LOCAL pack IS ktti(xmax, sigma0s, alphas, kmax).
		SET dtmax TO pack[0].

		IF dtmax<dts {
			UNTIL NOT (dtmax<dts) {
				SET dtmin TO dtmax.
				SET xmin TO xmax.
				SET xmax TO 2*xmax.

				LOCAL pack IS ktti(xmax, sigma0s, alphas, kmax).
				SET dtmax TO pack[0].
			}
		}
	}

	IF (xmin>=xguess) OR (xguess>=xmax) {
		SET xguess TO 0.5*(xmin+xmax).
	}

	LOCAL pack IS ktti(xguess, sigma0s, alphas, kmax).
	SET dtguess TO pack[0].

	IF dts<dtguess {
		IF (xguess<xlast) AND (xlast<xmax) AND (dtguess<dtlast) AND (dtlast<dtmax) {
			SET xmax TO xlast.
			SET dtmax TO dtlast.
		}
	} ELSE {
		IF (xmin<xlast) AND (xlast<xguess) AND (dtmin<dtlast) AND (dtlast<dtguess) {
			SET xmin TO xlast.
			SET dtmin TO dtlast.
		}
	}

	LOCAL pack IS kil(imax, dts, xguess, dtguess, xmin, dtmin, xmax, dtmax, sigma0s, alphas, kmax, A, D, E).
	SET xguess TO pack[0].
	SET dtguess TO pack[1].
	SET A TO pack[2].
	SET D TO pack[3].
	SET E TO pack[4].

	LOCAL rs IS 1 + 2*(b0*A + sigma0s*D*E).
	LOCAL b4 IS 1/rs.

	LOCAL xc IS 0.
	LOCAL dtc IS 0.
	IF n>0 {
		SET xc TO f6*(xguess+n*xP).
		SET dtc TO f4*(dtguess+n*Ps).
	} ELSE {
		SET xc TO f6*xguess.
		SET dtc TO f4*dtguess.
	}

	SET last["dtcp"] TO dtc.
	SET last["xcp"] TO xc.
	SET last["A"] TO A.
	SET last["D"] TO D.
	SET last["E"] TO E.

	LOCAL F IS 1-2*A.
	LOCAL Gs IS 2*(D*E + sigma0s*A).
	LOCAL Fts IS -2*b4*D*E.
	LOCAL Gt IS 1-2*b4*A.

	LOCAL r_ IS r0m*(F*ir0 + Gs*v0s).
	LOCAL v_ IS f2*(Fts*ir0 + Gt*v0s).

	RETURN LIST(r_, v_, last).
}
