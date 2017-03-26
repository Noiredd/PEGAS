FUNCTION si {
	DECLARE PARAMETER dterror.
	DECLARE PARAMETER xguess.
	DECLARE PARAMETER dtguess.
	DECLARE PARAMETER xmin.
	DECLARE PARAMETER dtmin.
	DECLARE PARAMETER xmax.
	DECLARE PARAMETER dtmax.
	
	SET etp TO 0.000001.
	SET dxs TO 0.
	
	SET dtminp TO dtguess - dtmin.
	SET dtmaxp TO dtguess - dtmax.
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
