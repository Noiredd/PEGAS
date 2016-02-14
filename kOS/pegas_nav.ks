//	pegas_nav.ks
//	Functions to calculate current ship directions as read on the navball.
//	Based on lib_navball.ks, created by KSLib team, shared under MIT license.

//	east		technical, to obtain ship's East vector
FUNCTION east {
	RETURN VCRS(SHIP:UP:VECTOR, SHIP:NORTH:VECTOR).
}

//	navPitch	gets current compass pitch (degrees above horizon)
FUNCTION navPitch {
	RETURN 90-VANG(SHIP:UP:VECTOR, SHIP:FACING:FOREVECTOR).
}

//	navHdgV		gets compass heading for any vector
FUNCTION navHdgV {
	DECLARE PARAMETER pointing.
	LOCAL result IS ARCTAN2( VDOT(east(), pointing), VDOT(SHIP:NORTH:VECTOR, pointing) ).
	IF result < 0 { RETURN result+360. } ELSE RETURN result.
}

//	navHeading	shortcut to get current compass heading (degrees from north towards east)
FUNCTION navHeading {
	RETURN navHdgV(SHIP:FACING:FOREVECTOR).
}

//	navRoll		gets current roll angle
FUNCTION navRoll {
	IF VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR) < 0.2 { RETURN 0. } //	deadzone against gimbal lock (when vehicle is too vertical, roll angle becomes indeterminate)
	ELSE {
		LOCAL raw IS VANG(VXCL(SHIP:FACING:VECTOR, SHIP:UP:VECTOR), SHIP:FACING:STARVECTOR).
		IF VANG(SHIP:UP:VECTOR, SHIP:FACING:TOPVECTOR) > 90 {
			RETURN 270-raw.
		} ELSE {
			RETURN -90-raw.
		}
	}.
}