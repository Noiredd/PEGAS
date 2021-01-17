//  Pre-flight checks for critical configuration variables
SET missingConfigVars TO LIST().
IF NOT (DEFINED vehicle) {
    missingConfigVars:ADD("vehicle (physical description of the vehicle)").
}
IF NOT (DEFINED sequence) {
    missingConfigVars:ADD("sequence (launch sequence description)").
}
IF NOT (DEFINED controls) {
    missingConfigVars:ADD("controls (initial guidance parameters)").
}
IF NOT (DEFINED mission) {
    missingConfigVars:ADD("mission (target orbit description)").
}
//  Crash the system now instead of trying to run and fail after wasting time on loading sub-modules.
IF missingConfigVars:LENGTH > 0 {
    CLEARSCREEN.
    PRINT "CRITICAL ERROR.".
    PRINT " ".
    PRINT "The following necessary configuration variables have not been defined:".
    FOR missingVar IN missingConfigVars {
        PRINT "  * " + missingVar.
    }
    PRINT "Please set these variables before re-running the system.".
    PRINT "Refer to the documentation if unsure how to proceed.".
    PRINT " ".
    PRINT "For your convenience, PEGAS will now crash.".
    PRINT " ".
    PRINT " ".
    PRINT " ".
    PRINT " ".
    PRINT " ".
	SET _ TO sequence.
	SET _ TO controls.
	SET _ TO vehicle.
	SET _ TO mission.
}
