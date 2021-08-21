// A PEGAS addon that adds support for Kerbal Alarm Clock mod.

// Addon settings
LOCAL alarmsEnabled IS TRUE.		//	Controls whether the addon is enabled. If you don't want PEGAS to add alarms, set this to FALSE.
LOCAL kacAlarmAdvance IS 30.		//	Defines how many seconds before lift off the alarm should go off (30 seconds by default).

//	Add alarm if requested
FUNCTION addKacAlarm {
	//	Make sure Kerbal Alarm Clock is installed before trying to add the alarm.
	IF ADDONS:AVAILABLE("KAC") {
		//	Only add the alarm if it will go off more that 5 seconds from now. Otherwise it's not really needed.
		IF liftoffTime - kacAlarmAdvance > currentTime + 5 {
			ADDALARM("Raw", liftoffTime:SECONDS - kacAlarmAdvance, "Launch Alarm", SHIP:NAME + " is launching in " + kacAlarmAdvance + " seconds.").
			pushUIMessage("Alarm added to KAC.").
		}
	} ELSE {
		//	If KAC is not available, display a UI message
		pushUIMessage("Failed to add KAC alarm! KAC not installed.").
	}
}

// If alarms are enabled, register this addon with PEGAS in the initiation phase
IF alarmsEnabled {
	register_hook(addKacAlarm@, "init").
}