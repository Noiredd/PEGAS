GLOBAL vehicle IS LIST(
					LEXICON(
						//	This is a sustainer-type stage. It ignites on the pad, but UPFG takes control over it mid-flight.
						//	Therefore, no ignition or separation happens, and the actual mass of the stage at activation is
						//	inferred automatically.
						"name", "Common Core Booster",
						"massTotal", 334872,
						"massFuel", 207720+76367,
						"gLim", 4.5,
						"engines", LIST(LEXICON("isp", 338.4, "thrust", 4152000)),
						"staging", LEXICON(
										"jettison", FALSE,
										"ignition", FALSE
										)
					),
					LEXICON(
						"name", "Centaur (DEC)",
						"massTotal", 22374,			//	RSB Centaur has too much oxygen in the tank
						"massFuel", 16402+3281,		//	these masses are for Centaur that has been reset and had tanks readjusted
						"engines", LIST(LEXICON("isp", 422.0, "thrust", 2*67000)),
						"staging", LEXICON(
										"jettison", TRUE,
										"waitBeforeJettison", 3,
										"ignition", TRUE,
										"waitBeforeIgnition", 2,
										"ullage", "rcs",
										"ullageBurnDuration", 5,
										"postUllageBurn", 2
										)
					)
).
GLOBAL sequence IS LIST(
					LEXICON("time", -3.7, "type", "stage", "message", "RD-180 ignition"),
					LEXICON("time", 0, "type", "stage", "message", "LIFTOFF"),
					LEXICON("time", 100, "type", "stage", "message", "SRB jettison"),
					LEXICON("time", 210, "type", "stage", "message", "PLF jettison"),
					LEXICON("time", 215, "type", "stage", "message", "CFLR jettison")
).
GLOBAL controls IS LEXICON(
					"launchTimeAdvance", 150,
					"verticalAscentTime", 8,	//	8 seconds work well for 5t payload, 9 good for 15t
					"pitchOverAngle", 10,
					"upfgActivation", 115
).
SET STEERINGMANAGER:ROLLTS TO 10.
SWITCH TO 0.
CLEARSCREEN.
PRINT "Loaded boot file: AtlasV-532!".