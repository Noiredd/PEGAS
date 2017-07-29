GLOBAL vehicle IS LIST(
					LEXICON(
						//	This stage will be ignited upon UPFG activation.
						"name", "S-II",
						"massTotal", 611841+366,	//	kOS part upgrade
						"massFuel", 377580+72188,
						"engines", LIST(LEXICON("isp", 424, "thrust", 1023090*5)),
						"staging", LEXICON(
										"jettison", TRUE,
										"waitBeforeJettison", 5,
										"ignition", TRUE,
										"waitBeforeIgnition", 4,
										"ullage", "none"
										)
					),
					LEXICON(
						"name", "S-IVB",
						"massTotal", 122607+366,
						"massFuel", 90709+16492,
						"engines", LIST(LEXICON("isp", 424, "thrust", 1023090)),
						"staging", LEXICON(
										"jettison", TRUE,
										"waitBeforeJettison", 5,
										"ignition", TRUE,
										"waitBeforeIgnition", 4,
										"ullage", "none"
										)
					)
).
GLOBAL sequence IS LIST(
					LEXICON("time", -6.5, "type", "stage", "message", "F-1 ignition"),
					LEXICON("time", 0, "type", "stage", "message", "LIFTOFF")
).
GLOBAL controls IS LEXICON(
					"launchTimeAdvance", 120,
					"verticalAscentTime", 25,
					"pitchOverAngle", 3,
					"upfgActivation", 163
).
SET STEERINGMANAGER:ROLLTS TO 10.
SWITCH TO 0.
CLEARSCREEN.
PRINT "Loaded boot file: SaturnV!".
run mission.
run pegas.