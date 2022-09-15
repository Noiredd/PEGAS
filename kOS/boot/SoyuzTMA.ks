GLOBAL vehicle IS LIST(
	LEXICON(
		//THIRD STAGE WITH RD-0124
		"name", "R7, RD-0124 Third Stage",
		"massTotal", 16772,
		"massFuel", 3105+3800,
		"engines", LIST(LEXICON("isp", 330, "thrust", 148000)),
		"staging", LEXICON(
			"jettison", FALSE,
			"ignition", FALSE
		)
	)
).

GLOBAL controls IS LEXICON(
	"launchTimeAdvance", 140,
	"upfgActivation", 156,
	"pitchProgram", LEXICON(
		"altitude", LIST(200, 20000, 65000),
		"pitch", LIST(90, 45, 0)
	)
	//	Read: until altitude 200m fly at 90 degrees, then turn so that at 20km pitch is 45 degrees,
	//	then continue turning so that at 65km pitch is 0 degrees.
).

//Set time (in s) between ignition and liftoff here
SET IgnTime TO 2.

GLOBAL sequence IS LIST(
	LEXICON("time", -IgnTime, "type", "stage", "message", "Ignition!"),
	LEXICON("time", 0, "type", "stage", "message", "Liftoff!"),
	LEXICON("time", 70-IgnTime, "type", "stage", "message", "LES tower Ejected!" ), //"massLost", 2162,
	LEXICON("time", 77-IgnTime, "type", "stage", "message", "First Stage is ejecting"),
	LEXICON("time", 78-IgnTime, "type", "stage", "message", "First Stage is ejected" ),
	LEXICON("time", 100-IgnTime, "type", "stage", "message", "Fairings Ejected"), //"massLost", 3200,
	LEXICON("time", 153-IgnTime, "type", "stage", "message", "Third Stage Ignition" ),
	LEXICON("time", 154-IgnTime, "type", "stage", "message", "Second Stage ejection" )
).

GLOBAL mission IS LEXICON(
	"apoapsis", 72,
	"periapsis", 72,
	"inclination", 51.65
).

CLEARSCREEN.
PRINT "Loaded boot file: Soyuz TMA".
