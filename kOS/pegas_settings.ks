//	Settings for PEGAS

GLOBAL SETTINGS IS LEXICON(
	//	kOS_IPU controls how fast the kOS interpreter runs. Bumping this up is required to run the script fast enough.
	"kOS_IPU", 400,

	//	Which version of the CSER function to use. "new" implementation (by pand5461) has been extensively tested and is the default one.
	//	In case of problems, you can try switching to "old" CSER, but remember to bump up the IPU to about 500.
	"cserVersion", "new",

	//	In atmospheric part of ascent, when the vehicle pitches over, the wait for velocity vector to align will be forcibly broken after that many seconds.
	"pitchOverTimeLimit", 20,

	//	UPFG will start attempting to converge that many seconds before its scheduled activation.
	"upfgConvergenceDelay", 10,

	//	When time-to-go gets below that, keep attitude stable and simply count down time to cutoff.
	"upfgFinalizationTime", 5,

	//	Updating attitude commands will be forbidden that many seconds before staging (in an attempt to keep vehicle steady for a clean separation).
	"stagingKillRotTime", 5,

	//	Maximum difference between consecutive UPFG T-go predictions that allow accepting the solution.
	"upfgConvergenceCriterion", 0.5,

	//	Maximum angle between guidance vectors calculated by UPFG between stages that allow accepting the solution.
	"upfgGoodSolutionCriterion", 15,

	"_", ""	//	dummy for commit reasons
).
