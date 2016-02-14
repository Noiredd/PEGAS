//	pegas_set.ks
//	Settings file

GLOBAL S_countdown IS 5.	//	countdown time in seconds
GLOBAL S_logging IS 0.		//	1 will output a flight log file (beware of crashes - log will fill up your processor memory!)
GLOBAL S_msgTTL IS 3.		//	displayed message time to live (see: pegas_lib, addMessage)
GLOBAL S_krTime IS 5.		//	time before 1st stage shutdown when rocket begins killing rotation for a safe separation (TODO: maybe make it part of the pitch program?)
GLOBAL S_pegCycle IS 1.		//	PEG major cycle (time between calculations) in seconds
GLOBAL S_pegEpsilon IS 3.	//	stop solving matrices when PEG-calculated time to cutoff becomes smaller than this

GLOBAL S_target IS 200.		//	target altitude in kilometres ASL (TODO: when yaw control is implemented, maybe move that to a separate file, along with the rest of orbit parameters)