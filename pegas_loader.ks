//	pegas_loader.ks
//	System setup: compiles and copies modules onto the main processor.

SWITCH TO 0.	//	in case kOS not started on archive
COMPILE pegas_nav.	//	navigation routines library
COMPILE pegas_lib.	//	PEGAS-specific functions
COMPILE pegas_set.	//	current settings
COMPILE pegas.		//	launch sequence
//	Except the settings, you might want to comment out most compilation (unless you're editing the script, that is), it will save lots of load time.

SWITCH TO 1.
COPY pegas_nav.ksm FROM 0.
COPY pegas_lib.ksm FROM 0.
COPY pegas_set.ksm FROM 0.
COPY pegas.ksm FROM 0.
//	Copies the compiled modules to cut weight.

RUN pegas.ksm.