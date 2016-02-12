//	This is a PEGAS sample boot file.
//	You're supposed set the variables yourself and attach it to a kOS processor that will run the whole launch sequence. 
//	Make sure your processor has enough memory to store the whole code.

GLOBAL P_vName IS "PEGAS test ICBM".
//	vehicle name

GLOBAL P_seq IS LIST(-1.4, 0, 129.0, 139.0, 140.0, 140.5, 160.5, 165, 378.2 ).
//	launch events sequence, in relation to liftoff, in a specific order:
//		ignition (delay before release)
//		booster jettison (or zero if none are present)
//		fairings jettison (or zero if none are present)
//		first stage main engine cutoff
//		separation
//		ullage thruster ignition
//		second stage main engine ignition
//		PEG activation
//		stage 2 maximum burn time (not related to liftoff!) - used to initialize PEG

GLOBAL P_pt IS LIST(  0.0000,  16.0000,  19.3000,  24.0000,  48.1154,  67.9449,  93.7537, 139.0000 ).
GLOBAL P_pp IS LIST(  0.0000,   0.0000,   3.2405,   3.2405,  14.9415,  27.4732,  43.4237,  58.7565 ).
//	first stage guidance pitch-time table: P_pt is time in seconds since liftoff, P_pp is pitch angle (0 is vertical)
//	I'm very sorry that pitch is given in degrees from vertical, while kOS relates to the horizon.
//	We'll end up having to subtract any calculated pitch from 90 before supplying it to any steering command.

GLOBAL P_umode IS 25.
//	ullage thruster mode:
//		if zero - ullage handled by SRBs, no shutdown necessary
//		if positive - ullage handled by RCS, value is ullage burn time

TOGGLE AG10.
//	opens a kOS terminal (be sure to set an action group on the processor running PEGAS)

RUN pegas_loader.ks.
//	continue boot sequence