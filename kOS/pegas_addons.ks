//	Addons library

//	This lexicon stores all delegates registered for each of the possible hooks.
GLOBAL addonHookRegistry IS LEXICON(
	"init", LIST(),			//	system is ready, everything set, just before the GUI is first drawn
	"passivePre", LIST(),	//	passive guidance loop, before everything else
	"passivePost", LIST(),	//	passive guidance loop, after everything else
	"activeInit", LIST(),	//	transition from passive to active guidance, right before the first loop
	"activePre", LIST(),	//	active guidance loop, before everything else
	"activePost", LIST(),	//	active guidance loop, after everything else
	"final", LIST()			//	just before exiting the program
).

//	This list stores all registered addons
GLOBAL registeredAddons IS LIST().


//	Call this from your addon file to register your function at any desired hook.
FUNCTION registerHook {
	//	Expects a global variable "addonHookRegistry" as lexicon.

	DECLARE PARAMETER hook.	//	Expects a delegate
	DECLARE PARAMETER mode. //	Expects a string (registry key)

	IF addonEnabled {
		addonHookRegistry[mode]:ADD(hook).
	}
}

//	Main module calls this to execute all hooked delegates.
FUNCTION callHooks {
	//	Expects a global variable "addonHookRegistry" as lexicon.
	DECLARE PARAMETER mode.	//	Expects a string (registry key)

	FOR hook IN addonHookRegistry[mode] {
		hook:CALL().
	}
}

//	Scans the addon directory to detect user addons and execute them.
FUNCTION scanAddons {
	//	Look at all the script files in the addons directory and execute them.
	LOCAL addonDir IS CORE:CURRENTVOLUME:OPEN("addons").
	LOCAL addonItems IS addonDir:LEXICON().

	FOR itemName IN addonItems:KEYS() {
		//	Execute all ".ks" files
		IF itemName:ENDSWITH(".ks") {
			//	Addons can name themselves by overriding this variable
			GLOBAL addonName IS "".
			//	Addons can be enabled/disabled by overriding this variable
			GLOBAL addonEnabled IS TRUE.
			//	Run addon code (this is where the addon calls "registerHook")
			RUNPATH("addons/" + itemName).
			//	Log that it has been loaded - either via the variable, or by file name
			IF addonEnabled {
				IF addonName:LENGTH > 0 {
					registeredAddons:ADD(addonName).
				} ELSE {
					registeredAddons:ADD(itemName).
				}
			}
		}
	}
}
