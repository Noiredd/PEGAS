## PEGAS documentation
**PEGAS** consists of two main elements: a MATLAB prototype (with a test environment) and the kOS code (*currently obsolete, to-be-work-in-progress*).
In the following section you will find a brief documentation for the MATLAB part of the project, specifically the UPFG implementation with some explanation of the algorithm itself, and the simulation environment.
Hopefully this will allow a properly equipped user (one obviously needs a MATLAB license to use this software) to run their own simulations.

### [Unified Powered Flight Guidance](upfg.md)
This part focuses on the algorithm itself, stating the guidance problem and explaining the solution.
Some math background might be necessary to understand the concepts used here.

### [PEGAS MATLAB implementation](simulation.md)
PEGAS includes a simulation environment to test the guidance algorithm, including a physics simulation, vehicle setup and various debugging/visualisation tools.
This section covers the framework from the concept and physics point of view.
Potentially it is too detailed than necessary in order to run your own mission - you might not need to know how all the internal structures are laid out - however, reading *introduction*, *flow*, *physics* and *vehicle* is recommended.

### [Tutorial: run my own mission](tutorial.md)
A brief walkthrough on running a simulation from scratch, on an example Space Shuttle mission.

### Example missions
Demonstration of PEGAS capabilities and limitations.
