## Structure reference

### Controls
`GLOBAL controls IS LEXICON().`

Variable of type `LEXICON`.
Defines vehicle behavior in phases 0 and 1 (pre-launch and atmospheric ascent).
Has to have the following 4 keys:

Key                | Units | Meaning
---                | ---   | ---
launchTimeAdvance  | s     | Launch time will be scheduled that many seconds before the launch site rotates directly under the target orbit
verticalAscentTime | s     | After liftoff, vehicle will fly straight up for that many seconds before pitching over
pitchOverAngle     | deg   | Vehicle will pitch over by that many degrees away from vertical
upfgActivation     | s     | The active guidance phase will be activated that many seconds after liftoff

### Sequence
`GLOBAL sequence IS LIST().`

Variable of type `LIST` containing `LEXICON`s.
Defines 
