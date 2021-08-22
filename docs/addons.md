## Addons

Since version 1.3-alpha, PEGAS can be extended via _addons_ -
custom pieces of kOS code that can be _hooked_ to the main module, `pegas.ks`.

### Getting addons
Visit [PEGAS-addons](https://github.com/Noiredd/PEGAS-addons) repository to browse and download addons.

### Installing addons
Simply put the addon file in the `addons/` directory.
PEGAS automatically detects all installed addons, you don't need to do anything else.

You can temporarily _disable_ an addon by adding the following line into the addon file:
```
GLOBAL addonEnabled IS FALSE.
```
(or, if the variable already exists, changing from `TRUE` to `FALSE`).

To _uninstall_ an addon, simply remove its `.ks` file.

### Creating addons
Addons system is fairly simple as long as you understand [delegates](http://ksp-kos.github.io/KOS_DOC/language/delegates.html).

The idea is that you create your own module (i.e. a `.ks` file)
and within it define some function (or functions)
that you would wish to be executed during normal PEGAS operation.
Then you _register_ your function under either of the available _hooks_,
that is points in the main module (`pegas.ks`) when you would like this function to be called.
PEGAS takes care of the rest:
on each of these predefined hook locations it calls all registered delegates.

To give a simple example:
```
// define your function
FUNCTION myCustomFunction {
  // do something here
}

// register a hook
registerHook(myCustomFunction@, "init").  // note the delegate notation: @
// the above function seems undefined, but is in fact provided by pegas_addons.ks
```
In this example, a function `myCustomFunction` has been registered to run at `"init"`.
This means PEGAS will call this function as soon as it initializes all its internal structures.  
For other possibilities, check the [addon handling code](../kOS/pegas_addons.ks) itself,
paricularly `addonHookRegistry`.
To see the exact hook points, check the [main module](../kOS/pegas.ks),
look for lines containing `callHooks()`.

Your custom function can be anything, as long as it expects **no arguments**.
You can return values, but they will be ignored.
Of course, you're free to use and modify all PEGAS' global variables.

It's nice to include the following variables in your module:
* `addonName`: string with a name for your addon,
* `addonEnabled`: bool, `TRUE` by default; allows users to enable/disable your addon easily.
