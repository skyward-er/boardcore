# Skyward Boardcore

Description
----------

Skyward boardcore was born as Skyward's software framework, a common place where 
to manage and share all the code that is destined to the rocket.

It is founded on sbs (Skyward Build System),
a tool that permits to easily compile and reuse code for different target boards on the miosix kernel.

If you are new to this place, please read the **coding rules**(TODO) of our community before you start coding!

Content
--------
| Folder        | AL's definition           | Comments  |
| ----------- |----------------------| ----------|
| bin/ | compiled binaries |   |
| build/ | sbs stuff, not interesting |   |
| config/ |  same... |   |
|data/ | configuration (barely used now)
| libs/ | external libs are here (maybe as git submodules) e.g. libs/miosix-kernel  |   |
| obj/ | build folder, not interesting |   |
| scripts/ | some tools |   |
| src/ | sources! |   |
| src/entrypoints | each file here is a 'main' |   |
| src/shared | collection of stuff, drivers, libraries, ... written by us |   |

In the main root you can find sbs.conf which defines all the boards that sbs will build


Getting Started
---------------
Install Python and Git (and miosix toolchain?)

Clone this repo

```
git clone --recursive https://github.com/skyward-er/skyward-boardcore.git
```

Build everything
```
cd skyward-boardcore
./sbs 
```

If sbs exits with OK, *rejoyce* and pat yourself on the shoulder - you've got things *working*!
You'll find all the binary files in the *bin/* folder, ready to be flashed on your target board.

If you want to compile only a specific entrypoint, you can `make clean` and then `.\sbs -b *boardname*`.

*Other useful things to know when getting started*:

* [Eclipse configuration](todo)
* [SBS full reference](todo)
* [Flashing on STM32 DISCOVERY](todo)
* [Flashing on a TROOPER](todo)

(TODO) Writing a Driver
------
Maybe create a template/"wizard" tool?
* Create your own branch
* Create a folder in src/shared/driver with 
	* A object.cpp and object.h
	* A config.h file that contains all the defines, gpio typedefs and everything that can be fine-tuned
	* A test.cpp that will contain a sample of the usage
* Create an entrypoint: put a .cpp main in src/entrypoints
* Modify sbs.conf: add a srcfile and a board

(TODO) Coding Rules & Guides 
------------
TODO *everything*: decide style conventions, write design best practices and start adding examples
* Code style and naming conventions
* Git rules
* Best Practices: *general design rules that you should follow in order to write bomb-proof code*
	- Terraneo's examples on multi threading
	- [NASA coding principles](http://pixelscommander.com/wp-content/uploads/2014/12/P10.pdf) (revisited)
* Common examples: *Things that you are very likely to need when writing code in boardcore*
	- Read from and Write to serial (other than default)
	- data logging
	- active objects
	- error reporting (fault counter)
	- miosix queues
	- miosix threads
	- scheduler e funzioni con timeout
	- profiling della memoria
* Cheat Sheet: *Things that made us say "If only I knew that before!" (or maybe worse)*
	- Add another usart to a board 
	- Use right amount of stack (printf!)
	- Chiamare una funzione di un oggetto in un thread (wrapper statico)
