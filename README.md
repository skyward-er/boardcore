Skyward Boardcore
-------------

Skyward Boardcore was born as a software framework and shared place
where we can easly develop and manage all the code that is destined to the rockets.

It is founded on [SBS](todo)(Skyward Build System),
a tool that permits to easily compile and reuse code for different boards.

Most of the software that we use for the boards is based on [miosix](https://miosix.org/), a lightweigth kernel
for embedded developing which provides support for basic things such as Threads, GPIO, Time and many other.                        

### Content

| Folder        | What's in it  |
| ----------- | ---------------------- | 
| bin/ | compiled binaries that can be flashed on a target board |
| build/ | sbs stuff, not interesting |
| config/ |  miosix external config (boards ecc)|
| data/ | configuration (barely used now) |
| libs/ | external libs (Miosix kernel as a git submodule) |
| obj/ | build folder, not interesting |  |
| scripts/ | some tools (e.g. script for flashing on the boards) |
| src/ | sources! |
| src/entrypoints | software each file here is a 'main' |
| src/shared | objects, drivers and other stuff written by us |

In the main folder you will find sbs.conf which defines all the boards that sbs will build.

Getting Started
---------------

Install Python and Git (and miosix toolchain?)

Clone this repo with the `--recursive` option
```
git clone --recursive https://github.com/skyward-er/skyward-boardcore.git
```

Build everything
```
cd skyward-boardcore
./sbs 
```

If sbs exited with an *OK* message, check that the *bin/* folder contains the boards' binaries and then
pat yourself on the shoulder - you've got things *working*!

*Mac users:*

> "You're entering a world of pain"

Next steps
----------

To start messing around without getting hurt:

* [SBS Quick Guide](https://github.com/skyward-er/skyward-boardcore#sbs-build
* [Eclipse configuration](todo)
* [Writing a Driver](todo)
* [Flashing on a STM32 DISCOVERY](todo)
* [Flashing on a TROOPER](todo)

Also refer to [our wiki](https://github.com/skyward-er/skyward-boardcore/wiki) for coding conventions, best practices etc .

Useful links
-----------

* [Miosix Wiki](https://miosix.org/wiki/index.php?title=Main_Page) for the installation.
* [Miosix Doxygen](https://miosix.org/doxygen/doxygen_k2.01/index.html) for the full documentation (classes, constants ecc).
* [ELC Handbook](https://github.com/skyward-er/elc-internal-reports/tree/master/The%20ELC%20Handbook) 
* [Wiki di skyward](todo)

(TODO put in the wiki) SBS Quick Guide
------
SBS is the tool with which we can compile board-specific software without needing to deal with 
the miosix kernel configuration every time.

It can be used to build the entire system or just a single board firmware, using 
```
.\sbs -b *boardname*
```

### Configuration

The board list is defined in the *sbs.conf* file which is structured as follows 
(as described in the file itself):
```
 Available types: srcfiles/board
 srcfiles:
     Files: a '\n'-separated list of files

 board:
     BoardId: see the list below (discovery, stormtrooper ecc)
     BinName: name of the final binary (without extension!)
     Include: a space-separated list of files, %something will be substituted
              with the corresponding 'srcfiles'
     Main:    name of the main file (e.g. 'foo' -> src/entrypoints/foo.cpp)
```

The *srcfiles* are used for grouping together classes that refer to a common device/porpouse,
while the *boards* contain information on how to build a specific entrypoint.

For example

```
#srcfiles

[canbus]
Type:       srcfiles
Files:      src/shared/drivers/canbus/CanManager.cpp 
            src/shared/drivers/canbus/CanBus.cpp 
            src/shared/drivers/canbus/CanSocket.cpp 
            src/shared/drivers/canbus/CanInterrupt.cpp 

#boards

[discovery-canbus-test]
Type:       board
BoardId:    stm32f429zi_stm32f4discovery
BinName:    discovery-canbus-test
Include:    %canbus %shared
Defines:    
Main:       canbus-test

```

### Other options

Prompting `./sbs --help` will print the options menu:

```
  -h, --help            show this help message and exit
  -b BOARD, --board=BOARD
                        Build a specific board
  -c, --clean           Run a 'make clean'
  -d, --debug           Build a board with debug logs
  -l, --list            List all build configurations
  -g, --gen-faults      Generate fault list header files and exit
  -v, --verbose         Print a verbose output
  -j JOBS, --jobs=JOBS  Specifies the number of jobs (commands) to run
                        simultaneously.
```



(TODO put in the wiki) Writing a Driver
------

* Create your own branch
* Create a folder in src/shared/driver with 
	* A object.cpp and object.h
	* A config.h file that contains all the defines, gpio typedefs and everything that can be fine-tuned
	* A test.cpp that will contain a sample of the usage
* Create an entrypoint: put a .cpp main in src/entrypoints
* Modify sbs.conf: add a srcfile and a board (see [sbs quick guide](todo)).


(TODO put in the wiki) Coding Rules & Best Practices
------------
TODO *everything*: decide style conventions, write design best practices and start adding examples
* Code style and naming conventions
* Git rules: [branching](http://nvie.com/posts/a-successful-git-branching-model/), commit messages style

* Best Practices: *general design rules that you should follow in order to write bomb-proof code*
	- Terraneo's examples on multi threading
	- [NASA coding principles](http://pixelscommander.com/wp-content/uploads/2014/12/P10.pdf) (revisited)
* Common examples: *Things that you are very likely to need when writing code in boardcore*
	- Read from and Write to serial (other than default)
	- data logging (debugging)
	- active objects (scheduling)
	- error reporting (fault counter)
	- miosix queues
	- miosix threads
	- scheduler e funzioni con timeout
	- profiling della memoria

* Cheat Sheet: *Things that made us say "If only I knew that before!" (or maybe worse)*
	- Add another usart to a board 
	- Use right amount of stack (printf!)
	- Chiamare una funzione di un oggetto in un thread (wrapper statico)
