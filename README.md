![alt text](https://avatars2.githubusercontent.com/u/8077370?s=200&v=4)

Skyward Boardcore
[![pipeline status](https://git.skywarder.eu/r2a/skyward-boardcore/badges/master/pipeline.svg)](https://git.skywarder.eu/r2a/skyward-boardcore/commits/master)
-------------

Boardcore is the framework in which we develop and build the software dedicated to our rockets' boards.

The software is mainly built for [Miosix](https://miosix.org/), a lightweigth OS
for embedded developing which provides support for basic things such as Threads, GPIO, Time and many other.  

Building is made with [SBS](https://github.com/skyward-er/skyward-boardcore/wiki/Skyward-Build-System-(SBS)), a build system
which was created to easily compile and reuse code for different boards. 

### Content

| Folder        | What's in it  |
| ----------- | ---------------------- | 
| **bin/** | compiled binaries that can be flashed on a target board |
| **build/** | sbs stuff, not interesting |
| **config/** |  miosix external config (boards ecc)|
| **data/** | configuration (barely used now) |
| **libs/** | external libs (Miosix kernel as a git submodule) |
| **obj/** | build folder, not interesting |  |
| **scripts/** | some tools (e.g. script for flashing on the boards) |
| **src/** | sources! |
| **src/entrypoints** | software each file here is a 'main' |
| **src/shared** | objects, drivers and other stuff written by us |

In the main folder you will find **sbs.conf** which defines all the boards that sbs will build.

### Getting Started

Install Python, Git and Miosix toolchain. Also openocd and Clang-format are reccomended.

Clone this repo with the `--recursive` option and build everything.
```
git clone --recursive https://github.com/skyward-er/skyward-boardcore.git
cd skyward-boardcore
./sbs 
```
If SBS exited with an *OK* message - you've got things *working*!

If SBS exited with an *OK* message, check that the **bin/** folder contains the boards' binaries and then
pat yourself on the shoulder - you've got things *working*!

*Mac users:*

> "You're entering a world of pain"

### What's next?

In the [Wiki](https://github.com/skyward-er/skyward-boardcore/wiki) you will find some first-steps **guides** (configuring the IDE, building a firmware etc) as well as the **coding guidelines** and some **best practices** we adopt.

Or, if you just want to start messing around, try [this](https://github.com/skyward-er/skyward-boardcore/wiki/Writing-a-driver).


Useful links
-----------

* [Miosix Wiki](https://miosix.org/wiki/index.php?title=Main_Page) for the installation.
* [Miosix Doxygen](https://miosix.org/doxygen/doxygen_k2.01/index.html) for the full documentation (classes, constants ecc).
* [ELC Handbook](https://github.com/skyward-er/elc-internal-reports/tree/master/The%20ELC%20Handbook) 
* [Wiki di skyward](todo)
