Skyward Boardcore
-------------

Here we develop and manage all the code that is destined to the boards.

Our software is mainly built upon [miosix](https://miosix.org/), a lightweigth OS
for embedded developing which provides support for basic things such as Threads, GPIO, Time and many other.  

Building is made with [SBS](todo) (Skyward Build System),
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

Install Python and Git (and miosix toolchain?)

Clone this repo with the `--recursive` option and build everything.
```
git clone --recursive https://github.com/skyward-er/skyward-boardcore.git
cd skyward-boardcore
./sbs 
```

If sbs exited with an *OK* message, check that the **bin/** folder contains the boards' binaries and then
pat yourself on the shoulder - you've got things *working*!

*Mac users:*

> "You're entering a world of pain"

### What's next?

Wanna start messing around? Start with [this](https://github.com/skyward-er/skyward-boardcore/wiki/Writing-a-driver).

Please also consider reading the **code conventions** and **best practices** which you'll found in [the wiki](https://github.com/skyward-er/skyward-boardcore/wiki).
