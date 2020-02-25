![alt text](https://avatars2.githubusercontent.com/u/8077370?s=200&v=4)

Skyward Boardcore
[![pipeline status](https://git.skywarder.eu/scs/skyward-boardcore/badges/master/pipeline.svg)](https://git.skywarder.eu/scs/skyward-boardcore/commits/master)
-------------

Boardcore is a framework for developing and building missile software for custom boards with Miosix .

[Miosix](https://miosix.org/) is a lightweigth OS for embedded developing which provides support for basic things such as Threads, GPIO, Time and many other. You can find our fork of the kernel here: [skyward/miosix-kernel](https://git.skywarder.eu/elc/miosix-kernel)

Building is made with [SBS](https://git.skywarder.eu/scs/skyward-boardcore/wikis/Skyward-Build-Systems-(SBS)), which was created to easily compile and reuse code for different boards.

### Content

| **src/**     | sources!                                                |
| ------------ | ------------------------------------------------------- |
| shared/      | objects, drivers and other stuff written by us          |
| entrypoints/ | each file here is a 'main'                              |
| tests/       | contains the 'main' of every test                       |
| **bin/**     | compiled binaries that can be flashed on a target board |
| **build/**   | sbs stuff, not interesting                              |
| **data/**    | configuration (barely used now)                         |
| **libs/**    | external libs (Miosix kernel as a git submodule)        |
| **obj/**     | build folder, not interesting                           |
| **scripts/** | some tools (e.g. script for flashing on the boards)     |

In the main folder you will find **sbs.conf** which is used to configure the build system.

### Getting Started

#### Dependencies

* Python3
* Git
* Miosix toolchain

Also, openocd, cppcheck and clang-format are recommended for a better experience.

#### Cloning the repo


Clone this repo with the `--recursive` option and build everything.
```
git clone --recursive https://git.skywarder.eu/scs/skyward-boardcore.git
cd skyward-boardcore
python3 sbs -v
```

SBS will start building all the entrypoints. Depending on how many entrypoints there are, this operation can take several minutes.

Once SBS finished, check the resulting message: if every build displays an *OK* message, pat yourself on the back - you've got things *working*!

### What's next?

In the [Wiki](https://git.skywarder.eu/scs/skyward-boardcore/wikis/home) you will find some first-steps **guides** (configuring the IDE, building a firmware etc) as well as the **coding guidelines** and some **best practices** we adopt.

If you want to contribute to this repository, please read [Git Workflow](https://git.skywarder.eu/scs/skyward-boardcore/wikis/Git-Workflow).

If you just want to start messing around, try [this](https://git.skywarder.eu/scs/skyward-boardcore/wikis/Boardcore-Quick-Start).