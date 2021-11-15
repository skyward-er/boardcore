![alt text](https://avatars2.githubusercontent.com/u/8077370?s=200&v=4)

Skyward Boardcore
[![pipeline status](https://git.skywarder.eu/scs/skyward-boardcore/badges/master/pipeline.svg)](https://git.skywarder.eu/scs/skyward-boardcore/commits/master)
-------------

Boardcore is a framework for developing and building missile software for custom boards with Miosix.

[Miosix](https://miosix.org/) is a lightweight OS for embedded developing which provides support for basic things such as Threads, GPIO, Time and many other. You can find our fork of the kernel here: [skyward/miosix-kernel](https://git.skywarder.eu/scs/miosix-kernel)

Building is made with [SBS](https://git.skywarder.eu/scs/skyward-boardcore/wikis/Skyward-Build-System-(SBS)), which was created to easily compile and reuse code for different boards.

### Content

| **src/**     | sources!                                                |
| ------------ | ------------------------------------------------------- |
| shared/      | objects, drivers and other stuff written by us          |
| entrypoints/ | each file here is a 'main'                              |
| tests/       | contains the 'main' of every test                       |
| **build/**   | compiled binaries that can be flashed on a target board |
| **data/**    | configuration (barely used now)                         |
| **libs/**    | external libs (Miosix kernel as a git submodule)        |
| **scripts/** | some tools (e.g. script for flashing on the boards)     |

In the main folder you will find **CMakeLists.txt** which is used to configure the build system.

### Getting Started

#### Dependencies

* CMake
* Git
* Miosix Toolchain

Also, Ccache, Ninja, OpenOCD, Cppcheck and clang-format are recommended for a better experience.

#### Cloning the repo

Clone this repo with the `--recursive` option.
```sh
git clone --recursive https://git.skywarder.eu/scs/skyward-boardcore.git
cd skyward-boardcore
```

### Building

You can build everything using CMake:
```sh
mkdir build
cd build
cmake -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_TOOLCHAIN_FILE=../libs/miosix-kernel/miosix/_tools/toolchain.cmake -GNinja ..
cmake --build .
```

Or using the SBS wrapper script:
```
./sbs
```

The build system will start building all the targets (entrypoints and tests). Depending on how many targets there are, this operation may take several minutes.

### What's next?

In the [Wiki](https://git.skywarder.eu/scs/skyward-boardcore/wikis/home) you will find some first-steps **guides** (configuring the IDE, building a firmware, etc.) as well as the **coding guidelines** and some **best practices** we adopt.

If you want to contribute to this repository, please read [Git Workflow](https://git.skywarder.eu/scs/skyward-boardcore/wikis/Git-Workflow).

If you just want to start messing around, try [this](https://git.skywarder.eu/scs/skyward-boardcore/wikis/Boardcore-Quick-Start).
