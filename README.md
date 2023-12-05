<div align="center">
	<img src="https://avatars2.githubusercontent.com/u/8077370?s=200&v=4" alt="Skyward Experimental Rocketry" width="200"></a>
<h1>Skyward Boardcore</h1>
<a href="https://git.skywarder.eu/avn/swd/skyward-boardcore/-/pipelines"><img src="https://git.skywarder.eu/avn/swd/skyward-boardcore/badges/main/pipeline.svg"></a>
</div>

Boardcore is a C++14 rocket software framework targeting custom embedded boards designed by Skyward Experimental Rocketry.

Boardcore runs on top of [Miosix](https://miosix.org/), a lightweight operating system for microcontrollers. It implements a POSIX-like API, the C standard library and the C++ standard library. It also provides a pre-emptive scheduler (among others) and a custom concurrency API with support for Threads and synchronization primitives.
A FAT32 filesystem, a serial port driver, platform timers and General Purpose I/O functionalities (GPIO) are also exposed through a custom API.

We maintain a fork of the Miosix OS to fit our needs: [skyward/miosix-kernel](https://git.skywarder.eu/avn/swd/miosix-kernel)

The project is built with the CMake build system. We also have our own script that is built on top of CMake, to automate compilation and deployment to different hardware targets, [SBS](https://git.skywarder.eu/avn/swd/skyward-boardcore/wikis/Skyward-Build-System-(SBS)).

## Content

| Path             | Description                                                 |
| ---------------- | ----------------------------------------------------------- |
| src/bsps/		   | Board Support Packages (BSP)                                |
| src/shared/      | Device drivers and general-purpose classes/utilities        |
| src/entrypoints/ | Entry points for general tasks (e.g. calibration)           |
| src/tests/       | Entry points for on-device unit testing                     |
| build/           | Build output directory                                      |
| libs/            | External libraries (git submodules)                         |
| scripts/         | Various tools and utilities (e.g. linting, log decoder)     |

## Getting Started

### Dependencies

* `CMake` 3.25
* `Git`
* `Miosix` Toolchain

We also recommend to install `Ccache`, `Ninja`, `OpenOCD`, `Cppcheck`,`ClangFormat` and `pre-commit` for a smoother development experience.

### Cloning the repo

Clone the repository with the `--recursive` option:
```sh
git clone --recursive https://git.skywarder.eu/avn/swd/skyward-boardcore.git
cd skyward-boardcore
```

## Building

Using the SBS script is the recommended way to build the project. When called with no arguments, it will build all targets:
```sh
./sbs
```

This may take a while depending on how many targets there are. If `ccache` is installed, it will be used to cache intermediate compilation artifacts to speed-up incremental builds.

Alternatively, you can build with CMake commands:
```sh
mkdir build
cd build
cmake -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_TOOLCHAIN_FILE=../libs/miosix-kernel/miosix/_tools/toolchain.cmake -GNinja ..
cmake --build .
```

## Documentation

The code is documented with Doxygen, check it out [here](http://avn.pages.skywarder.eu/swd/skyward-boardcore).

## Contributing

You can install a pre-commit hook to ensure changes to the code will pass CI:

```sh
pre-commit install
```

## What's next?

You can find first-step **guides** in the [Wiki](https://git.skywarder.eu/avn/swd/skyward-boardcore/wikis/home) (configuring the IDE, building examples, etc.) as well our **code guidelines** and **best practices** that we follow.

If you want to contribute to this repository, please read our [Git Workflow](https://git.skywarder.eu/avn/swd/skyward-boardcore/wikis/Git-Workflow).

If you want to start messing around with the code, check out the [LED blink guide](https://git.skywarder.eu/avn/swd/skyward-boardcore/-/wikis/LED-Blink).
