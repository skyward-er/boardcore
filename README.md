<p align="center">
	<img src="https://avatars2.githubusercontent.com/u/8077370?s=200&v=4" alt="Skyward" width="200"></a>
</p>
<h2 align="center">Skyward Boardcore</h2>
<p align="center">
	<a href="https://git.skywarder.eu/avn/swd/skyward-boardcore/-/pipelines"><img src="https://git.skywarder.eu/avn/swd/skyward-boardcore/badges/main/pipeline.svg"></a>
</p>

Boardcore is a framework for developing and building rockets software for custom boards with Miosix.

[Miosix](https://miosix.org/) is a lightweight OS for embedded developing which provides support for basic things such as Threads, GPIO, Time and many other. You can find our fork of the kernel here: [skyward/miosix-kernel](https://git.skywarder.eu/avn/swd/miosix-kernel)

Building is made with [SBS](https://git.skywarder.eu/avn/swd/skyward-boardcore/wikis/Skyward-Build-System-(SBS)), which was created to easily compile and reuse code for different boards.

### Content

| Path             | Description                                                 |
| ---------------- | ----------------------------------------------------------- |
| src/shared/      | Objects, drivers and other stuff written by us              |
| src/entrypoints/ | Each file here is a 'main'                                  |
| src/tests/       | Contains the 'main' of every test                           |
| build/           | Compiled binaries that can be flashed on a target board     |
| libs/            | External libs (Miosix kernel and others as a git submodule) |
| scripts/         | Some tools (e.g. script for flashing on the boards)         |

In the main folder you will find **CMakeLists.txt** which is used to configure the build system.

### Getting Started

#### Dependencies

* CMake
* Git
* Miosix Toolchain

Also, Ccache, Ninja, OpenOCD, Cppcheck, clang-format and pre-commit are recommended for a better experience.

#### Cloning the repo

Clone this repo with the `--recursive` option.
```sh
git clone --recursive https://git.skywarder.eu/avn/swd/skyward-boardcore.git
cd skyward-boardcore
```

### Building

You can build everything using the SBS script:
```sh
./sbs
```

The build system will start building all the targets (entrypoints and tests). Depending on how many targets there are, this operation may take several minutes. If you installed ccache, subsequent runs will be much faster.

Or you can use directly CMake:
```sh
mkdir build
cd build
cmake -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_TOOLCHAIN_FILE=../libs/miosix-kernel/miosix/_tools/toolchain.cmake -GNinja ..
cmake --build .
```

### Contributing

You can install a pre-commit hook to ensure changes to the code will pass CI:

```sh
pre-commit install
```

### What's next?

In the [Wiki](https://git.skywarder.eu/avn/swd/skyward-boardcore/wikis/home) you will find some first-steps **guides** (configuring the IDE, building a firmware, etc.) as well as the **coding guidelines** and some **best practices** we adopt.

If you want to contribute to this repository, please read [Git Workflow](https://git.skywarder.eu/avn/swd/skyward-boardcore/wikis/Git-Workflow).

If you just want to start messing around, try [this](https://git.skywarder.eu/avn/swd/skyward-boardcore/-/wikis/LED-Blink).
