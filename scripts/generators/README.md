# Generators scripts

The scripts contained in this directory are used to generate cpp code from scxml files, they accept a directory or a list of files.

## Eventgen

The Eventgen program generates the files `Events.h`, `Topics.h` and `EventString.cpp` gathering information from all scxml files specified.

```shell
usage: eventgen.py [-h] [-q] [-f [F [F ...]]] [directory]

positional arguments:
  directory             Directory where to search files

optional arguments:
  -h, --help            show this help message and exit
  -q, --quiet           Output only essential messages
  -f [F [F ...]], --files [F [F ...]]
```

## FSMGen

The FSMGen program generates cpp files (`FSMController.h`, `FSMController.cpp`, `FSMData.h` and `test-FMS.cpp`) for each scxml file specified.

```shell
usage: fsmgen.py [-h] [-q] [-a AUTHORS] [-n MAIN_NAMESPACE] [-f [F [F ...]]] [directory]

positional arguments:
  directory             Directory where to search files

optional arguments:
  -h, --help            show this help message and exit
  -q, --quiet           Output only essential messages
  -a AUTHORS, --authors AUTHORS
  -n MAIN_NAMESPACE, --main_namespace MAIN_NAMESPACE
                        DeathStackBoard as default
  -f [F [F ...]], --files [F [F ...]]
```
