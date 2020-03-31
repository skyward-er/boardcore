# Events Generator Script

This script generates the enums for all Events and Topics handled in a project, given a list of `scxml` files that describe all states and transitions. To execute the script:

`eventgen.py <LIST_OF_SCXML_FILES>`

The header files will be generated in the `generated/` folder.

Note that the SCXML files should contain transitions with the following form: *< TOPIC >.< EVENT >*.

