# Mavlink Driver

:warning: NOTE: This driver uses mavlink helper struct and functions that are present in ANY mavlink library,
but the mavlink library itself IS NOT included in boardcore. Hence, this will not compile alone.

You should provide your own implementation of the mavlink messages in the mavlink_skyward_lib repository,
then include it in your project and add PATH_TO_MAV_SKYWARD_LIB to the includes.

For example

* you create a new repo `my-board-repo` and add `skyward-boardcore` as submodule
* you go to the `mavlink_skyward_lib` repository and add your own branch `mybranch` there
* you add `mavlink_skyward_lib`/`mybranch` in `my-board-repo` as a submodule
* now you can use the driver, because it will find the mavlink header that you have