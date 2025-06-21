# ROS Tutorial: Using rosed to edit files

# Objective
- Introduce using `rosed` to make editing easier in ROS workspaces.

# Hardware Required
- None

# Software Required
- Ubuntu 18.04
- ROS Melodic
- Terminal
- Properly built and sourced catkin workspace

# 1. Using rosed
rosed is part of the rosbash suite. It allows you to directly edit a file within a package by using the package name rather than having to type the entire path to the package.

Usage:


` rosed [package_name] [filename]`

Example:

`rosed roscpp Logger.msg`

This example demonstrates how you would edit the Logger.msg file within the roscpp package.

If this example doesn't work it's probably because you don't have the vim editor installed. If the filename is not uniquely defined within the package, a menu will prompt you to choose which of the possible files you want to edit.


# 2. Using rosed with tab completion
This way you can easily see and optionally edit all files from a package without knowing its exact name.

Usage:

`$ rosed [package_name] <tab><tab>`

Example:
```
$ rosed roscpp <tab><tab>
Empty.srv                   package.xml
GetLoggers.srv              roscpp-msg-extras.cmake
Logger.msg                  roscpp-msg-paths.cmake
SetLoggerLevel.srv          roscpp.cmake
genmsg_cpp.py               roscppConfig-version.cmake
gensrv_cpp.py               roscppConfig.cmake
msg_gen.py                  
```
# 3. Editor
The default editor for rosed is vim. The more beginner-friendly editor nano is included with the default Ubuntu install. You can use it by editing your ~/.bashrc file to include:

`export EDITOR='nano -w'`

To set the default editor to emacs you can edit your `~/.bashrc` file to include:

`export EDITOR='emacs -nw'`

**NOTE**: changes in .bashrc will only take effect for new terminals. Terminals that are already open will not see the new environmental variable.

Open a new terminal and see if EDITOR is defined:

```
$ echo $EDITOR
nano -w
or
emacs -nw
```
