# ROS Tutorial: Using rosed to edit files

# Objective
- Introduce using `rosed` to make editing easier in ROS workspaces.

# 1. Using rosed
`rosed` is a tool that comes with rosbash. It lets you edit files inside a ROS package by using just the package name, so you don’t have to type the full file path.

Usage:

` rosed [package_name] [filename]`

Enter the following commmand:

```bash
rosed roscpp Logger.msg
```

This example shows how to edit the Logger.msg file inside the roscpp package using rosed.
If it does not work, it might be because you do not have the vim editor installed. If there is more than one file with the same name in the package, a menu will pop up asking you to pick which file you want to edit.

# 2. Using rosed with tab completion
This way you can easily see and optionally edit all files from a package without knowing its exact name.

```bash
$ rosed roscpp <tab><tab>
                 
```
# 3. Editor

By default, rosed uses the vim editor. If you prefer a simpler editor, nano comes pre-installed on Ubuntu and is easier for beginners. To make rosed use nano instead, you can add this line to your `~/.bashrc` file:

```YAML
export EDITOR='nano -w'
````
To set the default editor to emacs you can edit your `~/.bashrc` file to include:

```YAML
export EDITOR='emacs -nw'
````

> Changes you make in .bashrc only apply to new terminal windows. If you keep using terminals that were already open, they won’t have the updated settings.

Open a new terminal and see if EDITOR is defined:

```bash
$ echo $EDITOR
nano -w
or
emacs -nw
```
