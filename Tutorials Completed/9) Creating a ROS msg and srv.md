# ROS Tutorial: Creating a ROS msg and srv

## Objective
- Covers how to create aand build msg and srv files.
- Utilizing `rosmsg`, `rossrv`, `roscp`

1. Introduction to msg and srv
- `msg`: msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.

- `srv`: an srv file describes a service. It is composed of two parts: a request and a response.


msg files are stored in the msg directory of a package, and srv files are stored in the srv directory.


msgs are just simple text files with a field type and field name per line. The field types you can use are:

- int8, int16, int32, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files
- variable-length array[] and fixed-length array[C]

There is also a special type in ROS: Header, the header contains a timestamp and coordinate frame information that are commonly used in ROS. You will frequently see the first line in a msg file have `Header header`.

Here is an example of a msg that uses a Header, a string primitive, and two other msgs :

```
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
```

srv files are just like msg files, except they contain two parts: a request and a response. The two parts are separated by a '---' line. Here is an example of a srv file:

```
int64 A
int64 B
---
int64 Sum
```
In the above example, A and B are the request, and Sum is the response.


2. Using msg

**Creating a msg**

Let's define a new msg in the package that was created in the previous tutorial.

```
$ roscd beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```
The example `.msg` file above contains only 1 line. You can, of course, create a more complex file by adding multiple elements, one per line, like this:

```
string first_name
string last_name
uint8 age
uint32 score

```

There's one more step, though. We need to make sure that the msg files are turned into source code for Python, and other languages:

Open package.xml, and make sure these two lines are in it and uncommented:

```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```
Note that at build time, we need **"message_generation"**, while at runtime, we only need **"message_runtime"**.

Open `CMakeLists.txt` in your favorite text editor.

Add the **message_generation** dependency to the **find_package** call which already exists in your `CMakeLists.txt` so that you can generate messages. You can do this by simply adding message_generation to the list of COMPONENTS such that it looks like this:

Note Do not just add this to your CMakeLists.txt, modify the existing text to add message_generation before the closing parenthesis

```
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```
You may notice that sometimes your project builds fine even if you did not call find_package with all dependencies. This is because catkin combines all your projects into one, so if an earlier project calls find_package, yours is configured with the same values. But forgetting the call means your project can easily break when built in isolation.

Also make sure you export the message runtime dependency.

```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)

```
Find the following block of code:

```
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
Uncomment it by removing the # symbols and then replace the stand in `Message*.msg` files with your `.msg` file, such that it looks like this:

```
add_message_files(
  FILES
  Num.msg
)
```
By adding the `.msg` files manually, we make sure that CMake knows when it has to reconfigure the project after you add other .msg files.

Now we must ensure the generate_messages() function is called.

For ROS Hydro and later, you need to uncomment these lines:

```
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
```
so it looks like:

```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
In earlier versions, you may just need to uncomment one line:

`generate_messages()`

Now you're ready to generate source files from your msg definition. If you want to do so right now, skip next sections to Common step for msg and srv.

3. Using rosmsg
That's all you need to do to create a msg. Let's make sure that ROS can see it using the rosmsg show command.

Usage:

` rosmsg show [message type]`

Example:

` rosmsg show beginner_tutorials/Num`

You will see:

`int64 num`

In the previous example, the message type consists of two parts:

- **beginner_tutorials** -- the package where the message is defined
- **Num** -- The name of the msg Num.

If you can't remember which Package a msg is in, you can leave out the package name. Try:
`rosmsg show Num`

You will see:

```
[beginner_tutorials/Num]:
int64 num
```
4. Using srv

Creating a srv
Let's use the package we just created to create a srv:

```
roscd beginner_tutorials
mkdir srv
```

Instead of creating a new srv definition by hand, we will copy an existing one from another package. For that, roscp is a useful commandline tool for copying files from one package to another.

Usage:

`roscp [package_name] [file_to_copy_path] [copy_path]`

Now we can copy a service from the `rospy_tutorials` package:

`roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv`

We need to make sure that the srv files are turned into source code for Python, and other languages.
Unless you have done so already, open package.xml, and make sure these two lines are in it and uncommented:

```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```
As before, note that at build time, we need **message_generation**, while at runtime, we only need **message_runtime**. Unless you have done so already for messages in the previous step, add the message_generation dependency to generate messages in **CMakeLists.txt**:

**Do not just add this line to your CMakeLists.txt, modify the existing line**

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```
(Despite its name, message_generation works for both msg and srv.)

Also you need the same changes to package.xml for services as for messages, so look above for the additional dependencies required.

**Remove # to uncomment the following lines:**

```
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

```
And replace the placeholder Service*.srv files for your service files:

```
add_service_files(
  FILES
  AddTwoInts.srv
)

```


5. Using rossrv

Usage:
`rossrv show <service type>`

Example:

`rossrv show beginner_tutorials/AddTwoInts`

You will see:

```
int64 a
int64 b
---
int64 sum
```

Similar to rosmsg, you can find service files like this without specifying package name:

```
$ rossrv show AddTwoInts
[beginner_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum
```
Here, two services are shown. The first is the one you just created in the beginner_tutorials package, and the second is the pre-existing one from the rospy_tutorials package.

6. Common step for msg and srv

Unless you have already done this in the previous steps, change in CMakeLists.txt. :

```
# generate_messages(
#   DEPENDENCIES
# #  std_msgs  # Or other packages containing msgs
# )
```
Uncomment it and add any packages you depend on which contain .msg files that your messages use (in this case std_msgs), such that it looks like this:

```
generate_messages(
  DEPENDENCIES
  std_msgs
)

```
Now that we have made some new messages we need to make our package again:

```
# In your catkin workspace
$ roscd beginner_tutorials
$ cd ../..
$ catkin_make
$ cd -
```
An alternative to the catkin_make system is to use catkin build

```
# In your catkin workspace
$ roscd beginner_tutorials
$ cd ../..
$ catkin build
$ cd -
```
 - Any .msg file in the msg directory will generate code for use in all supported languages.
 - The C++ message header file will be generated in **~/catkin_ws/devel/include/beginner_tutorials/.** The Python script will be created in **~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg.**
 - The lisp file appears in **~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/msg/.**
 - Similarly, any .srv files in the srv directory will have generated code in supported languages. For C++, this will generate header files in the same directory as the message header files. For Python and Lisp, there will be an 'srv' folder beside the 'msg' folders.


7. Getting Help
 It can be difficult to keep track of what arguments each command requires. Luckily, most ROS tools provide their own help.

Try:

`rosmsg -h`
You should see a list of different rosmsg subcommands.

Commands:
```
  rosmsg show     Show message description
  rosmsg list     List all messages
  rosmsg md5      Display message md5sum
  rosmsg package  List messages in a package
  rosmsg packages List packages that contain messages
```
You can also get help for subcommands

` rosmsg show -h`

This shows the arguments that are needed for rosmsg show:

Usage: 

`rosmsg show [options] <message type>`

Options:
 ` -h, --help  show this help message and exit`
 ` -r, --raw   show raw message text, including comments`
