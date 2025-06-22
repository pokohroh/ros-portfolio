# ROS Tutorial: Creating a ROS msg and srv

## Objective
- Covers how to create aand build msg and srv files.
- Utilizing `rosmsg`, `rossrv`, `roscp`

## 1. Introduction to msg and srv
* msg: These are simple text files that define the parts (fields) of a ROS message. ROS uses them to create message code in different programming languages.

* srv: These files describe a service in ROS. They have two sections: one for the request and one for the response.

msg files are kept in the msg folder inside a package, and srv files are kept in the srv folder. msg files are simple text files where each line lists a field type and a field name.

You can use these field types in a msg file:

* Integers: int8, int16, int32, int64 (and their unsigned versions like uint8)
* Floating-point numbers: float32, float64
* string
* time and duration
* Other message types (msg files):
  * Arrays: variable-length (with [ ]) and fixed-length (like [C], where C is a number)
* There is also a special type in ROS: Header, the header contains a timestamp and coordinate frame information that are commonly used in ROS. You will frequently see the first line in a msg file have `Header header`.

## 2. Using msg

Create a new msg in the beginner_tutorials package that was created in the previous tutorial.

```bash
$ roscd beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```
The example `.msg` file shown earlier had just one line, but you can make it more detailed by adding more fields one per line like this:

```bash
string first_name
string last_name
uint8 age
uint32 score

```

We need to make sure the `.msg` files are converted into source code (like for Python or other languages).

To do that, open the package.xml file and check that these two lines are included and not commented out (make sure they don’t have <!-- --> around them):

```YAML
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

 > Note that at build time, we need **"message_generation"**, while at runtime, we only need **"message_runtime"**.

Open `CMakeLists.txt` in your text editor.

In your CMakeLists.txt file, look for the `find_package` line , it should already be there. To make sure your messages get generated, you need to add `message_generation` to the list of COMPONENTS. It should look like this:

> Note: Do not just add this to your CMakeLists.txt, modify the existing text to add message_generation before the closing parenthesis

```YAML
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```
Sometimes your project might build correctly even if you didn’t include all the needed find_package calls. This happens as catkin builds all packages together, so if another package already included it, yours might still work.

But if you try to build your project by itself later, it could fail. It is important to include all the required `find_package` calls yourself.

Also, don’t forget to export the message runtime dependency to make sure other packages can use your messages when needed.

```YAML
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)

```
Find the following block of code:

```YAML
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
Uncomment it by removing the # symbols and then replace the stand in `Message*.msg` files with your `.msg` file:

```YAML
add_message_files(
  FILES
  Num.msg
)
```
By adding the `.msg` files manually, we make sure that CMake knows when it has to reconfigure the project after you add other .msg files.

Now we must ensure the `generate_messages()` function is called.

You need to uncomment these lines:

```YAML
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
```
It should look like this:

```YAML
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
Now it is ready to generate source files from your msg definition. 

## 3. Using rosmsg
 To create a msg we use the `rosmsg` command. ROS can see it using the `rosmsg show `command.

Example:

```bash
rosmsg show beginner_tutorials/Num
```

Output should show this:

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
## 4. Using srv

Creating a srv. Use the package we just created to create a srv:

```bash
roscd beginner_tutorials
mkdir srv
```
Instead of writing a new .srv file from scratch, we can save time by copying one from another package. To do that, you can use the roscp command — it’s a handy tool for copying files between ROS packages.

Usage:

`roscp [package_name] [file_to_copy_path] [copy_path]`

Now we can copy a service from the `rospy_tutorials` package:

`roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv`

To use .srv files in your code (like Python or C++), ROS needs to convert them into source code.

If you haven’t done this yet, open your package.xml file and make sure these two lines are there and not commented out (no <!-- --> around them):

```YAML
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```
* You need message_generation during build time (to generate the code)

* You need message_runtime during runtime (when your code is actually running).

* If you did not add this while setting up messages, go to your CMakeLists.txt and update the existing find_package line, do not add a new one. Just include `message_generation` in the COMPONENTS list so the build system knows to generate the service code.

```YAML
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```
> (Despite its name, message_generation works for both msg and srv.)

For services, you need to make the same changes in package.xml as you did for messages.
That means adding the same dependencies, look above to see which ones you need to include and make sure they are not commented out.

**Remove # to uncomment the following lines:**

```YAML
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

```
And replace the placeholder Service `.srv` files for your service files:

```YAML
add_service_files(
  FILES
  AddTwoInts.srv
)

```

## 5. Using rossrv

`rossrv` is a command-line tool in ROS that helps you work with service definitions (.srv files).
You can use it to:

* View the structure of a service (rossrv show)
* Find out which package a service belongs to (rossrv find)
* List all available services (rossrv list)
  
Example:

```bash
rossrv show beginner_tutorials/AddTwoInts
```

You will see an output like this:

```bash
int64 a
int64 b
---
int64 sum
```

To find service files like this without specifying package name:

```bash
$ rossrv show AddTwoInts
[beginner_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum
```
Here, you can see two services listed. The first one is the service you just created in the `beginner_tutorials` package, and the second one is a built-in service from the `rospy_tutorials` package.

## 6. msg and srv

Unless you have already done this in the previous steps, change in CMakeLists.txt. :

```YAML
# generate_messages(
#   DEPENDENCIES
# #  std_msgs  # Or other packages containing msgs
# )
```
Uncomment it and add any packages you depend on which contain .msg files that your messages use (in this case std_msgs), such that it looks like this:

```YAML
generate_messages(
  DEPENDENCIES
  std_msgs
)

```
Now that we have made some new messages we need to make our package again in your catkin workspace:

```bash
$ roscd beginner_tutorials
$ cd ../..
$ catkin_make
$ cd -
```

* Any .msg files in the msg folder will be automatically turned into code for all supported programming languages.

* For Python, the message files will be in ~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg/.

* Similarly, .srv files in the srv folder will be converted into code too. For Python there will be an srv folder next to the msg folder.

## 7. ROS Help tools
It is easy to forget what arguments each command needs. So, most ROS tools have built-in help you can use to check how to use them.

Try this command: 

```bash
rosmsg -h
```
You should see a list of different rosmsg subcommands:

```bash
  rosmsg show     Show message description
  rosmsg list     List all messages
  rosmsg md5      Display message md5sum
  rosmsg package  List messages in a package
  rosmsg packages List packages that contain messages
```
