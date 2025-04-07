## Demos
A short guide about packages and a few examples starting in the 'What is this section?', the talker and listener nodes demo is highly interesting for Lab 1. The talker publish messages to a topic and a subscriber reads these messages and prints it in the terminal. 
## First package
We start by making a directory using mkdir, move into it and then create the ROS2 package using ros2 pkg create. In the package, we create the empty file --node-name my_node, and the package name is my_package. You can see the file structure under Package Structure section below. 
To start, run the commands in the First package section below.


```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --node-name my_node my_package --dependencies ament_cmake_python rclpy
cd ~/ros2_ws
colcon build
source install/local_setup.bash
ros2 run my_package my_node
```

> **Note**: The cmake_python structure (using --build-type ament_cmake + Python) is much easier to maintain compared to pure Python packages and offers more flexibility when you eventually need to mix languages. While this structure supports both C++ and Python components in the same package, in this course we will be focusing exclusively on Python-based development.

### Modifying CMakeLists.txt for Python-Only Package

After creating a new package with `ros2 pkg create`, you'll get a C++-focused CMakeLists.txt. To make it Python-focused like our demos package, follow these steps:

### Package Structure (Python-Only)

For a Python ROS 2 package named `my_package`:

```
my_package/
├── CMakeLists.txt         # Modified build configuration
├── package.xml            # Package metadata
├── src/                   # Source directory for Python code
│   ├── __init__.py        # Makes src/ a Python package
│   ├── node.py            # Python nodes at root of src/
│   └── examples/          # Optional subdirectories for organization
│       ├── __init__.py    # Makes examples a subpackage
│       └── example_node.py # Example Python nodes
```

### Modifying the Auto-Generated CMakeLists.txt

1. Keep the required dependencies, and **remove** all the C++ build parts:

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_package)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# REMOVE all of these C++ parts:
# - add_executable()
# - target_include_directories()
# - target_compile_features()
# - ament_target_dependencies() for the C++ executable
# - install(TARGETS my_node...)
```

2. Add Python module installation:

```cmake
# Install Python modules
ament_python_install_package(src)

# Install Python executables
install(PROGRAMS
  src/node.py
  # Add other Python scripts as needed
  src/examples/example_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

3. Keep the testing section and ament_package() call.

### Setting Up Python Files

1. Create the Python package structure:

```bash
mkdir -p src
touch src/__init__.py
# For subfolders:
mkdir -p src/examples
touch src/examples/__init__.py
```

2. Create Python nodes with executable permissions:

```bash
# Basic node
touch src/node.py
chmod +x src/node.py
# Example node
touch src/examples/example_node.py
chmod +x src/examples/example_node.py
```

3. Add a shebang line at the top of each Python file:

```python
#!/usr/bin/env python3

# Rest of your Python code...
```

This creates a Python-focused ROS 2 package similar to our demos package.

## **What Is This?**

This package contains the following ROS 2 nodes:

1. `listener`
2. `talker`
3. `service_client`
4. `service_server`
5. `param_talker`
6. `config_reader`
7. `action_client`
8. `action_server`

## **Build**

While in your workspace root, run this command to build the package:

```bash
colcon build 
```

## **Run**

### Basic Listener & Talker

This runs `talker` and `listener` ROS 2 nodes which exchange the following string with incremeting integer:

> Hello World: <count_>

```bash
# Open new terminal
ros2 run demos talker.py
```

```bash
# Open new terminal
ros2 run demos listener.py
```

### Server & Client

This runs `service_client` and `service_server` ROS 2 client and server where the server processes two integers sent from the client and publishes its sum to the client to be printed out.

#### Server

```bash
# Open new terminal
ros2 run demos service_server.py
```

#### Client [Synchronous]

```bash
# Open new terminal
ros2 run demos service_client.py
```

### Parameter Usage

This runs the `param_talker` ROS 2 node which demonstrates basic parameter usage. This node is similar to the `talker` node but uses two parameters:

1. `message` - The base message to publish (default: "Hello World")
2. `timer_period` - How often to publish messages in seconds (default: 1.0)

```bash
# Run with default parameters
ros2 run demos param_talker.py
```

You can also run with custom parameter values:

```bash
# Change the message
ros2 run demos param_talker.py --ros-args -p message:="Hello ROS"

# Change the timer period (publish twice per second)
ros2 run demos param_talker.py --ros-args -p timer_period:=0.5

# Change both parameters
ros2 run demos param_talker.py --ros-args -p message:="Hello ROS" -p timer_period:=0.5
```

#### Using a Parameter File

You can also load parameters from a YAML file:

```bash
# Run with parameters from a YAML file
ros2 run demos param_talker.py --ros-args --params-file src/demos/config/params.yaml
```

### Parameter Files

The `config_reader` node demonstrates how to use parameter files with ROS 2:

```bash
# Run with default parameters
ros2 run demos config_reader.py

# Run with parameters from a config file
ros2 run demos config_reader.py --ros-args --params-file src/demos/config/params.yaml
```

This node is similar to the param_talker node but has two key differences:
1. It loads parameters from a separate YAML file
2. It actively checks for parameter changes in each timer cycle, enabling runtime updates

Unlike param_talker.py which only reads parameters at startup, config_reader.py checks for parameter changes each time its timer fires, allowing it to respond to parameter changes made during execution.

#### Changing Parameters at Runtime

You can change parameters while the config_reader node is running:

```bash
# Change the message
ros2 param set /config_reader message "Updated during runtime"

# Change the publishing rate
ros2 param set /config_reader timer_period 2.0
```

The node will automatically detect these changes and:
1. Use the updated message in the next published message
2. Recreate the timer with the new period if it has changed

This demonstrates how nodes can dynamically adapt to parameter changes without restarting.

#### Listing Parameters

You can also list and inspect parameters:

```bash
# List all parameters
ros2 param list /config_reader

# Get a specific parameter value
ros2 param get /config_reader message
```

### Actions

This runs the `action_client` and `action_server` ROS 2 nodes to demonstrate the ROS 2 Actions API. Actions are designed for long-running tasks that provide feedback during execution.

#### Server

```bash
# Open new terminal
ros2 run demos action_server.py
```

#### Client

```bash
# Open new terminal
ros2 run demos action_client.py
```

The action server processes a goal (count up to a specified number) with feedback during processing (current count) and a final result (success or failure status).
