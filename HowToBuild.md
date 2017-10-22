# sdl_implementation_reference
## Build Instruction

### 1. Install the compiler.

#### a) System requirement
Your operating system should be Ubuntu16.04 x64, other OS may be not compatible.

#### b) Install gcc-arm-linux-gnueabihf and g++-arm-linux-gnueabihf by apt
```shell
$sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```
#### c) Install support packages for 32-bit architecture
```shell
$sudo apt-get install lib32ncurses5 lib32z1 lib32stdc++6
```

### 2. Install all the packages for sdl dependence
We saved all the packages on the github, you just need download and install as follow.
```shell
$git clone https://github.com/luwanjia/sdl_libraries.git
$cd sdl_libraries
$sudo ./install.sh
```
And you can also uninstall all the packages by uninstall.sh command.
```shell
$sudo ./uninstall.sh
```

### 2. Build and Run

#### a) Get source code.
```shell
$git clone https://github.com/luwanjia/sdl_implementation_reference.git
```
#### b) cmake && make && make install
```shell
$cd sdl_implementation_reference
$mkdir build
$cd build
$cmake -DCMAKE_BUILD_TYPE="Release" ..
$make
$make install
```	
After those operations, the binary package will be created, copy the bin folder to your embedded linux system, and run.
