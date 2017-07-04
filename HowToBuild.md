# sdl_implementation_reference
## Build Instruction

### 1. Install the compiler.

We should use an cross compiler which requires the version lower than 4.9, otherwise the program can not run.

#### a) if your system support default package, please use follow command to install the compiler
```shell
sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```
#### b) otherwise, use the local package. Here is [gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.xz](https://releases.linaro.org/archive/14.04/components/toolchain/binaries/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.xz), please extract it, and copy to /opt directory as administrator. 
then set the PATH to your bash system. Open the file .bashrc in your home directory, append the string:
```shell
export PATH=$PATH:<path to the compiler you just copied>
```
then restart your bash window, and now it should work.
Here to download the cross compiler:
Link:http://pan.baidu.com/s/1i542ohn Key: xwqs

#### c) if can not find compiler, say "No such file or directory.", please install 32 bits library.
```shell
sudo apt-get install lib32ncurses5 lib32z1 lib32stdc++6
```
### 2. Install the bluetooth compiler.

#### a) Download the installation package from "http://www.filewatcher.com/d/Debian/armhf/libdevel/libbluetooth-dev_4.99-2_armhf.deb.111400.html"

#### b) Please extract libbluetooth-dev_4.99-2_armhf.deb,and copy to /opt directory as administrator.

#### c) Set the PATH to your bash system. Open the file .bashrc in your home directory, append the string:
```shell
export CPLUS_INCLUDE_PATH=<path to the compiler you just copied>	
```
### 3. Build and Run

#### a) Get source code.
```shell
git clone https://github.com/luwanjia/sdl_implementation_reference.git
```
#### b) Generate project by cmake
```shell
$cd sdl_implementation_reference
$mkdir build
$cd build
$cmake -DCMAKE_BUILD_TYPE="Release" ..
$make
$make install
```	
After those operations, the binary package will be created, copy the bin folder to you embedded linux system, and run.
