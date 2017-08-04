# sdl_implementation_reference
## Build Instruction

### 1. Install the compiler.
#### a) Download the package of arm-linux-gnueabihf-gcc
The compiler is [gcc-linaro-5.4.1-2017.05-x86_64_arm-linux-gnueabihf.tar.xz](https://releases.linaro.org/components/toolchain/binaries/5.4-2017.05/arm-linux-gnueabihf/gcc-linaro-5.4.1-2017.05-x86_64_arm-linux-gnueabihf.tar.xz), please download and extract it, and copy to /opt directory as administrator. 
```shell
$wget -c https://releases.linaro.org/components/toolchain/binaries/5.4-2017.05/arm-linux-gnueabihf/gcc-linaro-5.4.1-2017.05-x86_64_arm-linux-gnueabihf.tar.xz
$tar -xvf gcc-linaro-5.4.1-2017.05-x86_64_arm-linux-gnueabihf.tar.xz
$sudo mv gcc-linaro-5.4.1-2017.05-x86_64_arm-linux-gnueabihf /opt/arm-linux-gnueabihf-5.4.1
```
#### b) Setup the environment variable
Open the file .bashrc in your home directory, append the string:

export PATH=$PATH:/opt/arm-linux-gnueabihf-5.4.1/bin

then update environment variable and test the compiler version
```shell
$source .bashrc
$arm-linux-gnueabihf-gcc --version
```

#### c) if can not find compiler, say "No such file or directory.", please install 32 bits library.
```shell
sudo apt-get install lib32ncurses5 lib32z1 lib32stdc++6
 cp -rf libbluetooth/usr/lib/* /opt/arm-linux-gnueabihf-5.4.1/arm-linux-gnueabihf/lib/

### 2. Build and Run

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
