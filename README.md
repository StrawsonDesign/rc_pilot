# Documentation
[Documentation for rc_pilot](http://www-personal.umich.edu/~ghaggin/rc_pilot/)

# Setup
## Install dependencies on Beaglebone:

```shell
$ sudo apt install libjson-c-dev libjson-c3
```

## Install librobotcontrol on Beaglebone:

The repo for the library is located on the [Strawson Design](https://github.com/StrawsonDesign/librobotcontrol) GitHub page. There is a bug in the 1.0.4 release version (```rc_duplicate_filter``` fails for initialized filter) so [install librobotcontrol from source](http://strawsondesign.com/docs/librobotcontrol/installation.html).


## Transfer rc_pilot source to Beaglebone:
In the scripts folder there are some convenient scripts for moving files to and from the vehicle.  These scripts were build and tested on Ubuntu 19.04 but have not been tested on any other system.  They should work on any system with bash and rsync.  To transfer the files to the vehicle run:

```shell
$ ./scripts/transfer_rcpilot [usb|wifi]
```

## Build on Beaglebone (from project directory):

```shell
$ make
```

## Run RC_pilot:

```shell
$ cd bin
$ sudo ./rc_pilot -s ../settings/quad_settings.json
```

# Miscellaneous
## Building docs:

Make sure that doxygen is installed on your system and run

``` shell
$ make docs
```

Open the docs with the web browser of your choice

```shell
$ firefox docs/html/index.html
$ google-chrome docs/html/index.html
```

## (COMING SOON) Testing:
Test cases are written using the [Boost 1.66](https://www.boost.org/users/history/version_1_66_0.html) testing suite.  Look at make target test for system install location.  This suite is not written with portability in mind so it may take some tweaking to get it to work right now.  Typically, installing boost with the option ```--prefix=/usr/local``` should put the libraries and headers in the correct location but don't count on it.

Run the test suit with the make target:
```shell
$ make test
```

### Adding Test Modules

To add a test modules, create a new file <module_name>_test.cpp and format it as follows:

```c++
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUIT(Test_<module_name>)

BOOST_AUTO_TEST_CASE(test1){...}

BOOST_AUTO_TEST_CASE(test2){...}

...

BOOST_AUTO_TEST_CASE_END()
```

The make target will compile each test module into a single executable and run all test cases simultaneously.