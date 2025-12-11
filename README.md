# 2026_C_AV_FC
Hosts the code of Firehorn II Avionics Flight Computer

# Contributing

## STM32CubeIDE Installation

To contribute to this repository, you should first have a recent version of STM32CubeIDE. Once you have
it installed, clone this repository and use the `File > New > STM32 Project from an Existing .ioc file`
and select the file `2026_C_AV_FC.ioc` (the name of the directory in which you cloned should be `2026_C_AV_FC`).
Verify that it properly opens the project, and click on `Project > Generate Code`. After that, you should be
able to press on the hammer with `1 Debug` to compile the codebase for the microcontroller. You should then
be able to upload it to the STM32 directly if everything worked well.

## Unit Testing Installation

To contribute to this repository, you should also be ready to add some unit tests that are run on your machine
instead of on the microcontroller. To do so, we use a custom CMake that operates independently from the
STM32CubeIDE project. You should have the following packets installed for it to work :

```
sudo apt-get install -y g++ cmake valgrind lcov
```

To prepare the build process, first create a folder called `build` and `cd` into it. You can now configure
the build process by running `cmake .. -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON`. You may now build
the codebase by using `cmake --build .`.

Finally you can run tests in coverage mode or memory check mode by running either
`cmake --build . --target coverage` or `cmake --build . --target memcheck`. The coverage mode generates a
folder `html_report` with an `index.html` file that you can open to view the code coverage (line, function
and branches) of your application. The memory check mode allows you to verify that your code does not do
any memory invalid operations (such as index out of bounds, or double free or use after free). The last
two are less important as no dynamic memory should be used on STM32 (you may use it in specific unit test
helpers if you need, but they should still be memory safe).

The github actions run use the following script to build all and test coverage and memory.
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON
cmake --build .
cmake --build . --target coverage
cmake --build . --target memcheck
```

## Writing a Driver

If you want to write the driver for a sensor called `X`, the first thing you should do is
to create a folder called `Driver/X`. Inside it, you should place at least 2 files, `X.h`
(or `X.hpp`) which defines the interface of your Driver, `Impl/X.c` (or `Impl/X.cpp`).

**WARNING**: Your file `X.h` should *NEVER* import directly anything that starts with
`stm32h7xx.h` or `main.h`, and should instead import `stm32hal.h`. This file is used
to create fake types (e.g. for hardware handles) for your interface so your file can
compile on your machine.

Then, you can add more files for your driver based of the following table and in
order of priority :
1. Hardware Manual Tests (Folder `Tests/Manual/`): It should contain .h files with their .c file containing a manual test.
2. Mock implementations: File `Impl/X_mock.cpp` with an interface `Impl/X_mock.h` to give the data to the mock.
3. Mock Unit Tests (Folder `Tests/Mock/`): It should contain .cpp files forr tests of the Mock Interface.

Upon writing the mock implementation, you should add you driver to the flight computer source files inside
the root `CMakeLists.txt` (in the Flight Computer section). You should also add your driver's directory 
in the directories to include (also in the Flight Computer section).

All Unit Tests should be written using the googletest framework. You can see an example of how to write such
a test in the `Application/Tests` folder. You should also create a `CMakeLists.txt` in your unit test folder.
Inside it you should put a `create_test` for every C++ test file (the syntax is
`create_test(<executable name> <test file>)`, again you can see the example in the `Application/Tests` folder).
