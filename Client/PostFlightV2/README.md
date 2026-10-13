
Compiling the postflight client requires having `g++-16` installed (personnally version 16.0.1 20260315 experimental). It should work on later versions of `g++-16` but if it doesn't, try using that one.

The compilation command is the following :
```sh
g++-16 -o out main.cpp -freflection -std=c++26 -I../../ -I. -DUNIT_TEST_ENV ../../Application/Data/*.cpp ../../Application/Data/Stores/*.cpp ../../Drivers/STM32HAL/Simulations/*.cpp -fpermissive
```

One may generate a sample `log.bin` file using the following command:
```sh
g++-16 -o writer test_writer.cpp -freflection -std=c++26 -I../../ -I. -DUNIT_TEST_ENV ../../Application/Data/*.cpp ../../Application/Data/Stores/*.cpp ../../Drivers/STM32HAL/Simulations/*.cpp && ./writer
```
