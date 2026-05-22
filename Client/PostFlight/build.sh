
python3 rules_generator.py rules.yaml > csv_rules.hpp
g++-16 -std=c++26 -freflection -o extract_logs extract_logs.cpp ../../Application/Data/data.cpp ../../Application/Data/gps_store.cpp ../../Application/Data/Stores/*.cpp ../../Drivers/STM32HAL/Simulations/*.cpp -DUNIT_TEST_ENV -I./pfr/include -I../../ -I.
