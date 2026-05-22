
#include "bits/stdc++.h"
#include <meta>
#include <cstdint>
#include "../../Application/Modules/sd_logger_types.hpp"
#include "../../Application/Data/data.hpp"
#include "Application/Kalman/kalman/eskf_logger.hpp"

using namespace std;
using namespace flight_computer;

std::string push (const std::string &prefix, const std::string &suffix) {
    return prefix.empty() ? suffix : (prefix + "." + suffix);
}

template<typename T>
void generate_csv (std::stringstream &stream, const std::string &prefix, const T& obj, bool header, bool is_first) {
    if (!is_first) stream << ",";
    if (header) stream << prefix;
    else stream << obj;
}

#define GENERATE_FIELD(field) { \
    using FieldType = decltype(obj.field); \
    generate_csv<FieldType>(stream, push(prefix, #field), obj.field, header, is_first); \
    is_first = false; \
}

#define GENERATE_FIXED_FIELD(FieldType, field) { \
    generate_csv<FieldType>(stream, push(prefix, #field), obj.field, header, is_first); \
    is_first = false; \
}
#define GEN_CSV(T) template<> void generate_csv<T> (std::stringstream &stream, const std::string &prefix, const T &obj, bool header, bool is_first)
