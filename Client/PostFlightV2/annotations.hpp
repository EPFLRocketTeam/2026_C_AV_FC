#pragma once

#if defined(__glibcxx_reflection) && __glibcxx_reflection >= 202506L
#include <string_view>

namespace csv {

struct ignore_t {};
inline constexpr ignore_t ignore{};

struct rename {
    char value[32]{};

    constexpr rename(const char *s) {
        std::size_t i = 0;
        while (s[i] != '\0' && i < 31) { value[i] = s[i]; ++i; }
    }

    constexpr std::string_view name() const {
        std::size_t len = 0;
        while (len < 32 && value[len] != '\0') ++len;
        return std::string_view(value, len);
    }
};
template<typename T>
struct decode_with {
  using target = T;

  constexpr decode_with () {}
};

}

  #define CSV_IGNORE [[=csv::ignore]]
  #define CSV_RENAME(name) [[=csv::rename(name)]]
  #define CSV_DECODE_WITH(type) [[=csv::decode_with<type>()]]
#else
  #define CSV_IGNORE
  #define CSV_RENAME(name)
  #define CSV_DECODE_WITH(type)
#endif
