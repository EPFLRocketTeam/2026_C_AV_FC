
#include "./types.hpp"
#include <ostream>
#include <meta>
#include <type_traits>
#include <cstring>
#include <optional>

namespace csv {

template<typename T>
struct value {
    using type = T;

    const T& value;
    bool first = true;
};

};

#define X(type) \
    std::ostream& operator<< (std::ostream& os, const csv::value<type> &x);
X_PRIMITIVE_TYPES
#undef X

template<typename T>
    requires std::is_enum_v<T>
std::ostream& operator<< (std::ostream& os, const csv::value<T> &x);

template<typename T, size_t N>
std::ostream& operator<<(std::ostream &os, const csv::value<std::array<T, N>> &x);

template<typename T>
    requires std::is_class_v<T>
std::ostream& operator<<(std::ostream &os, const csv::value<T>& x);

#define CSV_VALUE_BASE_FN(type) \
    std::ostream& operator<< (std::ostream& os, const csv::value<type> &x) { \
        if (!x.first) os << ","; \
        os << x.value; \
        return os; \
    }

#define X(type) \
    CSV_VALUE_BASE_FN(type)
X_PRIMITIVE_TYPES
#undef X

template<typename T, size_t N>
std::ostream& operator<<(std::ostream &os, const csv::value<T[N]> &x);

template<typename T, size_t N>
std::ostream& operator<<(std::ostream &os, const csv::value<T[N]> &x) {
    for (size_t i = 0; i < N; i++) {
        os << csv::value<T>{ x.value[i], x.first && (i == 0) };
    }
    return os;
}

template<typename T>
    requires std::is_enum_v<T>
std::ostream& operator<<(std::ostream &os, const csv::value<T>& x) {
    if (!x.first) os << ",";

    static constexpr auto enumerators = std::define_static_array(
        std::meta::enumerators_of(^^T)
    );

    bool found = false;
    template for (constexpr auto e : enumerators) {
        if (!found && x.value == [:e:]) {
            os << std::meta::identifier_of(e);
            found = true;
        }
    }

    if (!found) {
        os << "UNKNOWN(" << ((int) x.value) << ")";
    }

    return os;
}

template<typename T, size_t N>
std::ostream& operator<<(std::ostream &os, const csv::value<std::array<T, N>> &x) {
    for (size_t i = 0; i < N; i ++) {
        os << csv::value<T>{ x.value[i], x.first && (i == 0) };
    }
    return os;
}

consteval std::meta::info decode_target_of(std::meta::info member) {
    for (auto anno : std::meta::annotations_of(member)) {
        auto t = std::meta::remove_cv(std::meta::type_of(anno));
        if (std::meta::has_template_arguments(t)
            && std::meta::template_of(t) == ^^csv::decode_with) {
            return std::meta::template_arguments_of(t)[0];
        }
    }
    return std::meta::info{};
}

template<typename Dst, typename Src>
struct cast_result { using type = Dst; };

template<typename Dst, typename Src, size_t N>
struct cast_result<Dst, Src[N]> {
    using type = std::array<typename cast_result<Dst, Src>::type, N>;
};

template<typename Dst, typename Src>
using cast_result_t = typename cast_result<Dst, Src>::type;

template<typename Dst, typename Src>
constexpr cast_result_t<Dst, Src> DoCast(const Src& val) {
    if constexpr (std::is_array_v<Src>) {
        cast_result_t<Dst, Src> out{};
        for (size_t i = 0; i < std::extent_v<Src>; i++)
            out[i] = DoCast<Dst>(val[i]);   // recurses for 2D arrays
        return out;
    } else {
        return static_cast<Dst>(val);
    }
}

template<typename T>
    requires std::is_class_v<T>
std::ostream& operator<<(std::ostream &os, const csv::value<T>& x) {
    bool first = x.first;

    static constexpr auto members =
        std::define_static_array(
            std::meta::nonstatic_data_members_of(^^T, std::meta::access_context::current())
        );

    template for (constexpr auto member : members) {
        constexpr bool skip = ([member] {
            for (auto anno : std::meta::annotations_of(member))
                if (std::meta::remove_cv(std::meta::type_of(anno)) == ^^csv::ignore_t)
                    return true;
            return false;
        })();
        constexpr std::meta::info decode_target = decode_target_of(member);

        if constexpr (!skip) {    
            using FieldT = [: std::meta::type_of(member) :];

            FieldT tmp;
            std::memcpy(&tmp,
                        reinterpret_cast<const unsigned char*>(&x.value)
                            + std::meta::offset_of(member).bytes,
                        sizeof(FieldT));

            if constexpr (decode_target != std::meta::info{}) {
                using TargetT = [: decode_target :];
                using TrueTargetT = cast_result_t<TargetT, FieldT>;
                const TrueTargetT decoded = DoCast<TargetT, FieldT>(tmp);
                os << csv::value<TrueTargetT>{ decoded, first };
            } else {
                os << csv::value<FieldT>{ tmp, first };
            }
            first = false;
        }
    }

    return os;
}
