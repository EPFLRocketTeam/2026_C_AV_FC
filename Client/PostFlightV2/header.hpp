
#include "./types.hpp"
#include "./annotations.hpp"
#include <ostream>
#include <meta>
#include <type_traits>

namespace csv {

template<typename T>
struct header {
    using type = T;

    bool first = true;
    std::string field;
};

};

#define X(type) \
    std::ostream& operator<< (std::ostream& os, const csv::header<type> &x);
X_PRIMITIVE_TYPES
#undef X

template<typename T>
    requires std::is_enum_v<T>
std::ostream& operator<< (std::ostream& os, const csv::header<T> &x);

template<typename T, size_t N>
std::ostream& operator<<(std::ostream &os, const csv::header<std::array<T, N>> &x);

template<typename T>
    requires std::is_class_v<T>
std::ostream& operator<<(std::ostream &os, const csv::header<T>& x);

#define CSV_HEADER_BASE_FN(type) \
    std::ostream& operator<< (std::ostream& os, const csv::header<type> &x) { \
        if (!x.first) os << ","; \
        if (x.field == "") os << "value"; \
        else os << x.field; \
        return os; \
    }

#define X(type) \
    CSV_HEADER_BASE_FN(type)
X_PRIMITIVE_TYPES
#undef X


template<typename T>
    requires std::is_enum_v<T>
CSV_HEADER_BASE_FN(T)

template<typename T, size_t N>
std::ostream& operator<<(std::ostream &os, const csv::header<T[N]> &x);

template<typename T, size_t N>
std::ostream& operator<<(std::ostream &os, const csv::header<T[N]> &x) {
    for (size_t i = 0; i < N; i++) {
        os << csv::header<T>{ x.first && (i == 0), x.field + "[" + std::to_string(i) + "]" };
    }
    return os;
}

template<typename T, size_t N>
std::ostream& operator<<(std::ostream &os, const csv::header<std::array<T, N>> &x) {
    for (size_t i = 0; i < N; i ++) {
        os << csv::header<T>{ x.first && (i == 0), x.field + "[" + std::to_string(i) + "]" };
    }
    return os;
}

template<typename T>
    requires std::is_class_v<T>
std::ostream& operator<<(std::ostream &os, const csv::header<T>& x) {
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

        if constexpr (!skip) {
            constexpr std::string_view name_view = ([member]() -> std::string_view {
                for (auto anno : std::meta::annotations_of(member)) {
                    if (std::meta::remove_cv(std::meta::type_of(anno)) == ^^csv::rename) {
                        auto renamed = std::meta::extract<csv::rename>(anno);
                        return std::define_static_string(renamed.name());
                    }
                }
                return std::meta::identifier_of(member);
            })();

            std::string name = std::string(name_view);

            using FieldT = [: std::meta::type_of(member) :];

            os << csv::header<FieldT>{ first, (x.field == "") ? name : (x.field + "." + name) };
            first = false;
        }
    }

    return os;
}
