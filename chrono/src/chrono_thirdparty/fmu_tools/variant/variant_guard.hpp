#ifndef VARIANT_GUARD_HPP
#define VARIANT_GUARD_HPP

#if _HAS_CXX17
    #include <variant>
    namespace varns = std;
#else
    #include "variant.hpp"
    namespace varns = mpark;
#endif

#endif  // VARIANT_GUARD_HPP
