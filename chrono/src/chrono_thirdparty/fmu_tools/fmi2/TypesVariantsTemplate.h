#ifndef TYPESVARIANTS_H
#define TYPESVARIANTS_H

#include <string>

#include "variant/variant_guard.hpp"
#include "fmi2/fmi2TypesPlatformCustom.h"


#define FMITYPESPLATFORM_DEFAULT


// variants does not allow repeated types;
// unfortunately, in the default implementation, both fmi2Boolean and fmi2Integer point to a int type
// that's why, by default, FmuVariable::PtrType and FmuVariable::StartType does not have both of them
// however, in case the user has its own custom types definition, it is required to define new valid variants
// in order to cover all possible available types

/// FMU_ACTION: update the following declarations according to "fmi2TypesPlatformCustom.h"
using FmuVariableBindType = varns::variant<fmi2Real*, fmi2Integer*, fmi2Boolean*, fmi2String*,
                                            std::function<fmi2Real()>,
                                            std::function<fmi2Integer()>,
                                            std::function<fmi2Boolean()>,
                                            std::function<fmi2String()>>;
using FmuVariableStartType = varns::variant<fmi2Real, fmi2Integer, fmi2Boolean, std::string>;

#endif
