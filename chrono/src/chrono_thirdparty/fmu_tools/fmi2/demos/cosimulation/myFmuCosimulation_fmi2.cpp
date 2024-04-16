// =============================================================================
// Example FMU for co-simulation (FMI 2.0 standard)
// =============================================================================
//
// Illustrates the FMU exporting capabilities in fmu_tools (FmuToolsExport.h)
//
// =============================================================================

#define NOMINMAX

#include <cassert>
#include <map>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <math.h>

// #define FMI2_FUNCTION_PREFIX MyModel_
#include "myFmuCosimulation_fmi2.h"

// -----------------------------------------------------------------------------

// Implement function declarted in FmuToolsExport.h to create an instance of this FMU. 
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName,
                                             fmi2Type fmuType,
                                             fmi2String fmuGUID,
                                             fmi2String fmuResourceLocation,
                                             const fmi2CallbackFunctions* functions,
                                             fmi2Boolean visible,
                                             fmi2Boolean loggingOn) {
    return new myFmuComponent(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
}

// -----------------------------------------------------------------------------

// Construct the FMU component
myFmuComponent::myFmuComponent(fmi2String instanceName,
                               fmi2Type fmuType,
                               fmi2String fmuGUID,
                               fmi2String fmuResourceLocation,
                               const fmi2CallbackFunctions* functions,
                               fmi2Boolean visible,
                               fmi2Boolean loggingOn)
    : FmuComponentBase(
          instanceName,
          fmuType,
          fmuGUID,
          fmuResourceLocation,
          functions,
          visible,
          loggingOn,
          {{"logEvents", true},
           {"logSingularLinearSystems", true},
           {"logNonlinearSystems", true},
           {"logStatusWarning", true},
           {"logStatusError", true},
           {"logStatusPending", true},
           {"logDynamicStateSelection", true},
           {"logStatusDiscard", true},
           {"logStatusFatal", true},
           {"logAll", true}},
          {"logStatusWarning", "logStatusDiscard", "logStatusError", "logStatusFatal", "logStatusPending"}) {
    initializeType(fmuType);

    // During construction,
    // - list the log categories that this FMU should handle, together with a flag that specifies if it
    // they have to be enabled dy default;
    // - list, among the log categories, which are those to be considered as debug

    // Define new units if needed
    UnitDefinitionType UD_J("J");
    UD_J.kg = 1;
    UD_J.m = 2;
    UD_J.s = -2;
    addUnitDefinition(UD_J);

    // Declare relevant variables
    AddFmuVariable(&q_t[0], "x_tt", FmuVariable::Type::Real, "m/s2", "cart acceleration");
    AddFmuVariable(&q[0], "x_t", FmuVariable::Type::Real, "m/s", "cart velocity");
    AddFmuVariable(&q[1], "x", FmuVariable::Type::Real, "m", "cart position");
    AddFmuVariable(&q_t[2], "theta_tt", FmuVariable::Type::Real, "rad/s2", "pendulum ang acceleration");
    AddFmuVariable(&q[2], "theta_t", FmuVariable::Type::Real, "rad/s", "pendulum ang velocity");
    AddFmuVariable(&q[3], "theta", FmuVariable::Type::Real, "rad", "pendulum angle");
    auto& fmu_len = AddFmuVariable(&len, "len", FmuVariable::Type::Real, "m", "pendulum length",
                                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);
    auto& fmu_m = AddFmuVariable(&m, "m", FmuVariable::Type::Real, "kg", "pendulum mass",
                                 FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);
    auto& fmu_M = AddFmuVariable(&M, "M", FmuVariable::Type::Real, "kg", "cart mass",
                                 FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);

    /// One can also pass std::functions to get/set the value of the variable if queried
    // AddFmuVariable(std::make_pair(
    //     std::function<fmi2Real()>([this]() -> fmi2Real {
    //         return (0.5*(this->m*this->len*this->len/3)*(this->q_t[2]*this->q_t[2]));}),
    //     std::function<void(fmi2Real)>([this](fmi2Real) {})),
    //     "kineticEnergy",    FmuVariable::Type::Real, "J",    "kinetic energy");

    // same result is achieved through helper function 'MAKE_GETSET_PAIR'
    AddFmuVariable(
        MAKE_GETSET_PAIR(fmi2Real,
                         { return (0.5 * (this->m * this->len * this->len / 3) * (this->q_t[2] * this->q_t[2])); }, {}),
        "kineticEnergy", FmuVariable::Type::Real, "J", "kinetic energy");

    // Set name of file expected to be present in the FMU resources directory
    filename = "myData.txt";
    auto& fmu_Filename =
        AddFmuVariable(&filename, "filename", FmuVariable::Type::String, "kg", "additional mass on cart",
                       FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);

    auto& fmu_approximateOn =
        AddFmuVariable(&approximateOn, "approximateOn", FmuVariable::Type::Boolean, "1", "use approximated model",
                       FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);

    // Additional commands
    q = {0, 0, 0, 3.14159265358979323846 / 4};

    sendToLog("Resources directory location: " + std::string(fmuResourceLocation) + ".\n", fmi2Status::fmi2OK, "logAll");
}

// -----------------------------------------------------------------------------

std::array<double, 4> operator*(double lhs, const std::array<double, 4>& rhs) {
    std::array<double, 4> temp;
    temp[0] = rhs[0] * lhs;
    temp[1] = rhs[1] * lhs;
    temp[2] = rhs[2] * lhs;
    temp[3] = rhs[3] * lhs;
    return temp;
}

std::array<double, 4> operator+(const std::array<double, 4>& lhs, const std::array<double, 4>& rhs) {
    std::array<double, 4> temp;
    temp[0] = rhs[0] + lhs[0];
    temp[1] = rhs[1] + lhs[1];
    temp[2] = rhs[2] + lhs[2];
    temp[3] = rhs[3] + lhs[3];
    return temp;
}

double myFmuComponent::get_x_tt(double th_t, double th) {
    return (m * sin(th) * (len * th_t * th_t + g * cos(th))) / (-m * cos(th) * cos(th) + M + m);
}

double myFmuComponent::get_th_tt(double th_t, double th) {
    return -(sin(th) * (len * m * cos(th) * th_t * th_t + M * g + g * m)) / (len * (-m * cos(th) * cos(th) + M + m));
}

double myFmuComponent::get_x_tt_linear(double th_t, double th) {
    return (m * th * (len * th_t * th_t + g)) / M;
}

double myFmuComponent::get_th_tt_linear(double th_t, double th) {
    return -(th * (len * m * th_t * th_t + M * g + g * m)) / (len * M);
}

void myFmuComponent::get_q_t(const std::array<double, 4>& _q, std::array<double, 4>& q_t) {
    q_t[0] = approximateOn ? get_x_tt_linear(_q[2], _q[3]) : get_x_tt(_q[2], _q[3]);
    q_t[1] = _q[0];
    q_t[2] = approximateOn ? get_th_tt_linear(_q[2], _q[3]) : get_th_tt(_q[2], _q[3]);
    q_t[3] = _q[2];
}

// -----------------------------------------------------------------------------

void myFmuComponent::_enterInitializationMode() {}

void myFmuComponent::_exitInitializationMode() {
    std::string myfile_location = m_resources_location + "/" + filename;
    std::ifstream resfile(myfile_location);

    if (resfile.good()) {
        sendToLog("Successfully opened required file: " + myfile_location + ".\n", fmi2Status::fmi2OK, "logAll");
    } else {
        sendToLog("Unable to open required file: " + myfile_location + "; check if 'resources' folder is set",
                  fmi2Status::fmi2Fatal, "logStatusFatal");
        throw std::runtime_error("Unable to open required file: " + myfile_location);
    }

    double additional_mass = 0;
    if (resfile >> additional_mass) {
        M += additional_mass;
    } else {
        throw std::runtime_error("Expected number in: " + myfile_location);
    }

    resfile.close();
    sendToLog("Loaded additional cart mass " + std::to_string(additional_mass) + " from " + filename + ".\n",
              fmi2Status::fmi2OK, "logAll");
}

fmi2Status myFmuComponent::_doStep(fmi2Real currentCommunicationPoint,
                                   fmi2Real communicationStepSize,
                                   fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real _stepSize = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                                      std::min(communicationStepSize, m_stepSize));

        std::array<double, 4> k1 = {0, 0, 0, 0};
        std::array<double, 4> k2 = {0, 0, 0, 0};
        std::array<double, 4> k3 = {0, 0, 0, 0};
        std::array<double, 4> k4 = {0, 0, 0, 0};

        get_q_t(q, k1);                           // = q_t(q(step, :));
        get_q_t(q + (0.5 * _stepSize) * k1, k2);  // = q_t(q(step, :) + stepsize*k1/2);
        get_q_t(q + (0.5 * _stepSize) * k2, k3);  // = q_t(q(step, :) + stepsize*k2/2);
        get_q_t(q + _stepSize * k3, k4);          // = q_t(q(step, :) + stepsize*k3);

        q_t = (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        q = q + _stepSize * q_t;
        m_time = m_time + _stepSize;

        sendToLog("Step at time: " + std::to_string(m_time) + " succeeded.\n", fmi2Status::fmi2OK, "logAll");
    }

    return fmi2Status::fmi2OK;
}
