// =============================================================================
// Example FMU for co-simulation (FMI 2.0 standard)
// =============================================================================
//
// Illustrates the FMU exporting capabilities in fmu_tools (FmuToolsExport.h)
//
// =============================================================================

#pragma once

#include <vector>
#include <array>
#include <fstream>

// #define FMI2_FUNCTION_PREFIX MyModel_
#include "fmi2/FmuToolsExport.h"

class myFmuComponent : public FmuComponentBase {
  public:
    myFmuComponent(fmi2String instanceName,
                   fmi2Type fmuType,
                   fmi2String fmuGUID,
                   fmi2String fmuResourceLocation,
                   const fmi2CallbackFunctions* functions,
                   fmi2Boolean visible,
                   fmi2Boolean loggingOn);

    ~myFmuComponent() {}

    virtual void _enterInitializationMode() override;

    virtual void _exitInitializationMode() override;

    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint,
                               fmi2Real communicationStepSize,
                               fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

    // Problem-specific functions
    double get_x_tt(double th_t, double th);
    double get_th_tt(double th_t, double th);
    double get_x_tt_linear(double th_t, double th);
    double get_th_tt_linear(double th_t, double th);
    void get_q_t(const std::array<double, 4>& _q, std::array<double, 4>& q_t);

  private:
    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    // Problem-specific data members
    std::array<double, 4> q;
    std::array<double, 4> q_t;
    double len = 0.5;
    double m = 1.0;
    double M = 1.0;
    const double g = 9.81;
    fmi2Boolean approximateOn = fmi2False;
    std::string filename;
};
