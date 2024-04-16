// =============================================================================
// Example FMU instantiation for co-simulation (FMI 2.0 standard)
// =============================================================================
//
// Illustrates the FMU importing capabilities in fmu_tools (FmuToolsImport.h)
//
// =============================================================================

#include <iostream>
#include <cstddef>

#include "fmi2/FmuToolsImport.h"

std::string unzipped_fmu_folder = FMU_UNPACK_DIRECTORY;
////std::string unzipped_fmu_folder = FMU_MAIN_DIRECTORY;

int main(int argc, char* argv[]) {
    FmuUnit my_fmu;

    try {
        ////my_fmu.LoadUnzipped(unzipped_fmu_folder);
        my_fmu.Load(FMU_FILENAME, FMU_UNPACK_DIRECTORY);  // FMU unpacked in provided directory
        ////my_fmu.Load(FMU_FILENAME);                        // FMU unpacked in a directory in /tmp

    } catch (std::exception& my_exception) {
        std::cout << "ERROR loading FMU: " << my_exception.what() << "\n";
    }

    std::cout << "FMU Version:  " << my_fmu._fmi2GetVersion() << "\n";
    std::cout << "FMU Platform: " << my_fmu._fmi2GetTypesPlatform() << "\n";

    ////my_fmu.Instantiate("FmuComponent", my_fmu.GetUnzippedFolder() + "resources");
    my_fmu.Instantiate("FmuComponent");  // automatic loading of default resources

    std::vector<std::string> categoriesVector = {"logAll"};
    my_fmu.SetDebugLogging(fmi2True, categoriesVector);

    // alternatively, with native interface:
    // std::vector<const char*> categoriesArray;
    // for (const auto& category : categoriesVector) {
    //    categoriesArray.push_back(category.c_str());
    //}

    // my_fmu._fmi2SetDebugLogging(my_fmu.component, fmi2True, categoriesVector.size(), categoriesArray.data());

    double start_time = 0;
    double stop_time = 2;
    my_fmu._fmi2SetupExperiment(my_fmu.component,
                                fmi2False,  // tolerance defined
                                0.0,        // tolerance
                                start_time,
                                fmi2False,  // use stop time
                                stop_time);

    my_fmu._fmi2EnterInitializationMode(my_fmu.component);

    {
        fmi2ValueReference valref = 8;
        fmi2Real m_in = 15;
        my_fmu._fmi2SetReal(my_fmu.component, &valref, 1, &m_in);

        fmi2Real m_out;
        my_fmu._fmi2GetReal(my_fmu.component, &valref, 1, &m_out);
        std::cout << "m_out_: " << m_out << std::endl;
    }

    my_fmu._fmi2ExitInitializationMode(my_fmu.component);

    // Test a simulation loop
    double time = 0;
    double dt = 0.001;

    for (int i = 0; i < 1000; ++i) {
        my_fmu._fmi2DoStep(my_fmu.component, time, dt, fmi2True);

        time += dt;
    }

    // Getting FMU variables through FMI functions, i.e. through valueRef
    fmi2Real val_real;
    for (fmi2ValueReference valref = 1; valref < 12; valref++) {
        my_fmu._fmi2GetReal(my_fmu.component, &valref, 1, &val_real);
        std::cout << "REAL: valref: " << valref << " | val: " << val_real << std::endl;
    }

    // Getting FMU variables through custom fmu-tools functions, directly using the variable name
    auto status = my_fmu.GetVariable("m", val_real, FmuVariable::Type::Real);

    fmi2Boolean val_bool;
    for (fmi2ValueReference valref = 1; valref < 2; valref++) {
        my_fmu._fmi2GetBoolean(my_fmu.component, &valref, 1, &val_bool);
        std::cout << "BOOLEAN: valref: " << valref << " | val: " << val_bool << std::endl;
    }

    return 0;
}
