// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple powertrain model for the Sedan vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
//
// =============================================================================

#ifndef UAZBUS_ENGINE_SIMPLEMAP_H
#define UAZBUS_ENGINE_SIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace uaz {

/// @addtogroup vehicle_models_uaz
/// @{

/// Simple UAZBUS powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API UAZBUS_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    UAZBUS_EngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunctionInterp::AddPoint() function.
    virtual void SetEngineTorqueMaps(ChFunctionInterp& map0,  ///< [out] engine map at zero throttle
                                     ChFunctionInterp& mapF   ///< [out] engine map at full throttle
                                     ) override;
};

/// @} vehicle_models_uaz

}  // namespace uaz
}  // end namespace vehicle
}  // end namespace chrono

#endif
