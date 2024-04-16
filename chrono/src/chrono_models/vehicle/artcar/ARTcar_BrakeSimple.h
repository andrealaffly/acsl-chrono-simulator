// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Jayne Henry
// =============================================================================
//
// ARTcar simple brake models (front and rear).
//
// =============================================================================

#ifndef ARTCAR_BRAKESIMPLE_H
#define ARTCAR_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace artcar {

/// @addtogroup vehicle_models_artcar
/// @{

/// Simple ARTcar brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API ARTcar_BrakeSimple : public chrono::vehicle::ChBrakeSimple {
  public:
    ARTcar_BrakeSimple(const std::string& name);
    virtual ~ARTcar_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

/// @} vehicle_models_artcar

}  // namespace artcar
}  // namespace vehicle
}  // namespace chrono

#endif
