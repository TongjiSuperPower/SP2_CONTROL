/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#include "sp2_hw/hardware_interface/CanBus.hpp"
namespace SP2Control
{
    CanBus::CanBus(const std::string &name, ID2ACTDATA_MAP *id2act_data, TYPE2ACTCOEFF_MAP *type2act_coeff)
        : can_bus_data_()
    {
    }
} // namespace SP2Control