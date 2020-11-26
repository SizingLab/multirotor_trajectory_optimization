"""
This module is for registering all internal OpenMDAO modules that we want
available through OpenMDAOSystemRegistry
"""
#  This file is part of FAST-OAD : A framework for rapid Overall Aircraft Design
#  Copyright (C) 2020  ONERA & ISAE-SUPAERO
#  FAST is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
from fastoad.module_management import OpenMDAOSystemRegistry
from .multirotor import Multirotor, MTOW, SystemConstraints
from .sizing_scenarios import HoverAndTO
from .propeller import Propeller
from .motor import Motor
from .battery_and_esc import BatteryAndESC
from .frame import Frame
from .mission import Mission
from .trajectory import Model


def register_openmdao_systems():
    """
    The place where to register FAST-OAD internal models.

    Warning: this function is effective only if called from a Python module that
    is a started bundle for iPOPO
    """
    # Complete multirotor ################################################################
    # OpenMDAOSystemRegistry.register_system(
    #     Multirotor, "multirotor.system"
    # )
    # Sizing scenarios ################################################################
    OpenMDAOSystemRegistry.register_system(
        HoverAndTO, "multirotor.sizing_scenarios"
    )
    # Propeller ################################################################
    OpenMDAOSystemRegistry.register_system(
        Propeller, "multirotor.propeller"
    )
    # Motor ################################################################
    OpenMDAOSystemRegistry.register_system(
        Motor, "multirotor.motor"
    )
    # Battery and ESC ################################################################
    OpenMDAOSystemRegistry.register_system(
        BatteryAndESC, "multirotor.battery_and_esc"
    )
    # Frame ################################################################
    OpenMDAOSystemRegistry.register_system(
        Frame, "multirotor.frame"
    )
    # Mission ################################################################
    OpenMDAOSystemRegistry.register_system(
        Mission, "multirotor.mission"
    )
    # # MTOW ################################################################
    # OpenMDAOSystemRegistry.register_system(
    #     MTOW, "multirotor.mtow"
    # )
    # System constraints ################################################################
    OpenMDAOSystemRegistry.register_system(
        SystemConstraints, "multirotor.system_constraints"
    )
    # System constraints ################################################################
    OpenMDAOSystemRegistry.register_system(
        Model, "multirotor.trajectory"
    )
