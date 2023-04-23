#pragma once
#include <string>
#include "riptide_rviz/ControllerMapping.hpp"

namespace riptide_rviz
{
    enum InputType { ANALOG, DIGITAL };
    class ControllerMapping
    {
        public:
            std::string name;
            riptide_rviz::InputType inputType;
            int primaryChannel = -1;
            int secondaryChannel = -1;
            ControllerMapping();
    };
}
