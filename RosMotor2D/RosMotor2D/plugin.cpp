#include "plugin.h"
#include "MotorTwoDim.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <MotorTwoDim>("Utilities")
    );
    plugin->add(summation_decl);
}
