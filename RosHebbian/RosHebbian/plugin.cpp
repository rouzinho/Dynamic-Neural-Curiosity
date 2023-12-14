#include "plugin.h"
#include "RosHebbian.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <RosHebbian>("Utilities")
    );
    plugin->add(summation_decl);
}
