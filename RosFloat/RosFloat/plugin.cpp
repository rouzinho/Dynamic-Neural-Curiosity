#include "plugin.h"
#include "RosFloat.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <RosFloat>("Utilities")
    );
    plugin->add(summation_decl);
}
