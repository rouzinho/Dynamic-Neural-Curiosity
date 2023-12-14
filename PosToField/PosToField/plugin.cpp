#include "plugin.h"
#include "PosToField.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <PosToField>("Utilities")
    );
    plugin->add(summation_decl);
}
