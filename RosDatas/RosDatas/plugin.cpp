#include "plugin.h"
#include "RosDatas.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <RosDatas>("Utilities")
    );
    plugin->add(summation_decl);
}
