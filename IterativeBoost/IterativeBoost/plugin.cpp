#include "plugin.h"
#include "IterativeBoost.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <IterativeBoost>("Utilities")
    );
    plugin->add(summation_decl);
}
