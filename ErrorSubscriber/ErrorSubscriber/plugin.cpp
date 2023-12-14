#include "plugin.h"
#include "ErrorSubscriber.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <ErrorSubscriber>("Utilities")
    );
    plugin->add(summation_decl);
}
