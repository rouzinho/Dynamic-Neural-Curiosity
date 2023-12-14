#include "plugin.h"
#include "RosFloatPublisher.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <RosFloatPublisher>("Utilities")
    );
    plugin->add(summation_decl);
}
