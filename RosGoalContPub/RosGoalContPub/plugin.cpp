#include "plugin.h"
#include "RosGoalContPub.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <RosGoalContPub>("Utilities")
    );
    plugin->add(summation_decl);
}
