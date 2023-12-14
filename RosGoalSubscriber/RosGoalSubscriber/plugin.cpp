#include "plugin.h"
#include "RosGoalSubscriber.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <RosGoalSubscriber>("Utilities")
    );
    plugin->add(summation_decl);
}
