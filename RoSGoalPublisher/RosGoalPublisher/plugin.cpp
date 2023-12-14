#include "plugin.h"
#include "RosGoalPublisher.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <RosGoalPublisher>("Utilities")
    );
    plugin->add(summation_decl);
}
