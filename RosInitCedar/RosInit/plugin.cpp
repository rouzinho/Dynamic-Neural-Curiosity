#include "plugin.h"
#include "InitRos.h"
#include <cedar/processing/ElementDeclaration.h>
#include <cedar/processing/CppScriptDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::CppScriptDeclarationPtr ownscript
    (
        new cedar::proc::CppScriptDeclarationTemplate  <InitRos>("category")
    );
    plugin->add(ownscript);
}
