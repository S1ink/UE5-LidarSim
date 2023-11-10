#pragma once

#include "Modules/ModuleManager.h"


/** Please note that this file is mostly worthless as this modules is not accessed externally */
class FLidarSimModule : public IModuleInterface {
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

};
