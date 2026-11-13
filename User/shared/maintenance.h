#ifndef MAINTENANCE_H
#define MAINTENANCE_H
#include "app_types.h"
typedef enum { MAINT_OK, MAINT_MODE, MAINT_OUTPUT, MAINT_MOTION, MAINT_PARAM } MaintenanceCheck;
MaintenanceCheck maintenance_stop_check(const ControlSnapshot *s);
MaintenanceCheck maintenance_save_check(const ControlSnapshot *s,const AppParam *p);
#endif
