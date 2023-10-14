#pragma once

#include <stdbool.h>

#include "lookup_table_types.h"

#ifdef __cplusplus
extern "C" {
#endif
	float export_linear_control_30_10_60_100();
	void export_car_30_10_60_100(car_params* inval);
	void export_sim_params_30_10_60_100(sim_params* inval);
	void export_lookup_30_10_60_100(lookup_table* inval);
#ifdef __cplusplus
}
#endif