#pragma once

#include <stdbool.h>

#include "lookup_table_types.h"

#ifdef __cplusplus
extern "C" {
#endif
	void export_lookup(lookup_table* inval);
	extern const unsigned int UNREACHABLE;
#ifdef __cplusplus
}
#endif