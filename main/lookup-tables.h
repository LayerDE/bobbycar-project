#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	bool constant_table;
	int lookup_index0_max;
	int lookup_index1_max;
	float beta_max;
	float alpha_max;
	float linear_alpha_beta_faktor;
	float** lookup_alpha_by_beta;
	float** lookup_beta_by_alpha;
} lookup_table;

typedef void (*lookup_loader)(lookup_table* inval);

#ifdef __cplusplus
extern "C" {
#endif
	void export_lookup(lookup_table* inval);
	extern const float UNREACHABLE;
#ifdef __cplusplus
}
#endif