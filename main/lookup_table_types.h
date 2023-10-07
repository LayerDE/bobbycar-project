#pragma once

typedef struct{
	float alpha_max;
	float car2hitch;
	float car_wheelbase;
} car_params;

typedef struct {
	bool constant_table;
	car_params connected_car;
	int lookup_index0_max;
	int lookup_index1_max;
	float beta_max;
	float alpha_max;
	float linear_alpha_beta_faktor;
	float** lookup_alpha_by_beta;
	float** lookup_beta_by_alpha;
} lookup_table;

typedef struct {
	car_params connected_car;
	float hitch2axle;
	float beta_max;
	float alpha_max;
	float distance;
} sim_params;

typedef void (*lookup_loader)(lookup_table* inval);

extern const float* unreachable;