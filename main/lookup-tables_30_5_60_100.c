#include "lookup-tables_30_5_60_100.h"

#include <stdint.h>

// trailer and car constants of the table
static const float car_wheelbase = 0.35;
static const float car2hitch = 0.1;
static const float hitch2axle = 0.6;

//linear trailer control
static const float linear_factor;

// control constants of the table
static const float lookup_alpha_max = 0.000000;
static const int lookup_index0_max = 30;
static const int lookup_index1_max = 60;
static const float beta_max = 0.349066;
static const float alpha_max = 0.610865;
static const float linear_alpha_beta_faktor = 2.045534;
static const float distance = 1.0;

// Lookup Alpha by Beta
static const float lookup_ab_0[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.000000,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_1[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.005688,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_2[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.011377,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_3[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.017065,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_4[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.022753,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_5[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.028441,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_6[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.034130,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_7[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.039818,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_8[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.045506,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_9[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.051194,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_10[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.056883,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_11[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.062571,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_12[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.068259,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_13[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.073947,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_14[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.079636,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_15[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.085324,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_16[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.091012,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_17[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.096700,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_18[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.102389,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_19[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.108077,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_20[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.113765,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_21[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.119453,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_22[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.125142,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_23[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.130830,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_24[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.136518,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_25[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.142206,-3.141593,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_26[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.147895,-3.141593,-3.141593,-3.141593,};
static const float lookup_ab_27[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.153583,-3.141593,-3.141593,};
static const float lookup_ab_28[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.159271,-3.141593,};
static const float lookup_ab_29[] = {3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,-0.164959,};
static const float* lookup_alpha_by_beta[] = {(const float*)&lookup_ab_0, (const float*)&lookup_ab_1, (const float*)&lookup_ab_2, (const float*)&lookup_ab_3, (const float*)&lookup_ab_4, (const float*)&lookup_ab_5, (const float*)&lookup_ab_6, (const float*)&lookup_ab_7, (const float*)&lookup_ab_8, (const float*)&lookup_ab_9, (const float*)&lookup_ab_10, (const float*)&lookup_ab_11, (const float*)&lookup_ab_12, (const float*)&lookup_ab_13, (const float*)&lookup_ab_14, (const float*)&lookup_ab_15, (const float*)&lookup_ab_16, (const float*)&lookup_ab_17, (const float*)&lookup_ab_18, (const float*)&lookup_ab_19, (const float*)&lookup_ab_20, (const float*)&lookup_ab_21, (const float*)&lookup_ab_22, (const float*)&lookup_ab_23, (const float*)&lookup_ab_24, (const float*)&lookup_ab_25, (const float*)&lookup_ab_26, (const float*)&lookup_ab_27, (const float*)&lookup_ab_28, (const float*)&lookup_ab_29, };

// Lookup Beta by Alpha
static const float lookup_ba_0[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_1[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_2[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_3[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_4[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_5[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_6[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_7[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_8[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_9[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_10[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_11[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_12[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_13[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_14[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_15[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_16[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_17[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_18[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_19[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_20[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_21[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_22[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_23[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_24[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_25[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_26[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_27[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_28[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float lookup_ba_29[] = {-0.152974,-0.147794,-0.142623,-0.137460,-0.132304,-0.127156,-0.122014,-0.116880,-0.111752,-0.106630,-0.101514,-0.096404,-0.091299,-0.086199,-0.081104,-0.076013,-0.070927,-0.065845,-0.060766,-0.055690,-0.050618,-0.045548,-0.040481,-0.035416,-0.030353,-0.025291,-0.020231,-0.015173,-0.010115,-0.005057,0.000000,0.005057,0.010115,0.015173,0.020231,0.025291,0.030353,0.035416,0.040481,0.045548,0.050618,0.055690,0.060766,0.065845,0.070927,0.076013,0.081104,0.086199,0.091299,0.096404,0.101514,0.106630,0.111752,0.116880,0.122014,0.127156,0.132304,0.137460,0.142623,0.147794,};
static const float* lookup_beta_by_alpha[] = {(const float*)&lookup_ba_0, (const float*)&lookup_ba_1, (const float*)&lookup_ba_2, (const float*)&lookup_ba_3, (const float*)&lookup_ba_4, (const float*)&lookup_ba_5, (const float*)&lookup_ba_6, (const float*)&lookup_ba_7, (const float*)&lookup_ba_8, (const float*)&lookup_ba_9, (const float*)&lookup_ba_10, (const float*)&lookup_ba_11, (const float*)&lookup_ba_12, (const float*)&lookup_ba_13, (const float*)&lookup_ba_14, (const float*)&lookup_ba_15, (const float*)&lookup_ba_16, (const float*)&lookup_ba_17, (const float*)&lookup_ba_18, (const float*)&lookup_ba_19, (const float*)&lookup_ba_20, (const float*)&lookup_ba_21, (const float*)&lookup_ba_22, (const float*)&lookup_ba_23, (const float*)&lookup_ba_24, (const float*)&lookup_ba_25, (const float*)&lookup_ba_26, (const float*)&lookup_ba_27, (const float*)&lookup_ba_28, (const float*)&lookup_ba_29, };

//export function
float export_linear_control_30_5_60_100(){
        return linear_factor;
}

void export_car_30_5_60_100(car_params* inval){
        inval->alpha_max = alpha_max;
        inval->car2hitch = 0;
        inval->car_wheelbase = 0;
}

void export_lookup_30_5_60_100(sim_params* inval) {
        export_car_30_5_60_100(&inval->connected_car);
}

void export_lookup_30_5_60_100(lookup_table* inval) {
        inval->constant_table = true;
        export_car_30_5_60_100(&inval->connected_car);
        inval->alpha_max = alpha_max;
        inval->beta_max = beta_max;
        inval->lookup_alpha_by_beta = lookup_alpha_by_beta;
        inval->lookup_beta_by_alpha = lookup_beta_by_alpha;
        inval->linear_alpha_beta_faktor = linear_alpha_beta_faktor;
        inval->lookup_index0_max = lookup_index0_max;
        inval->lookup_index1_max = lookup_index1_max;
}