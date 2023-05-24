#include <math.h>
#include <stdint.h>
#include <c_data.h>
#include "defines.h"
#include "config.h"

uint32_t index_buff_vals[VAL_CNT];
uint32_t buff_vals[VAL_CNT][BUFFERSIZE];
uint32_t cur_buff_val_sum[VAL_CNT];

void init_buffer(){
    for (int i = 0; i < VAL_CNT; i++)
    {
        cur_buff_val_sum[i] = index_buff_vals[i] = 0;
        for (int j = 0; j < BUFFERSIZE; j++)
            cur_buff_val_sum[i] += (buff_vals[i][j] = ADC_MID);
    }
}

uint32_t value_buffer(uint32_t in, int val)
{
  cur_buff_val_sum[val] -= buff_vals[val][index_buff_vals[val]];
  cur_buff_val_sum[val] += (buff_vals[val][index_buff_vals[val]] = in);
  index_buff_vals[val] = (index_buff_vals[val] + 1) % (BUFFERSIZE);
  return (cur_buff_val_sum[val] / (BUFFERSIZE));
}


// bobbycar
int sign(float in){
  if(in > 0.0f)
    return 1;
   else if(in < 0.0f)
    return -1;
   else
    return 0;
}

int clean_adc_full(uint32_t inval)
{
  int outval = (int)(inval) - ADC_MID;
  int abs_outval = abs(outval);
  if (abs_outval < (DEAD_ZONE / 2)) // deadzone
    return 0;
  else
    abs_outval -= (DEAD_ZONE / 2);
  if (abs_outval > (ADC_MAX - ADC_MID - DEAD_ZONE * 3 / 2))
    return THROTTLE_MAX * sign(outval);
  return abs_outval * THROTTLE_MAX / (ADC_MAX - ADC_MID - DEAD_ZONE * 3 / 2) * sign(outval);
}

int clean_adc_steering(uint32_t inval)
{
  int outval = (int)(inval) - STR_MID;
  int abs_outval = abs(outval);
  if (abs_outval < (DEAD_ZONE / 2)) // deadzone
    return 0;
  else
    abs_outval -= (DEAD_ZONE / 2);
  if (abs_outval > (STR_RANGE - DEAD_ZONE * 3 / 2))
    return STR_MAX * sign(outval);
  return abs_outval * STR_MAX / (STR_RANGE - DEAD_ZONE * 3 / 2) * sign(outval);
}

int clean_adc_follower(uint32_t inval){
  return (int)(inval) - FLW_MID;
}

unsigned int clean_adc_half(uint32_t inval)
{
  int outval = (uint32_t)inval;
  if (abs(outval) > (ADC_MAX - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX;
  else if(abs(outval) < (((DEAD_ZONE * 3) / 2)))
    return 0;
  return outval * (THROTTLE_MAX / 2) / (ADC_MAX - DEAD_ZONE * 3);
}

int throttle_calc(int cleaned_adc)
{
  return cleaned_adc < 0 ?
    ((cleaned_adc * cleaned_adc / THROTTLE_MAX) * (-2) + cleaned_adc) / 3 *THROTTLE_REVERSE_MAX / THROTTLE_MAX
    : ((cleaned_adc * cleaned_adc / THROTTLE_MAX) * 2 + cleaned_adc) / 3;
}

int calc_torque(int throttle, int breaks)
{
  if (breaks == 0)
  { // drive forward
    return throttle;
  }
  else if (breaks == THROTTLE_MAX)
  { // drive backwards
    return -throttle;
  }
  else
  {
    return throttle - breaks;
  }
}

float calc_angle(int inval)
{
  return (float)inval * STEERING_ANGLE_FACTOR * 13.0f / 19.0f;
}

static inline float pow2(float x){
  return x*x;
}

float calc_trailer_clean_10k100k(float y){
  return (sqrt(41*pow2(y)-62*y+121)+y+9)/2*(y-1);
}

void calc_torque_per_wheel(int throttle, float alpha_steer,int torque_regulated, int *torque)
{
  if(alpha_steer == 0.0f){
    torque[0] = torque[1] = torque[2] = torque[3] = throttle;
  }
  else{
    float V[4];
    float V_bw = L_WHEELBASE/tan(fabs(alpha_steer));
    V[2] = (V_bw + L_WIDTH/2.0*sign(alpha_steer)) / V_bw; //torque[2] 
    V[3]= (V_bw - L_WIDTH/2.0*sign(alpha_steer)) / V_bw; //torque[3] 

    V[0] = (sqrt(pow2(V_bw + (L_STEERING_WIDTH/2.0)*sign(alpha_steer))+pow2(L_WHEELBASE))+L_STEERING_TO_WHEEL)/V_bw; //torque[0] 
    V[1] = (sqrt(pow2(V_bw - (L_STEERING_WIDTH/2.0)*sign(alpha_steer))+pow2(L_WHEELBASE))+L_STEERING_TO_WHEEL)/V_bw; //torque[1]

    float correction_factor = 4.0/(V[0]+V[1]+V[2]+V[3]);
    for(int x = 0; x < 4; x++)
      torque[x] = round((float)throttle*V[x]*correction_factor);
  }
  torque[0] += torque_regulated;
  torque[1] -= torque_regulated;
  for(int x = 0; x < 4; x++)
    if(torque[x]>THROTTLE_MAX)
      torque[x] = THROTTLE_MAX;
    else if(torque[x]<(-THROTTLE_MAX))
      torque[x] = (-THROTTLE_MAX);

}

float rad2deg(float in){
  return in * 45.0 / M_PI_4;
}
float deg2rad(float in){
  return in * M_PI_4 / 45.0;
}

static inline void swp(int *x, int *y)
{
  int tmp = *x;
  *x = *y;
  *y = tmp;
}
static inline void sort_array(int *x, int cnt)
{
  for (int y = 0; y < cnt - 1; y++)
    for (int z = y + 1; z < cnt; z++)
      if (x[y] > x[z])
        swp(&x[y], &x[z]);
}

int calc_average(const int *x, int cnt){
  int sum = 0;
  for(int y = 0; y < cnt; y++)
    sum+=x[y];
  return sum/cnt;
}

int calc_median(const int x[], int cnt)
{
  int median;
  c_data* data = NULL;
  c_data_spawn_ptr(data);
  c_data_set(data, (void*)x, cnt*sizeof(int));
  sort_array((int*)data->content, data->size);
  if (cnt % 2)
    median = x[cnt / 2];
  else
    median = (x[cnt / 2] + x[cnt / 2 + 1]) / 2;
  c_data_delete_ptr(data);
  return median;
}