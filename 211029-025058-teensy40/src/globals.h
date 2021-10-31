uint64_t acc_active;
uint64_t acc_allowed;
uint64_t crz_ended;
uint16_t accel_cmd;
uint64_t steering_angle;
uint64_t inverse_speed;
uint64_t distance_lead;
uint64_t relative_vel;
uint64_t counter;

int cnt = 0;

uint64_t acc_fixed;

uint64_t accel_request;
uint64_t radar_acc_command;
//timer
uint64_t can_tick;

bool rate_tmr;

bool begin;
bool b_acc_active;