#include <math.h>
#include "CoreSLAM.h"

ts_position_t ts_close_loop_position(ts_state_t *state, ts_sensor_data_t *sensor_data, 
									 ts_map_t *loop_close_map, ts_position_t *start_position, 
									 int *q)
{
	ts_scan_t scan;
	int quality;
	ts_position_t lc_position;

	ts_build_scan(sensor_data, &scan, state, 1);
	lc_position = ts_monte_carlo_search(&state->randomizer, &scan, loop_close_map, start_position, 600, 20, 100000, &quality, 0);
	if (q) *q = quality;
	return lc_position;
}

void ts_close_loop_trajectory(ts_sensor_data_t *sensor_data, int maxscans,
							  ts_position_t *startpos, ts_position_t *close_loop_position)
{
	int i, j;
	double weight, theta[2];
	ts_position_t *final_pos;
	for (i = 0; i != maxscans; i++) {
		weight = i / ((double)(maxscans - 1));
		final_pos = &sensor_data[i].position[TS_FINAL_MAP];
		final_pos->x = (1 - weight) * sensor_data[i].position[TS_DIRECTION_FORWARD].x + weight * sensor_data[i].position[TS_DIRECTION_BACKWARD].x;
		final_pos->y = (1 - weight) * sensor_data[i].position[TS_DIRECTION_FORWARD].y + weight * sensor_data[i].position[TS_DIRECTION_BACKWARD].y;
		// Make sure both angles are between 0 and 360
		for (j = 0; j != 2; j++) {
			theta[j] = sensor_data[i].position[j].theta;
			while (theta[j] < 0) theta[j] += 360;
			while (theta[j] >= 360) theta[j] -= 360;
		}
		if (fabs(theta[0] - theta[1]) >= 180)
			if (theta[0] > theta[1])
				theta[0] -= 360;
			else
				theta[1] -= 360;
		final_pos->theta = (1 - weight) * theta[0] + weight * theta[1];
	}
}
