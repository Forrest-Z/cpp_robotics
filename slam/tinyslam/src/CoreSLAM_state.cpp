#include <stdlib.h>
#include <math.h>
#include "CoreSLAM.h"

void ts_state_init(ts_state_t *state, ts_map_t *map, 
				   ts_robot_parameters_t *params, ts_laser_parameters_t *laser_params, 
				   ts_position_t *position, 
				   double sigma_xy, double sigma_theta, int hole_width, int direction)
{
	ts_random_init(&state->randomizer, 0xdead);
	state->map = map;
	state->params = *params;
	state->laser_params = *laser_params;
	state->position = *position;
	state->timestamp = 0; // Indicating start
	state->distance = 0;
	state->q1 = state->q2 = 0;
	state->psidot = state->v = 0;
	state->direction = direction;
	state->done = 0;
	state->draw_hole_map = 0;
	state->sigma_xy = sigma_xy;
	state->sigma_theta = sigma_theta;
	state->hole_width = hole_width;
}

void ts_build_scan(ts_sensor_data_t *sd, ts_scan_t *scan, ts_state_t *state, int span)
{
	int i, j;
	double angle_rad, angle_deg;

	scan->nb_points = 0;
	// Span the laser scans to better cosd->ver the space
	for (i = 0; i < state->laser_params.scan_size; i++)
	{
		for (j = 0; j != span; j++)
		{
			angle_deg = state->laser_params.angle_min +
					((double)(i * span + j)) *
					(state->laser_params.angle_max - state->laser_params.angle_min) /
					(state->laser_params.scan_size * span - 1);
			angle_deg += sd->psidot / 3600.0 * ((double)(i * span + j)) *
					(state->laser_params.angle_max - state->laser_params.angle_min) /
					(state->laser_params.scan_size * span - 1);

			angle_rad = angle_deg * M_PI / 180;
			if (i > state->laser_params.detection_margin &&
				i < state->laser_params.scan_size - state->laser_params.detection_margin)
			{
				if (sd->d[i] == 0) {
					scan->x[scan->nb_points] = state->laser_params.distance_no_detection * cos(angle_rad);
					scan->x[scan->nb_points] -= sd->v * 1000 * ((double)(i * span + j)) *
							(state->laser_params.angle_max - state->laser_params.angle_min) /
							(state->laser_params.scan_size * span - 1) / 3600.0;
					scan->y[scan->nb_points] = state->laser_params.distance_no_detection * sin(angle_rad);
					scan->value[scan->nb_points] = TS_NO_OBSTACLE;
					scan->nb_points++;
				}
				if (sd->d[i] > state->hole_width / 2) {
					scan->x[scan->nb_points] = sd->d[i] * cos(angle_rad);
					scan->x[scan->nb_points] -= sd->v * 1000 * ((double)(i * span + j)) *
							(state->laser_params.angle_max - state->laser_params.angle_min) /
							(state->laser_params.scan_size * span - 1) / 3600.0;
					scan->y[scan->nb_points] = sd->d[i] * sin(angle_rad);
					scan->value[scan->nb_points] = TS_OBSTACLE;
					scan->nb_points++;
				}
			}
		}
	}
}

void ts_iterative_map_building(ts_sensor_data_t *sd, ts_state_t *state)
{
	double psidot, v, d;
	ts_scan_t scan2map;
	double m, thetarad;
	ts_position_t position;

	// Manage robot position
	if (state->timestamp != 0) {
		m = state->params.r * M_PI / state->params.inc;
		v = m * (sd->q1 - state->q1 + (sd->q2 - state->q2) * state->params.ratio);
		thetarad = state->position.theta * M_PI / 180;
		position = state->position;
		position.x += v * 1000 * cos(thetarad);
		position.y += v * 1000 * sin(thetarad);
		psidot = (m * ((sd->q2 - state->q2) * state->params.ratio - sd->q1 + state->q1) / state->params.R) * 180 / M_PI;
		position.theta += psidot;
		v *= 1000000.0 / (sd->timestamp - state->timestamp);
		psidot *= 1000000.0 / (sd->timestamp - state->timestamp);
	}
	else {
		state->psidot = psidot = 0;
		state->v = v = 0;
		position = state->position;
		thetarad = state->position.theta * M_PI / 180;
	}

	// Change to (x,y) scan
	if (state->direction == TS_DIRECTION_FORWARD) {
		// Prepare speed/yawrate correction of scans (with a delay of 1 clock)
		sd->psidot = state->psidot;
		sd->v = state->v;
	}
	ts_build_scan(sd, &scan2map, state, 3);
	ts_build_scan(sd, &state->scan, state, 1);

	// Monte Carlo search
	position.x += state->laser_params.offset * cos(thetarad);
	position.y += state->laser_params.offset * sin(thetarad);
	sd->position[state->direction] = position =
		ts_monte_carlo_search(&state->randomizer, &state->scan, state->map, &position, state->sigma_xy, state->sigma_theta, 1000, NULL, 0);
	sd->position[state->direction].x -= state->laser_params.offset * cos(position.theta * M_PI / 180);
	sd->position[state->direction].y -= state->laser_params.offset * sin(position.theta * M_PI / 180);
	d = sqrt((state->position.x - sd->position[state->direction].x) * (state->position.x - sd->position[state->direction].x) +
		(state->position.y - sd->position[state->direction].y) * (state->position.y - sd->position[state->direction].y));
	state->distance += d;

	// Map update
	ts_map_update(&scan2map, state->map, &position, 50, state->hole_width);

	// Prepare next step
	state->position = sd->position[state->direction];
	state->psidot = psidot;
	state->v = v;
	state->q1 = sd->q1;
	state->q2 = sd->q2;
	state->timestamp = sd->timestamp;
}

