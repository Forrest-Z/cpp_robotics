#ifndef _TINYSLAM_H_
#define _TINYSLAM_H_
///----------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "opencv2/opencv.hpp"
///----------------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
///----------------------------------------------------------------------
#define TS_SCAN_SIZE                8192	//激光数据数组的最大值
#define TS_MAP_SIZE                 2048	//地图数据数组的行和列的大小
#define TS_MAP_SCALE                10.0	//地图数据分辨率,cm
#define TS_DISTANCE_NO_DETECTION    4000
#define TS_NO_OBSTACLE              65500	//地图数据数组中代表无障碍物的值
#define TS_OBSTACLE                 0		//地图数据数组中代表障碍物的值

#define TS_DIRECTION_FORWARD        0
#define TS_DIRECTION_BACKWARD       1
#define TS_FINAL_MAP                2

typedef unsigned short ts_map_pixel_t;
///----------------------------------------------------------------------
typedef struct 
{
	ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE];
} ts_map_t;//地图数据

typedef struct
{
	double x[TS_SCAN_SIZE], y[TS_SCAN_SIZE];	//激光数据距离激光传感器的坐标
	int value[TS_SCAN_SIZE];					//激光检测的距离数据
	int nb_points;								//激光数据点数个数
} ts_scan_t;//激光数据

typedef struct
{
	int timestamp;	//时间戳
	int q1, q2;		//左右轮编码器值
	ts_scan_t scan;	//激光数据
} ts_sensor_data_t2;//传感器数据

typedef struct
{
	double x, y;    // in cm
	double theta;   // in degrees
} ts_position_t;//位姿数据

typedef struct
{
	unsigned int timestamp;		//时间戳
	int q1, q2;                	//左右轮编码器值
	double v, psidot;          	// 根据机器人的位姿纠正激光点集位姿
	ts_position_t position[3]; 	// 0 : forward - 1 : backward - 2 : final / closed loop
                                // 相对位置的记录，向前移动，向后移动，闭环 (三种情况)
	int d[TS_SCAN_SIZE];        //激光检测的距离数据
} ts_sensor_data_t;//传感器数据

typedef struct
{
	unsigned long jz;
	unsigned long jsr;
	long hz;
	unsigned long iz;
	unsigned long kn[128];
	double wnt[128];
	double wn[128];
	double fn[128];
} ts_randomizer_t;// 随机部分？？

typedef struct
{
	double r;	    // 轮子的半径
	double R;	    // 左右轮间距一半距离
	int inc;	    // 轮子转一圈的编码值
	double ratio;   // 左右轮减速比
} ts_robot_parameters_t;//机器人参数

typedef struct
{
	double offset;  // position of the laser wrt center of rotation
	int scan_size;  // number of points per scan
	int angle_min;  // start angle for scan
	int angle_max;  // end angle for scan
	int detection_margin; // first scan element to consider
	double distance_no_detection; // default value when the laser returns 0
} ts_laser_parameters_t;//激光雷达参数

typedef struct
{
	ts_randomizer_t randomizer;
	ts_map_t *map;
	ts_robot_parameters_t params;
	ts_laser_parameters_t laser_params;
	ts_position_t position;
	int q1, q2;
	unsigned int timestamp;
	double psidot, v;
	double distance;
	int hole_width;
	int direction;
	int done, draw_hole_map;
	ts_scan_t scan;
	double sigma_xy;
	double sigma_theta;
} ts_state_t;//机器人状态数据
///----------------------------------------------------------------------
void ts_map_init(ts_map_t *map);

int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos);

void ts_map_laser_ray(ts_map_t *map,
					  int x1, int y1,   //机器人相对地图在地图坐标系位姿
					  int x2, int y2,   //激光点相对地图在地图坐标系似然位姿
					  int xp, int yp,   //激光点相对地图在地图坐标系位姿
					  int value,
					  int alpha);

void ts_map_update(ts_scan_t *scan, ts_map_t *map,
				   ts_position_t *position, int quality,
				   int hole_width);
///----------------------------------------------------------------------
void ts_random_init(ts_randomizer_t *d, unsigned long jsrseed);

long ts_random_int(ts_randomizer_t *d, long min, long max);

double ts_random_normal_fix(ts_randomizer_t *d);

double ts_random_normal(ts_randomizer_t *d, double m, double s);

double ts_random(ts_randomizer_t *d);

//根据激光雷达数据和上次坐标位置得到当前的最佳坐标位置
ts_position_t monte_carlo_move(ts_scan_t *scan, ts_map_t *map,
							   ts_position_t *start_pos, int debug);

ts_position_t ts_monte_carlo_search(ts_randomizer_t *randomizer,
									ts_scan_t *scan, ts_map_t *map,
									ts_position_t *start_pos,
									double sigma_xy,
									double sigma_theta,
									int stop, int *bestdist, int debug);
///----------------------------------------------------------------------
void ts_state_init(ts_state_t *state, ts_map_t *map,
				   ts_robot_parameters_t *params,
				   ts_laser_parameters_t *laser_params,
				   ts_position_t *position, double sigma_xy,
				   double sigma_theta, int hole_width, int direction);

void ts_build_scan(ts_sensor_data_t *sd, ts_scan_t *scan,
				   ts_state_t *state, int span);

void ts_iterative_map_building(ts_sensor_data_t *sd, ts_state_t *state);
///----------------------------------------------------------------------
// Loop closing
ts_position_t ts_close_loop_position(ts_state_t *state, ts_sensor_data_t
									*sensor_data, ts_map_t *loop_close_map,
									 ts_position_t *start_position, int *q);

void ts_close_loop_trajectory(ts_sensor_data_t *sensor_data, int maxscans,
							  ts_position_t *startpos,
							  ts_position_t *close_loop_position);
///----------------------------------------------------------------------
#endif // _TINYSLAM_H_
