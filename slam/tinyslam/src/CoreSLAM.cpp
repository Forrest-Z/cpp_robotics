#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "CoreSLAM.h"

#define SWAP(x, y) (x ^= y ^= x ^= y)

//初始化地图数据数组
void ts_map_init(ts_map_t *map)
{
	//创建指向地图数据数组指针
	ts_map_pixel_t *ptr;
	ptr = map->map;
	//初始化地图数据数组的值
	int initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;
	//遍历整个地图数据数组
	for (int y = 0; y < TS_MAP_SIZE; y++)
	{
		for (int x = 0; x < TS_MAP_SIZE; x++, ptr++) {
			*ptr = initval;
		}
	}
}

// 这个函数的输入是这一帧的点集，以及已有的地图，
// 还有当前随机取的一个位置（测试这个位置是否是
// 最优位置）。输出是这个位置的似然度，然后其它
// 函数根据这个似然度来判断最优位置。在这个函数
// 内部，是先将一帧的点集转为相对于当前position
// 的点集，然后再进行似然度计算。这个似然度的计算
// 仅仅是通过将hit点集的map value值加起来求平均值。
// 似然度（即激光点集与地图的匹配值）（越小匹配度越高）
int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos)
{
	int i, nb_points = 0;
	int64_t sum;

	double c = cos(pos->theta * M_PI / 180);
	double s = sin(pos->theta * M_PI / 180);

	// Translate and rotate scan to robot position
	// and compute the distance
	for (i = 0, sum = 0; i != scan->nb_points; i++)
	{
		// 取检测到障碍物的激光数据
		if (scan->value[i] != TS_NO_OBSTACLE)
		{
			// 根据激光数据计算激光点在地图坐标系中的位姿
			int x = (int)floor((pos->x + c * scan->x[i] - s * scan->y[i]) / TS_MAP_SCALE + 0.5);
			int y = (int)floor((pos->y + s * scan->x[i] + c * scan->y[i]) / TS_MAP_SCALE + 0.5);
			// Check boundaries
			if (x >= 0 && x < TS_MAP_SIZE && y >= 0 && y < TS_MAP_SIZE)
			{
				sum += map->map[y * TS_MAP_SIZE + x];//累计所有激光点处的地图数组值
				nb_points++;
			}
		}
	}
    // 似然度计算:通过将hit点集的map value值加起来求平均值。
	if (nb_points) sum = sum * 1024 / nb_points;//返回所有激光点处的地图数组值的均值
	else sum = 2000000000;

	return (int)sum;
}

// 使用的是Bresenham算法将laser rays绘制到地图中，
// 同时内部使用一个加强版的bresenham算法计算正确的侧面（profile？？）。
void ts_map_laser_ray(ts_map_t *map,
                      int x1, int y1,   //机器人相对地图在地图坐标系位姿
                      int x2, int y2,   //激光点相对地图在地图坐标系似然位姿
					  int xp, int yp,   //激光点相对地图在地图坐标系位姿
                      int value,
                      int alpha)
{
    // Robot is out of map
	if (x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE)
    {return;}

    int x2c = x2;
    int y2c = y2;

	// Clipping,剪裁
	if (x2c < 0) {
		if (x2c == x1) return;
		y2c += (y2c - y1) * (-x2c) / (x2c - x1);
		x2c = 0;
	}
	if (x2c >= TS_MAP_SIZE) {
		if (x1 == x2c) return;
		y2c += (y2c - y1) * (TS_MAP_SIZE - 1 - x2c) / (x2c - x1);
		x2c = TS_MAP_SIZE - 1;
	}
	if (y2c < 0) {
		if (y1 == y2c) return;
		x2c += (x1 - x2c) * (-y2c) / (y1 - y2c);
		y2c = 0;
	}
	if (y2c >= TS_MAP_SIZE) {
		if (y1 == y2c) return;
		x2c += (x1 - x2c) * (TS_MAP_SIZE - 1 - y2c) / (y1 - y2c);
		y2c = TS_MAP_SIZE - 1;
	}

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dxc = abs(x2c - x1);
    int dyc = abs(y2c - y1);
    int incptrx = (x2 > x1) ? 1 : -1;
    int incptry = (y2 > y1) ? TS_MAP_SIZE : -TS_MAP_SIZE;
    int sincv = (value > TS_NO_OBSTACLE) ? 1 : -1;

    int derrorv;
	if (dx > dy)
    {
		derrorv = abs(xp - x2);
	}
	else
    {
		SWAP(dx, dy); SWAP(dxc, dyc); SWAP(incptrx, incptry);
		derrorv = abs(yp - y2);
	}
    int error = 2 * dyc - dxc;
    int horiz = 2 * dyc;
    int diago = 2 * (dyc - dxc);
    int errorv = derrorv / 2;
    int incv = (value - TS_NO_OBSTACLE) / derrorv;
    int incerrorv = value - TS_NO_OBSTACLE - derrorv * incv;

    ts_map_pixel_t *ptr;
	ptr = map->map + y1 * TS_MAP_SIZE + x1;
    int pixval = TS_NO_OBSTACLE;
	for (int x = 0; x <= dxc; x++, ptr += incptrx)
    {
		if (x > dx - 2 * derrorv)
        {
			if (x <= dx - derrorv)
            {
				pixval += incv;
				errorv += incerrorv;
				if (errorv > derrorv)
                {
					pixval += sincv;
					errorv -= derrorv;
				}
			}
			else
            {
				pixval -= incv;
				errorv -= incerrorv;
				if (errorv < 0)
                {
					pixval -= sincv;
					errorv += derrorv;
				}
			}
		}
		//融入地图
		*ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8;
		if (error > 0)
        {
			ptr += incptry;
			error += diago;
		}
		else {error += horiz;}
	}
}

///更新地图
void ts_map_update(ts_scan_t *scan, 	//输入:激光数据
				   ts_map_t *map, 		//输出:地图数据
				   ts_position_t *pos, 	//输入:机器人位姿
				   int quality, 		//输入:障碍物对应的阈值
				   int hole_width)		//输入:似然函数峰值
{
    double c = cos(pos->theta * M_PI / 180);
    double s = sin(pos->theta * M_PI / 180);
    //世界位置->地图位置
    int x1 = (int)floor(pos->x / TS_MAP_SCALE + 0.5);
    int y1 = (int)floor(pos->y / TS_MAP_SCALE + 0.5);

	// Translate and rotate scan to robot position
	for (int i = 0; i != scan->nb_points; i++)
	{
        //激光点相对激光雷达的位姿
        double x2p = c * scan->x[i] - s * scan->y[i];
        double y2p = s * scan->x[i] + c * scan->y[i];
        //激光点相对地图位姿
        int xp = (int)floor((pos->x + x2p) / TS_MAP_SCALE + 0.5);
        int yp = (int)floor((pos->y + y2p) / TS_MAP_SCALE + 0.5);

        //激光点相对激光雷达的距离
        double dist = sqrt(x2p * x2p + y2p * y2p);
        //似然值
        double add = hole_width / 2 / dist;
        //加上似然值后的激光点相对激光雷达的地图位姿
		x2p = x2p / TS_MAP_SCALE * (1 + add);
		y2p = y2p / TS_MAP_SCALE * (1 + add);
        //加上似然值后的激光点相对地图位姿
        int x2 = (int)floor(pos->x / TS_MAP_SCALE + x2p + 0.5);
        int y2 = (int)floor(pos->y / TS_MAP_SCALE + y2p + 0.5);

        int value, q;
		if (scan->value[i] == TS_NO_OBSTACLE)
		{
			q = quality / 4;
			value = TS_NO_OBSTACLE;
		}
		else
		{
			q = quality;
			value = TS_OBSTACLE;
		}
		//printf("%d %d %d %d %d %d %d\n", i, x1, y1, x2, y2, xp, yp);
		ts_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);
	}
}

