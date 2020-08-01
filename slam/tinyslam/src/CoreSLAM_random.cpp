#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "CoreSLAM.h"

static unsigned long SHR3(ts_randomizer_t *d)
{
	d->jz = d->jsr;
	d->jsr ^= (d->jsr << 13);
	d->jsr ^= (d->jsr >> 17);
	d->jsr ^= (d->jsr << 5);

	return d->jz + d->jsr;
}

static double UNI(ts_randomizer_t *d)
{
	return .5 + (signed)SHR3(d) * .2328306e-9;
}

double ts_random_normal_fix(ts_randomizer_t *d)
{
	const double r = 3.442620; 	// The starting of the right tail 	
	static double x, y;
	for (;;)
	{
		x = d->hz*d->wn[d->iz];
		if (d->iz == 0)
		{ // iz==0, handle the base strip
			do
			{
				x = -log(UNI(d))*0.2904764;
				// .2904764 is 1/r				
				y = -log(UNI(d));
			} while (y + y<x*x);
			return (d->hz>0) ? r + x : -r - x;
		}

		// iz>0, handle the wedges of other strips		
		if (d->fn[d->iz] + UNI(d)*(d->fn[d->iz - 1] - d->fn[d->iz]) < exp(-.5*x*x))
			return x;
		// Start all over		
		d->hz = SHR3(d);
		d->iz = d->hz & 127;
		if ((unsigned long)abs(d->hz)<d->kn[d->iz])
			return (d->hz*d->wn[d->iz]);
	}
}

double ts_random_normal(ts_randomizer_t *d, double m, double s)
{
	double x;
	d->hz = SHR3(d);
	d->iz = d->hz & 127;
	x = ((unsigned long)abs(d->hz) < d->kn[d->iz]) ? d->hz * d->wn[d->iz] : ts_random_normal_fix(d); // Generic version
	return x * s + m;
};

void ts_random_init(ts_randomizer_t *d, unsigned long jsrseed)
{
	const double m1 = 2147483648.0;

	double dn = 3.442619855899, tn = dn, vn = 9.91256303526217e-3, q;
	int i;
	d->jsr = jsrseed;

	// Set up tables for Normal	  
	q = vn / exp(-.5*dn*dn);
	d->kn[0] = (int)((dn / q)*m1);
	d->kn[1] = 0;
	d->wn[0] = q / m1;
	d->wnt[0] = q;
	d->wn[127] = dn / m1;
	d->wnt[127] = dn;
	d->fn[0] = 1.;
	d->fn[127] = exp(-0.5*dn*dn);
	for (i = 126; i >= 1; i--)
	{
		dn = sqrt(-2.0*log(vn / dn + exp(-0.5*dn*dn)));
		d->kn[i + 1] = (int)((dn / tn)*m1);
		tn = dn;
		d->fn[i] = exp(-0.5*dn*dn);
		d->wn[i] = dn / m1;
		d->wnt[i] = dn;
	}
}

double ts_random(ts_randomizer_t *d)
{
	return UNI(d);
}

long ts_random_int(ts_randomizer_t *d, long min, long max)
{
	// Output random integer in the interval min <= x <= max
	long r;
	r = (long)((max - min + 1) * ts_random(d)) + min; // Multiply interval with random and truncate
	if (r > max) r = max;
	if (max < min) return 0x80000000;
	return r;
}

ts_position_t ts_monte_carlo_search(ts_randomizer_t *randomizer, ts_scan_t *scan, 
									ts_map_t *map, ts_position_t *start_pos, 
									double sigma_xy, double sigma_theta, int stop, int *bd, int debug)
{
	ts_position_t currentpos;	//当前位姿
	ts_position_t bestpos;		//最佳位姿
	ts_position_t lastbestpos;	//上刻位姿
	int currentdist;			//当前似然度
	int bestdist;				//最佳似然度
	int lastbestdist;			//上刻似然度
	int counter = 0;			//迭代次数

	if (stop < 0) { stop = -stop;}

	currentpos = bestpos = lastbestpos = *start_pos;

	currentdist = ts_distance_scan_to_map(scan, map, &currentpos);

	bestdist = lastbestdist = currentdist;

	while (counter < stop)
	{
		currentpos = lastbestpos;
		currentpos.x = ts_random_normal(randomizer, currentpos.x, sigma_xy);
		currentpos.y = ts_random_normal(randomizer, currentpos.y, sigma_xy);
		currentpos.theta = ts_random_normal(randomizer, currentpos.theta, sigma_theta);

		currentdist = ts_distance_scan_to_map(scan, map, &currentpos);

		if (currentdist < bestdist)
		{
			bestdist = currentdist;
			bestpos = currentpos;
			if (debug) printf("Monte carlo ! %lg %lg %lg %d (count = %d)\n",
							  bestpos.x, bestpos.y, bestpos.theta, bestdist, counter);
		}
		else
		{
			counter++;
		}
		if ((counter > stop / 3) && (bestdist < lastbestdist))
		{
			lastbestpos = bestpos;
			lastbestdist = bestdist;
			counter = 0;
			sigma_xy *= 0.5;
			sigma_theta *= 0.5;
		}
	}

	if (bd) *bd = bestdist;

	return bestpos;
}

/*
 * 描述：根据激光雷达数据和上次坐标位置得到当前的最佳坐标位置
 * 输入：激光数据，地图，上次位置，调试标志位：1-执行调试输出
 * 返回：最佳坐标位置
 */
ts_position_t monte_carlo_move(ts_scan_t *scan, ts_map_t *map, ts_position_t *start_pos, int debug)
{
	ts_position_t currentpos;	//当前位姿
	ts_position_t bestpos;		//最佳位姿
	ts_position_t lastbestpos;	//上刻位姿
	int currentdist;			//当前似然度
	int bestdist;				//最佳似然度
	int lastbestdist;			//上刻似然度
	int counter = 0;			//迭代次数
	int rand_times = 1000;
	//初始化位姿变量
	currentpos = bestpos = lastbestpos = *start_pos;

	//计算上一刻位姿似然度（即激光点集与地图的匹配值）
	currentdist = ts_distance_scan_to_map(scan, map, &currentpos);

	//初始化似然度变量
	bestdist = lastbestdist = currentdist;

	//迭代1000次,当bestdist最小时退出
	while (counter < rand_times)
	{
        //取随机位置:在上刻位姿处在（-25,25）cm的范围内取随机位置，随机角度为(-25,25)度
		currentpos = lastbestpos;
		currentpos.x += 50 * (((double)rand()) / RAND_MAX - 0.5);
		currentpos.y += 50 * (((double)rand()) / RAND_MAX - 0.5);
		currentpos.theta += 15 * (((double)rand()) / RAND_MAX - 0.5);

		//计算随机位姿似然度（即激光点集与地图的匹配值）
		currentdist = ts_distance_scan_to_map(scan, map, &currentpos);

		//比较似然度（越小匹配度越高）
		if (currentdist < bestdist)
		{
			bestdist = currentdist;	//更新似然度
			bestpos = currentpos;	//更新位姿
			if (debug) printf("Monte carlo ! %lg %lg %lg %d (count = %d)\n",
							  bestpos.x, bestpos.y, bestpos.theta,
							  bestdist, counter);
		}
		else 
		{
			counter++;//继续取随机位姿
		}

		// 当迭代次数大于100且最佳似然度被更新时
		// 更新取随机位姿的基础且重新迭代
		if ((counter > rand_times/10) && (bestdist < lastbestdist))
		{
			lastbestpos = bestpos;
			lastbestdist = bestdist;
			counter = 0;
		}
	}
	//printf("bestdst:%d \n",bestdist);

	return bestpos;
}
