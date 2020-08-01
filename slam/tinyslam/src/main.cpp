#include "CoreSLAM.h"

#define TEST_FILENAME "../test_data/test_lab2.dat"
#define TEST_MAX_SCANS 5000
#define TEST_SCAN_SIZE 682

ts_map_t trajectory, map, loop_map;
ts_sensor_data_t sensor_data[TEST_MAX_SCANS];

int ts_read_scans(char *filename, ts_sensor_data_t *sensor_data)
{
    FILE *input;
    char *str, line[4000];
    int nbscans = 0;//传感器数据个数

    input = fopen(filename, "rt");
    do {
        // Read the scan
        str = fgets(line, 4000, input);
        if (str == NULL) break;
        str = strtok(str, " ");
        sscanf(str, "%u", &sensor_data[nbscans].timestamp);
        str = strtok(NULL, " ");
        str = strtok(NULL, " ");
        sscanf(str, "%d", &sensor_data[nbscans].q1);
        str = strtok(NULL, " ");
        sscanf(str, "%d", &sensor_data[nbscans].q2);
        for (int i = 0; i < 10; i++)
            str = strtok(NULL, " ");
        for (int i = 0; i < TEST_SCAN_SIZE; i++)
        {
            if (str)
            {
                sscanf(str, "%d", &sensor_data[nbscans].d[i]);
                str = strtok(NULL, " ");
            }
            else sensor_data[nbscans].d[i] = 0;
        }
        nbscans++;
    } while (1);
    fclose(input);
    return nbscans;
}

///建图:建出来的地图是和实际一样
void draw_map2(ts_map_t *map, ts_map_t *overlay, int width, int height)
{
    cv::Mat image(2048, 2048, CV_8UC3, cv::Scalar(255,255,255));

    int y = (TS_MAP_SIZE - height) / 2;
    for (int yp = 0; yp < height; y++, yp++)
    {
        int x = (TS_MAP_SIZE - width) / 2;
        for (int xp = 0; xp < width; x++, xp++)
        {
            unsigned char *p = image.data + yp*width * 3 + xp * 3;
            if (overlay->map[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0)//显示轨迹??
            {
                p[0] = 0; p[1] = 0; p[2] = 255;
            }
            else if (overlay->map[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 254)//显示轨迹
            {
                p[0] = 0; p[1] = 0; p[2] = 255;
                cv::circle(image,cv::Point(x,y),5,cv::Scalar(0,0,255),-1);
            }
            else//显示地图
            {
                p[0] = (map->map[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8;
                p[1] = (map->map[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8;
                p[2] = (map->map[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8;
            }
        }
    }
    cv::namedWindow("tinyslam", cv::WINDOW_NORMAL);
    cv::imshow("tinyslam", image);
    cv::waitKey(1);
}

int main(int argc, char *argv[])
{
    ts_position_t startpos, position;
    ts_position_t loop_startpos, loop_endpos;
    int quality;
    int nbscans;//传感器个数
    ts_robot_parameters_t params;
    ts_laser_parameters_t laser_params;
    ts_state_t state;
    int  loop_start, nb_loops, loop_end[2];

    ///----------------------------------------------------------------------
    /// init
    // Read the scans from data file
    printf("Loading data from file %s...\n", TEST_FILENAME);
    int maxscans = ts_read_scans(TEST_FILENAME, sensor_data);

    if (argc == 3) {
        nb_loops = 2;
        loop_end[0] = atoi(argv[2]);
    }
    else
        nb_loops = 1;
    loop_end[nb_loops - 1] = maxscans;

    params.r = 0.077;
    params.R = 0.165;
    params.inc = 2000;
    params.ratio = 1.0;

    laser_params.offset = 145;
    laser_params.scan_size = TEST_SCAN_SIZE;
    laser_params.angle_min = -120;
    laser_params.angle_max = +120;
    laser_params.detection_margin = 70;
    laser_params.distance_no_detection = 4000;

    startpos.x = 0.5 * TS_MAP_SIZE * TS_MAP_SCALE;
    startpos.y = 0.5 * TS_MAP_SIZE * TS_MAP_SCALE;
    startpos.theta = 0;

    ts_map_init(&map);
    ts_map_init(&trajectory);
    ts_map_init(&loop_map);

    loop_start = 0;
    loop_startpos = startpos;
    ///----------------------------------------------------------------------
    ///
    for (int loop = 0; loop != nb_loops; loop++)
    {
        ///----------------------------------------------------------------------
        ///
        ts_state_init(&state, &map, &params, &laser_params, &loop_startpos,
                      100, 20, 600, TS_DIRECTION_FORWARD);
        for (nbscans = loop_start; nbscans != loop_end[loop]; nbscans++)
        {
            ts_iterative_map_building(&sensor_data[nbscans], &state);

            printf("#%d : %lg %lg %lg\n", nbscans, state.position.x, state.position.y,
                   state.position.theta);

            if (nbscans == 50) { loop_map = map; }

            int x = (int)floor(state.position.x / TS_MAP_SCALE + 0.5);
            int y = ((int)floor(state.position.y / TS_MAP_SCALE + 0.5));
            if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
            {
                trajectory.map[y * TS_MAP_SIZE + x] = 0;
            }
            draw_map2(&map, &trajectory, 2048, 2048);
        }
        printf("Forward distance: %lg\n", state.distance);
        printf("Starting point : %lg %lg %lg\n", startpos.x, startpos.y, startpos.theta);
        printf("End point : %lg %lg %lg\n",
               state.position.x, state.position.y, state.position.theta);
        ///----------------------------------------------------------------------
        ///
        printf("Try to close the loop...\n");

        position = state.position;
        position.x += state.laser_params.offset * cos(position.theta * M_PI / 180);
        position.y += state.laser_params.offset * sin(position.theta * M_PI / 180);
        loop_endpos = ts_close_loop_position(&state, &sensor_data[loop_end[loop] - 1],
                                             &loop_map, &position, &quality);
        loop_endpos.x -= state.laser_params.offset * cos(loop_endpos.theta * M_PI / 180);
        loop_endpos.y -= state.laser_params.offset * sin(loop_endpos.theta * M_PI / 180);

        printf("Loop close point : %lg %lg %lg (%d)\n", loop_endpos.x, loop_endpos.y, loop_endpos.theta, quality);
        ///----------------------------------------------------------------------
        ///
        map = loop_map;
        ts_map_init(&trajectory);

        ts_state_init(&state, &map, &params, &laser_params, &loop_endpos,
                      100, 20, 600, TS_DIRECTION_BACKWARD);
        for (nbscans = loop_end[loop] - 1; nbscans >= loop_start; nbscans--)
        {
            ts_iterative_map_building(&sensor_data[nbscans], &state);

            printf("#%d : %lg %lg %lg\n", nbscans, state.position.x, state.position.y, state.position.theta);

            int x = (int)floor(state.position.x / TS_MAP_SCALE + 0.5);
            int y = ((int)floor(state.position.y / TS_MAP_SCALE + 0.5));
            if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
                trajectory.map[y * TS_MAP_SIZE + x] = 254;

            draw_map2(&map, &trajectory, 2048, 2048);
        }
        printf("Backward distance: %lg\n", state.distance);
        printf("Starting point : %lg %lg %lg\n", startpos.x, startpos.y, startpos.theta);
        printf("End point : %lg %lg %lg\n",
               state.position.x, state.position.y, state.position.theta);
        ///----------------------------------------------------------------------
        ///
        printf("Closing the loop...\n");
        ts_close_loop_trajectory(&sensor_data[loop_start], loop_end[loop] - loop_start,
                                 &loop_startpos, &loop_endpos);
        draw_map2(&map, &trajectory, 2048, 2048);

        loop_map = map;

        loop_startpos = loop_endpos;
        loop_start = loop_end[loop];
    }

    state.done = 1;
    printf("Done\n");
    cv::waitKey(0);

    return 0;
}
