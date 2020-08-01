#include "../include/common.h"
#include "../include/mapper.h"
#include "../include/measurement_package.h"
#include "../include/ukf_slam.h"
#include "../include/draw.h"

using namespace std;

int main()
{
    // get data file path
    string in_map_name = "../data/world.dat";
    string in_sensor_name = "../data/sensor_data.dat";

    // read the map data for all landmarks
    Mapper mapper;
    mapper.initialize(in_map_name);

    // read the measurements with odometry and radar data
    MeasurementPackage measurements;
    measurements.initialize(in_sensor_name);
    cout << "measurements data size:" << measurements.data.size() << endl;

    Draw draw;
    UKFSLam ukfslam;
    ukfslam.Initialize();

    for (int i = 0; i < measurements.data.size(); i++)
    {
        draw.Clear();

        const auto& record = measurements.data[i];
        ukfslam.ProcessMeasurement(record);
        cout << "robot pose:["
             << ukfslam.mu(0) << ","
             << ukfslam.mu(1) << ","
             << ukfslam.mu(2)*180.0/3.14 << "]" << endl;
        for(int i = 0; i < ukfslam.landmarks.size(); i++)
        {
            cout << "landmark " << ukfslam.landmarks[i] << " pose:["
                 << ukfslam.mu(2*i + 3) << ","
                 << ukfslam.mu(2*i + 4) << "]" << endl;
        }
        cout << endl;

        draw.Plot_state(ukfslam.mu, // ukf估计机器人位姿和landmark位姿
                        ukfslam.Sigma, // ukf估计机器人位姿误差
                        mapper, // 原始landmark位姿
                        ukfslam.landmarks, // ukf估计landmark id list
                        record.radars); // 观测一次的数据(odom and landmark pose)
        draw.Pause();
    }

    draw.Show();

    return 0;
}
