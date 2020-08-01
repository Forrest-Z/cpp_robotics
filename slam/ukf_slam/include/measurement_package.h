#pragma once

#include "../lib/Eigen/Dense"
#include "common.h"

/// odom data
struct OdoReading
{
	float r1;   //
	float t;
	float r2;
};

/// redar data
struct RadarReading
{
	long long id;   // landmark id
	float range;    // landmark距离机器人的距离
	float bearing;  // landmark位于机器人的方向
};

/// 观测一次的数据
class Record
{
public:
	Record(){};
	OdoReading odo; // odom data
	vector<RadarReading> radars; // redar data list
};

/// 所有观测数据
class MeasurementPackage
{
public:

    vector<Record> data; // 观测数据集

    // get all redar data list
    void initialize(const string& filename)
    {
        // open data file
        ifstream in_file(filename, ifstream::in);
        if (!in_file.is_open())
        {
            cerr << "Cannot open input file: " << filename << endl;
            exit(EXIT_FAILURE);
        }

        // get data
        string line;
        Record record;
        int index = 0;
        while (getline(in_file, line))
        {
            string sensor_type;
            istringstream ss(line);
            ss >> sensor_type;
            //measurement type r1 t r2
            if (sensor_type.compare("ODOMETRY") == 0)
            {
                //end the first record;
                if (record.radars.size() != 0)
                {
                    data.push_back(record);
                    record.radars.clear();
                    if (debug && index < 50)
                        cout << index << "-----------" << endl;
                    index++;
                }
                auto& odo = record.odo;
                ss >> odo.r1;
                ss >> odo.t;
                ss >> odo.r2;
                if(debug && index < 50)
                    cout << record.odo.r1 << ": " << record.odo.t << ": " << record.odo.r2 << endl;
            }
            else if(sensor_type.compare("SENSOR") == 0)
            {
                auto& radars = record.radars;
                RadarReading radarR;
                ss >> radarR.id;
                ss >> radarR.range;
                ss >> radarR.bearing;
                radars.push_back(radarR);
                if (debug && index < 50)
                    cout << radars.back().id << ": " << radars.back().range << ": " << radars.back().bearing << endl;
            }else if(sensor_type.compare("SENSOR") == 0)
            {
                auto& radars = record.radars;
                RadarReading radarR;
                ss >> radarR.id;
                ss >> radarR.range;
                ss >> radarR.bearing;
                radars.push_back(radarR);
                if (debug && index < 50)
                    cout << radars.back().id << ": " << radars.back().range << ": " << radars.back().bearing << endl;
            }
        }

        if (record.radars.size() != 0)
        {
           data.push_back(record);
        }

        // close data file
        if (in_file.is_open())
        {
            in_file.close();
        }
    }
};