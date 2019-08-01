#include "IMUHandler.h"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

template<typename T>
constexpr T Pi{3.1415926535897932385L};

std::vector<std::vector<double>> loadData(std::string filename) {
    std::ifstream fin(filename);
	if (! fin.is_open())
        std::cout << "file: " << filename << "open error!\n";

    std::vector<std::vector<double>> ret;
    std::string line;
    while(std::getline(fin, line)) {
		std::vector<double> data;
        std::stringstream str(line);
        double val;
        while(str >> val) 
            data.push_back(val);
        ret.push_back(data);
	}
    std::cout << ret.size() << '\n';
    return ret;
}


int main(int argc, char* argv[]) {
    auto imu_data = loadData(argv[1]); 
    
    IMUIntegration handler;
    for(auto i = 0; i < imu_data.size(); ++ i) {
		double time_stamp = imu_data[i][0];
        Eigen::Vector3d ang_v(imu_data[i][2], imu_data[i][1], imu_data[i][3]);
        Eigen::Vector3d acc_v(imu_data[i][5], imu_data[i][4], imu_data[i][6]);
        IMUState imu_cur(time_stamp, ang_v, acc_v);
        handler.Integrate(imu_cur);
        std::cout << handler.getPos()(0) << '\t' << handler.getPos()(1) << '\t' << handler.getPos()(2) << '\n';
    }
	return 0;
}
