/*
 * rrLidar.hpp
 *
 *  Created on: Jan 28, 2017
 *      Author: davidj
 */

#ifndef SRC_RRLIDAR_HPP_
#define SRC_RRLIDAR_HPP_

#define LIDAR_I2C_DEFAULT_ADDR           0x62


class rrLidar {
	I2C* i2c_Lidar;

public:
	void init();
	void initTeleop();
	int GetLidarDistance();
	void i2cReset();

};

#endif /* SRC_RRLIDAR_HPP_ */
