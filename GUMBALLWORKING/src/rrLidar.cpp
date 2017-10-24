/*
 * rrLidar.cpp
 *
 *  Created on: Jan 28, 2017
 *      Author: davidj
 */
#include "WPILib.h"
#include "rrLidar.hpp"
	void rrLidar::i2cReset(){
		unsigned char LidarMassData[6];
		i2c_Lidar->ReadOnly(6,LidarMassData);

	}
	void rrLidar::init() {

		i2c_Lidar = new I2C(I2C::Port::kOnboard, LIDAR_I2C_DEFAULT_ADDR); //(I2C::Port::kOnboard or kMXP, Lidar Address)
		//i2c_Lidar->Write(0x00, 0x00);  // Reset Lidar, duration 25 ms
	}
	void rrLidar::initTeleop(){
		i2c_Lidar->Write(0x00, 0x00); // Reset Lidar, duration 25 ms
		Wait(.025);
		i2c_Lidar->Write(0x00, 0x04);
	}

	int rrLidar::GetLidarDistance(){
		unsigned char djoSendData[2];
		unsigned char LidarMassData[2];
		int distanceMeasured = 0;


		i2c_Lidar->Write(0x00, 0x04);
		Wait(.003);
		djoSendData[0] = 0x8F;  // Bulk Write Based Read_Only for a single register Read of 0x01
		i2c_Lidar->WriteBulk(djoSendData,1);
		i2c_Lidar->ReadOnly(2,LidarMassData);

		distanceMeasured = LidarMassData[1] * 1 + LidarMassData[0] * 256;

		return distanceMeasured;

	}
