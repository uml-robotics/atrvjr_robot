/*
 *  ATRVJR Driver, modified by Mikhail Medvedev from
 *  B21 Driver - By David Lu!! 2/2010
 *  Modified from Player code
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "rflex/atrvjr_driver.h"
#include "rflex/atrvjr_config.h"
//#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>

#include <chrono>
#include <math.h>
#include <cstdio>
#include <boost/thread.hpp>

ATRVJR::ATRVJR(rclcpp::Clock::SharedPtr cl):
   last_velocity_time(cl->now()), clock(cl)
{
    // Start watchdog thread
    boost::thread(boost::bind(&ATRVJR::watchdogThread, this));
//    bumps = new int*[2];
//
//    for (int index=0;index<2;index++) {
//        bumps[index] = new int[BUMPERS_PER[index]];
//        for (int i=0;i<BUMPERS_PER[index];i++) {
//            bumps[index][i] =0;
//        }
//    }

}

ATRVJR::~ATRVJR() {
//    delete bumps[0];
//    delete bumps[1];
//    delete bumps;
}


int ATRVJR::getNumBaseSonars() const {
    return SONARS_PER_RING[BASE_INDEX];
}

void ATRVJR::getBaseSonarReadings(double* readings) const {
    getSonarReadings(BASE_INDEX, readings);
}

//void ATRVJR::getBaseSonarPoints(sensor_msgs::PointCloud* cloud) const {
//    getSonarPoints(BASE_INDEX, cloud);
//}

//int ATRVJR::getBaseBumps(sensor_msgs::PointCloud* cloud) const {
//    return getBumps(BASE_INDEX, cloud);
//}

void ATRVJR::setSonarPower(bool on) {
    unsigned long echo, ping, set, val;
    if (on) {
        echo = SONAR_ECHO_DELAY;
        ping = SONAR_PING_DELAY;
        set = SONAR_SET_DELAY;
        val = 2;
    } else {
        echo = ping = set = val = 0;
    }
    configureSonar(echo, ping, set, val);
}


void ATRVJR::getSonarReadings(const int ringi, double* adjusted_ranges) const {
    int i = 0;
    for (int x = SONAR_RING_BANK_BOUND[ringi];x<SONAR_RING_BANK_BOUND[ringi+1];x++) {
        for (int y=0;y<SONARS_PER_BANK[x];y++) {
            int range = sonar_ranges[x*SONAR_MAX_PER_BANK+y];
            if (range > SONAR_MAX_RANGE)
                range = SONAR_MAX_RANGE;
            double fRange = range / (double) RANGE_CONVERSION;
            adjusted_ranges[i] = fRange;
            i++;
        }
    }
}
/*
void ATRVJR::getSonarPoints(const int ringi, sensor_msgs::PointCloud* cloud) const {
    int numSonar = SONARS_PER_RING[ringi];
    double* readings = new double[numSonar];
    getSonarReadings(ringi, readings);
    cloud->points.resize(numSonar);
    int c = 0;
    for (int i = 0; i < numSonar; ++i) {
        if (readings[i] < SONAR_MAX_RANGE/ (double) RANGE_CONVERSION) {
            double angle =  SONAR_RING_START_ANGLE[ringi] + SONAR_RING_ANGLE_INC[ringi]*i;
            angle *= M_PI / 180.0;

            double d = SONAR_RING_DIAMETER[ringi] + readings[i];
            cloud->points[c].x = cos(angle)*d;
            cloud->points[c].y = sin(angle)*d;
            cloud->points[c].z = SONAR_RING_HEIGHT[ringi];
            c++;
        }
    }
}
*/
//int ATRVJR::getBumps(const int index, sensor_msgs::PointCloud* cloud) const {
//    int c = 0;
//    double wedge = 2 * M_PI / BUMPERS_PER[index];
//    double d = SONAR_RING_DIAMETER[index]*1.1;
//    int total = 0;
//    for (int i=0;i<BUMPERS_PER[index];i++) {
//        int value = bumps[index][i];
//        for (int j=0;j<4;j++) {
//            int mask = 1 << j;
//            if ((value & mask) > 0) {
//                total++;
//            }
//        }
//    }
//
//    cloud->points.resize(total);
//    if (total==0)
//        return 0;
//    for (int i=0;i<BUMPERS_PER[index];i++) {
//        int value = bumps[index][i];
//        double angle = wedge * (2.5 - i);
//        for (int j=0;j<4;j++) {
//            int mask = 1 << j;
//            if ((value & mask) > 0) {
//                double aoff = BUMPER_ANGLE_OFFSET[j]*wedge/3;
//                cloud->points[c].x = cos(angle-aoff)*d;
//                cloud->points[c].y = sin(angle-aoff)*d;
//                cloud->points[c].z = BUMPER_HEIGHT_OFFSET[index][j];
//                c++;
//            }
//        }
//    }
//    return total;
//
//}

void ATRVJR::processDioEvent(unsigned char address, unsigned short data) {

    if (address == HEADING_HOME_ADDRESS) {
//        home_bearing = bearing;
//        printf("ATRVJR Home %f \n", config.driverAngle2real(home_bearing));
//    }// check if the dio packet came from a bumper packet
//    else if ((address >= BUMPER_ADDRESS) && (address < (BUMPER_ADDRESS+BUMPER_COUNT))) {
//        int index =0, rot = address - BUMPER_ADDRESS;
//        if (rot > BUMPERS_PER[index]) {
//            rot -= BUMPERS_PER[index];
//            index++;
//        }
//        bumps[index][rot] = data;
//        bumpsUpdateSignal.invoke();
    } else {
        printf("ATRVJR DIO: address 0x%02x (%d) value 0x%02x (%d)\n", address, address, data, data);
    }
}

void ATRVJR::setVelocity(const double tvel, const double rvel) {
    //last_velocity_time = rclcpp::Clock(RCL_ROS_TIME).now();
    last_velocity_time = clock->now();
    RFLEX::setVelocity(tvel, rvel);
}

void ATRVJR::watchdogThread() {
    while (true) {
        if (clock->now() - last_velocity_time > rclcpp::Duration(0.5, 0)) {
            RFLEX::setVelocity(0, 0);
        }
        usleep(100000);
    }
}
