#include <ros/ros.h>
#include <sys/stat.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

std::string mapFrame = "map";
std::string odomFrame = "odom";
std::string baseLinkFrame = "base_link";
std::string flaserFrame = "flaser";
std::string rlaserFrame = "rlaser";
std::string rawLaser1Frame = "laser1";
std::string rawLaser2Frame = "laser2";
std::string rawLaser3Frame = "laser3";
std::string rawLaser4Frame = "laser4";
std::string robotLaser0Frame = "robot_laser0";
std::string robotLaser1Frame = "robot_laser1";
std::string odomName = "/odom";
std::string flaserName = "/fscan";
std::string rlaserName = "/rscan";
std::string rawLaser1Name = "/scan1";
std::string rawLaser2Name = "/scan2";
std::string rawLaser3Name = "/scan3";
std::string rawLaser4Name = "/scan4";
std::string robotLaser0Name = "/robot_scan0";
std::string robotLaser1Name = "/robot_scan1";
std::string laserPoseName = "/laser_pose";
std::string robotPoseName = "/robot_pose";
bool broadcastBaseLink2LaserTF = true;
bool broadcastOdomTF = true;
bool offsetPose = true;
double skipTime = 0.0;
double finishTime = -1.0;

std::vector<sensor_msgs::LaserScan> flaserMsgs, rlaserMsgs;
std::vector<sensor_msgs::LaserScan> rawLaser1Msgs, rawLaser2Msgs, rawLaser3Msgs, rawLaser4Msgs;
std::vector<sensor_msgs::LaserScan> robotLaser0Msgs, robotLaser1Msgs;
std::vector<nav_msgs::Odometry> odomMsgs;
std::vector<geometry_msgs::PoseStamped> laserPoseMsgs, robotPoseMsgs;
double offsetPoseX, offsetPoseY, offsetPoseTheta;
double frontLaserOffset, rearLaserOffset;

typedef struct {
    double ipcTimestamp;
    std::string ipcHostname;
    double loggerTimestamp;
} Footer;

typedef struct {
    // ODOM x y theta tv rv accel
    double x, y, theta, tv, rv, accel;
    Footer footer;
} Odom;

typedef struct {
    // TRUEPOS true_x true_y true_theta odom_x odom_y odom_theta
     double trueX, trueY, trueTheta, odomX, odomY, odomTheta;
    Footer footer;
} TruePose;

typedef struct {
    // *LASER num_readings [range_readings] x y theta odom_x odom_y odom_theta
    // * is F or R
    int numReadings;
    std::vector<double> rangeReadings;
    double x, y, theta, odomX, odomY, odomTheta;
    Footer footer;
} Laser;

typedef struct {
    // RAWLASER* laser_type start_angle field_of_view angular_resolution maximum_range accuracy
    // remission_mode num_readings [range_readings] num_remissions [remission values];
    // * is 1, 2, 3, or 4
    int laserType;
    double startAngle, fieldOfView, angularResolution, maximumRange, accuracy;
    int remissionMode, numReadings;
    std::vector<double> rangeReadings;
    int numRemissions;
    std::vector<double> remissionValues;
    Footer footer;
} RawLaser;

typedef struct {
    // ROBOTLASER* laser_type start_angle field_of_view angular_resolution maximum_range accuracy
    // remission_mode num_readings [range_readings]
    // laser_pose_x laser_pose_y laser_pose_theta robot_pose_x robot_pose_y robot_pose_theta
    // laser_tv laser_rv forward_safety_dist side_safty_dist
    // * is 0 or 1
    int laserType;
    double startAngle, fieldOfView, angularResolution, maximumRange, accuracy;
    int remissionMode, numReadings;
    std::vector<double> rangeReadings;
    int numRemissions;
    std::vector<double> remissionValues;
    double laserPoseX, laserPoseY, laserPoseTheta, robotPoseX, robotPoseY, robotPoseTheta;
    double laserTv, laserRv, forwardSafetyDist, sideSaftyDist;
    Footer footer;
} RobotLaser;

std::vector<std::string> splitLineData(char *lineData) {
    std::vector<std::string> data;
    int i = 0, idx = 0;
    for (;;) {
        if (lineData[i] == ' ' || ((int)lineData[i] == 0 && (int)lineData[i + 1] == 0)) {
            std::string d = "";
            for (int j = idx; j < i; j++)
                d += lineData[j];
            data.push_back(d);
            idx = i + 1;
            if ((int)lineData[i] == 0 && (int)lineData[i + 1] == 0)
                break;
        }
        i++;
    }
    return data;
}

Odom parseOdom(std::vector<std::string> data) {
    Odom odom;
    odom.x = stod(data[1]);
    odom.y = stod(data[2]);
    odom.theta = stod(data[3]);
    odom.tv = stod(data[4]);
    odom.rv = stod(data[5]);
    odom.accel = stod(data[6]);
    odom.footer.ipcTimestamp = stod(data[7]);
    odom.footer.ipcHostname = data[8];
    odom.footer.loggerTimestamp = stod(data[9]);
    return odom;
}

TruePose parseTruePose(std::vector<std::string> data) {
    TruePose truePose;
    truePose.trueX = stod(data[1]);
    truePose.trueY = stod(data[2]);
    truePose.trueTheta = stod(data[3]);
    truePose.odomX = stod(data[4]);
    truePose.odomY = stod(data[5]);
    truePose.odomTheta = stod(data[6]);
    truePose.footer.ipcTimestamp = stod(data[7]);
    truePose.footer.ipcHostname = data[8];
    truePose.footer.loggerTimestamp = stod(data[9]);
    return truePose;
}

Laser parseLaser(std::vector<std::string> data) {
    Laser laser;
    laser.numReadings = stoi(data[1]);
    laser.rangeReadings.resize(laser.numReadings);
    for (int i = 0; i < laser.numReadings; ++i)
        laser.rangeReadings[i] = stod(data[2 + i]);
    laser.x = stod(data[2 + laser.numReadings]);
    laser.y = stod(data[2 + laser.numReadings + 1]);
    laser.theta = stod(data[2 + laser.numReadings + 2]);
    laser.odomX = stod(data[2 + laser.numReadings + 3]);
    laser.odomY = stod(data[2 + laser.numReadings + 4]);
    laser.odomTheta = stod(data[2 + laser.numReadings + 5]);
    laser.footer.ipcTimestamp = stod(data[2 + laser.numReadings + 6]);
    laser.footer.ipcHostname = data[2 + laser.numReadings + 7];
    laser.footer.loggerTimestamp = stod(data[2 + laser.numReadings + 8]);
    return laser;
}

RawLaser parseRawLaser(std::vector<std::string> data) {
    RawLaser rawLaser;
    rawLaser.laserType = stoi(data[1]);
    rawLaser.startAngle = stod(data[2]);
    rawLaser.fieldOfView = stod(data[3]);
    rawLaser.angularResolution = stod(data[4]);
//    rawLaser.maximumRange = stod(data[5]);
    rawLaser.maximumRange = stod(data[5]) - 1.0;
    rawLaser.accuracy = stod(data[6]);
    rawLaser.remissionMode = stoi(data[7]);
    rawLaser.numReadings = stoi(data[8]);
    rawLaser.rangeReadings.resize(rawLaser.numReadings);
    for (int i = 0; i < rawLaser.numReadings; ++i)
        rawLaser.rangeReadings[i] = stod(data[9 + i]);
    rawLaser.numRemissions = stoi(data[9 + rawLaser.numReadings]);
    rawLaser.remissionValues.resize(rawLaser.numRemissions);
    for (int i = 0; i < rawLaser.numRemissions; ++i)
        rawLaser.remissionValues[i] = stod(data[10 + rawLaser.numReadings + i]);
    rawLaser.footer.ipcTimestamp = stod(data[10 + rawLaser.numReadings + rawLaser.numRemissions]);
    rawLaser.footer.ipcHostname = data[10 + rawLaser.numReadings + rawLaser.numRemissions + 1];
    rawLaser.footer.loggerTimestamp = stod(data[10 + rawLaser.numReadings + rawLaser.numRemissions + 2]);
    return rawLaser;
}

RobotLaser parseRobotLaser(std::vector<std::string> data) {
    RobotLaser robotLaser;
    robotLaser.laserType = stoi(data[1]);
    robotLaser.startAngle = stod(data[2]);
    robotLaser.fieldOfView = stod(data[3]);
    robotLaser.angularResolution = stod(data[4]);
    robotLaser.maximumRange = stod(data[5]);
    robotLaser.accuracy = stod(data[6]);
    robotLaser.remissionMode = stoi(data[7]);
    robotLaser.numReadings = stoi(data[8]);
    robotLaser.rangeReadings.resize(robotLaser.numReadings);
    for (int i = 0; i < robotLaser.numReadings; ++i)
        robotLaser.rangeReadings[i] = stod(data[9 + i]);
    robotLaser.numRemissions = stoi(data[9 + robotLaser.numReadings]);
    robotLaser.remissionValues.resize(robotLaser.numRemissions);
    for (int i = 0; i < robotLaser.numRemissions; ++i)
        robotLaser.remissionValues[i] = stod(data[10 + robotLaser.numReadings + i]);
    robotLaser.laserPoseX = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions]);
    robotLaser.laserPoseY = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 1]);
    robotLaser.laserPoseTheta = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 2]);
    robotLaser.robotPoseX = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 3]);
    robotLaser.robotPoseY = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 4 ]);
    robotLaser.robotPoseTheta = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 5]);
    robotLaser.laserTv = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 6]);
    robotLaser.laserRv = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 7]);
    robotLaser.forwardSafetyDist = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 8]);
    robotLaser.sideSaftyDist = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 9]);
    // unknown data = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 10]);
    robotLaser.footer.ipcTimestamp = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 11]);
    robotLaser.footer.ipcHostname = data[10 + robotLaser.numReadings + robotLaser.numRemissions + 12];
    robotLaser.footer.loggerTimestamp = stod(data[10 + robotLaser.numReadings + robotLaser.numRemissions + 13]);
    return robotLaser;
}

nav_msgs::Odometry makeOdomMsg(Odom odom) {
    nav_msgs::Odometry odomMsg;
    odomMsg.header.frame_id = odomFrame;
    odomMsg.header.stamp = ros::Time(odom.footer.loggerTimestamp);
    odomMsg.child_frame_id = baseLinkFrame;
    odomMsg.pose.pose.position.x = odom.x;
    odomMsg.pose.pose.position.y = odom.y;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom.theta);
    odomMsg.twist.twist.linear.x = odom.tv;
    odomMsg.twist.twist.angular.z = odom.rv;
    return odomMsg;
}

sensor_msgs::LaserScan makeLaserMsg(Laser laser) {
    sensor_msgs::LaserScan laserMsg;
    laserMsg.header.stamp = ros::Time(laser.footer.loggerTimestamp);
    laserMsg.angle_min = -M_PI / 2.0;
    laserMsg.angle_max = M_PI / 2.0;
    laserMsg.angle_increment = (180.0 / (double)laser.numReadings) * M_PI / 180.0;
    laserMsg.scan_time = 10.0e-9;
    laserMsg.scan_time = 1.0 / 40.0;
    laserMsg.range_min = 0.02;
    laserMsg.range_max = 81.83 - 1.0;
    laserMsg.ranges.resize(laser.numReadings);
    laserMsg.intensities.resize(laser.numReadings);
    for (int i = 0; i < laser.numReadings; ++i) {
        laserMsg.ranges[i] = laser.rangeReadings[i];
        laserMsg.intensities[i] = 0.0;
    }
    return laserMsg;
}

sensor_msgs::LaserScan makeRawLaserMsg(RawLaser rawLaser) {
    sensor_msgs::LaserScan laserMsg;
    laserMsg.header.stamp = ros::Time(rawLaser.footer.loggerTimestamp);
    laserMsg.angle_min = rawLaser.startAngle;
    laserMsg.angle_max = rawLaser.startAngle + rawLaser.fieldOfView;
    laserMsg.angle_increment = rawLaser.angularResolution;
    laserMsg.scan_time = 10.0e-9;
    laserMsg.scan_time = 1.0 / 40.0;
    laserMsg.range_min = 0.02;
    laserMsg.range_max = rawLaser.maximumRange;
    laserMsg.ranges.resize(rawLaser.numReadings);
    laserMsg.intensities.resize(rawLaser.numReadings);
    for (int i = 0; i < rawLaser.numReadings; ++i) {
        laserMsg.ranges[i] = rawLaser.rangeReadings[i];
        laserMsg.intensities[i] = 0.0;
    }
    return laserMsg;
}

geometry_msgs::PoseStamped makePoseMsg(double timestamp, double x, double y, double theta) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = mapFrame;
    pose.header.stamp = ros::Time(timestamp);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    return pose;
}

void quat2rpy(double qx, double qy, double qz, double qw, double *r, double *p, double *y) {
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (r != NULL)
        *r = roll;
    if (p != NULL)
        *p = pitch;
    if (y != NULL)
        *y = yaw;
}

void parseLogFile(std::string logFile) {
    FILE *fp = fopen(logFile.c_str(), "r");
    if (fp == NULL) {
        fprintf(stderr, "Cannot open log file -> %s\n", logFile.c_str());
        exit(1);
    }

    struct stat sb;
    stat(logFile.c_str(), &sb);
    char *lineData = (char *)malloc(sb.st_size);
    double odomTVSum = 0.0, odomRVSum = 0.0;

    while (fscanf(fp, "%[^\n] ", lineData) != EOF) {
        std::vector<std::string> data = splitLineData(lineData);
        if (data[0] == "PARAM") {
            if (data[1] == "robot_frontlaser_offset") {
                frontLaserOffset = stod(data[2]);
            } else if (data[1] == "robot_rearlaser_offset") {
                rearLaserOffset = stod(data[2]);
            }
        } else if (data[0] == "SYNC") {
            std::string tagname = data[1];
        } else if (data[0] == "ODOM") {
            Odom odom = parseOdom(data);
            odomTVSum += odom.tv;
            odomRVSum += odom.rv;
            if ((int)odomMsgs.size() == 0) {
                offsetPoseX = odom.x;
                offsetPoseY = odom.y;
                offsetPoseTheta = odom.theta;
            }
            nav_msgs::Odometry odomMsg = makeOdomMsg(odom);
            if ((int)odomMsgs.size() > 0) {
                int idx = (int)odomMsgs.size() - 1;
                double dt = odomMsg.header.stamp.toSec() - odomMsgs[idx].header.stamp.toSec();
                if (dt > 0.05)
                    odomMsgs.push_back(odomMsg);
            } else {
                odomMsgs.push_back(odomMsg);
            }

/*
            printf("ODOM %lf %lf %lf %lf %lf %lf %lf %s %lf\n",
                odom.x, odom.y, odom.theta, odom.tv, odom.rv, odom.accel,
                odom.footer.ipcTimestamp, odom.footer.ipcHostname.c_str(), odom.footer.loggerTimestamp);
 */
        } else if (data[0] == "TRUEPOS") {
            TruePose truePose = parseTruePose(data);
/*
            printf("TRUEPOS %lf %lf %lf %lf %lf %lf %lf %s %lf\n",
                truePose.trueX, truePose.trueY, truePose.trueTheta,
                truePose.odomX, truePose.odomY, truePose.odomTheta,
                truePose.footer.ipcTimestamp, truePose.footer.ipcHostname.c_str(), truePose.footer.loggerTimestamp);
 */
        } else if (data[0] == "FLASER" || data[0] == "RLASER") {
            Laser laser = parseLaser(data);
            sensor_msgs::LaserScan laserMsg = makeLaserMsg(laser);
            if (data[0] == "FLASER") {
                laserMsg.header.frame_id = flaserFrame;
                flaserMsgs.push_back(laserMsg);
            } else {
                laserMsg.header.frame_id = rlaserFrame;
                rlaserMsgs.push_back(laserMsg);
            }
/*
            printf("%d\n", laser.numReadings);
            for (int i = 0; i < laser.numReadings; ++i)
                printf("%lf ", laser.rangeReadings[i]);
            printf("\n");
            printf("%lf %lf %lf"
                "%lf %lf %lf\n",
                laser.x, laser.y, laser.theta,
                laser.odomX, laser.odomY, laser.odomTheta);
            printf("%lf %s %lf\n",
                laser.footer.ipcTimestamp, laser.footer.ipcHostname.c_str(), laser.footer.loggerTimestamp);
 */
        } else if (data[0] == "RAWLASER1" || data[0] == "RAWLASER2" || data[0] == "RAWLASER3" || data[0] == "RAWLASER4") {
            RawLaser rawLaser = parseRawLaser(data);
            sensor_msgs::LaserScan laserMsg = makeRawLaserMsg(rawLaser);
            if (data[0] == "RAWLASER1") {
                laserMsg.header.frame_id = rawLaser1Frame;
                rawLaser1Msgs.push_back(laserMsg);
            } else if (data[0] == "RAWLASER2") {
                laserMsg.header.frame_id = rawLaser2Frame;
                rawLaser2Msgs.push_back(laserMsg);
            } else if (data[0] == "RAWLASER3") {
                laserMsg.header.frame_id = rawLaser3Frame;
                rawLaser3Msgs.push_back(laserMsg);
            } else {
                laserMsg.header.frame_id = rawLaser4Frame;
                rawLaser4Msgs.push_back(laserMsg);
            }
/*
            printf("%s %d "
                "%lf %lf %lf "
                "%lf %lf "
                "%d %d %d\n",
                data[0].c_str(), rawLaser.laserType,
                rawLaser.startAngle, rawLaser.fieldOfView, rawLaser.angularResolution,
                rawLaser.maximumRange, rawLaser.accuracy,
                rawLaser.remissionMode, rawLaser.numReadings, rawLaser.numRemissions);
            for (int i = 0; i < rawLaser.numReadings; ++i)
                printf("%lf ", rawLaser.rangeReadings[i]);
            printf("\n");
            for (int i = 0; i < rawLaser.numRemissions; ++i)
                printf("%lf ", rawLaser.remissionValues[i]);
            printf("\n");
            printf("%lf %s %lf\n",
                rawLaser.footer.ipcTimestamp, rawLaser.footer.ipcHostname.c_str(), rawLaser.footer.loggerTimestamp);
 */
        } else if (data[0] == "ROBOTLASER0" || data[0] == "ROBOTLASER1") {
            RobotLaser robotLaser = parseRobotLaser(data);
            geometry_msgs::PoseStamped laserPoseMsg = makePoseMsg(robotLaser.footer.loggerTimestamp,
                robotLaser.laserPoseX, robotLaser.laserPoseY, robotLaser.laserPoseTheta);
            geometry_msgs::PoseStamped robotPoseMsg = makePoseMsg(robotLaser.footer.loggerTimestamp,
                robotLaser.robotPoseX, robotLaser.robotPoseY, robotLaser.robotPoseTheta);
            laserPoseMsgs.push_back(laserPoseMsg);
            robotPoseMsgs.push_back(robotPoseMsg);
/*
            printf("%s %d "
                "%lf %lf %lf "
                "%lf %lf "
                "%d %d\n",
                data[0].c_str(), robotLaser.laserType,
                robotLaser.startAngle, robotLaser.fieldOfView, robotLaser.angularResolution,
                robotLaser.maximumRange, robotLaser.accuracy,
                robotLaser.remissionMode, robotLaser.numReadings);
            for (int i = 0; i < robotLaser.numReadings; ++i)
                printf("%lf ", robotLaser.rangeReadings[i]);
            printf("\n");
            for (int i = 0; i < robotLaser.numRemissions; ++i)
                printf("%lf ", robotLaser.remissionValues[i]);
            printf("\n");
            printf("%lf %lf %lf "
                "%lf %lf %lf "
                "%lf %lf "
                "%lf %lf\n",
                robotLaser.laserPoseX, robotLaser.laserPoseY, robotLaser.laserPoseTheta,
                robotLaser.robotPoseX, robotLaser.robotPoseY, robotLaser.robotPoseTheta,
                robotLaser.laserTv, robotLaser.laserRv,
                robotLaser.forwardSafetyDist, robotLaser.sideSaftyDist);
            printf("%lf %s %lf\n",
                robotLaser.footer.ipcTimestamp, robotLaser.footer.ipcHostname.c_str(), robotLaser.footer.loggerTimestamp);
 */
        }
    }

    fclose(fp);

    if (odomTVSum == 0.0 && odomRVSum == 0.0) {
        double time0 = odomMsgs[0].header.stamp.toSec();
        double yaw0;
        quat2rpy(odomMsgs[0].pose.pose.orientation.x, odomMsgs[0].pose.pose.orientation.y,
            odomMsgs[0].pose.pose.orientation.z, odomMsgs[0].pose.pose.orientation.w,
            NULL, NULL, &yaw0);
        for (int i = 1; i < (int)odomMsgs.size(); ++i) {
            double time = odomMsgs[i].header.stamp.toSec();
            double yaw;
            quat2rpy(odomMsgs[i].pose.pose.orientation.x, odomMsgs[i].pose.pose.orientation.y,
                odomMsgs[i].pose.pose.orientation.z, odomMsgs[i].pose.pose.orientation.w,
                NULL, NULL, &yaw);
            double dt = time -  time0;
            if (dt <= 0.01) {
                time0 = time;
                yaw0 = yaw;
                continue;
            }
            double dx = odomMsgs[i].pose.pose.position.x - odomMsgs[i - 1].pose.pose.position.x;
            double dy = odomMsgs[i].pose.pose.position.y - odomMsgs[i - 1].pose.pose.position.y;
            double dyaw = yaw - yaw0;
            while (dyaw < -M_PI)
                dyaw += 2.0 * M_PI;
            while (dyaw > M_PI)
                dyaw -= 2.0 * M_PI;
            odomMsgs[i].twist.twist.linear.x = sqrt(dx * dx + dy * dy) / dt;
            odomMsgs[i].twist.twist.angular.z = dyaw / dt;
            double xx = dx * cos(-yaw0) - dy * sin(-yaw0);
            if (xx < 0.0)
                odomMsgs[i].twist.twist.linear.x *= -1.0;
            time0 = time;
            yaw0 = yaw;
        }
    }

    if (offsetPose) {
        for (int i = 0; i < (int)odomMsgs.size(); ++i) {
            odomMsgs[i].pose.pose.position.x -= offsetPoseX;
            odomMsgs[i].pose.pose.position.y -= offsetPoseY;
/*
            double yaw;
            quat2rpy(odomMsgs[i].pose.pose.orientation.x, odomMsgs[i].pose.pose.orientation.y,
                odomMsgs[i].pose.pose.orientation.z, odomMsgs[i].pose.pose.orientation.w,
                NULL, NULL, &yaw);
            odomMsgs[i].pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw - offsetPoseTheta);
 */
        }
        for (int i = 0; i < (int)laserPoseMsgs.size(); ++i) {
            laserPoseMsgs[i].pose.position.x -= offsetPoseX;
            laserPoseMsgs[i].pose.position.y -= offsetPoseY;
/*
            double yaw;
            quat2rpy(laserPoseMsgs[i].pose.orientation.x, laserPoseMsgs[i].pose.orientation.y,
                laserPoseMsgs[i].pose.orientation.z, laserPoseMsgs[i].pose.orientation.w,
                NULL, NULL, &yaw);
            laserPoseMsgs[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw - offsetPoseTheta);
 */
        }
        for (int i = 0; i < (int)robotPoseMsgs.size(); ++i) {
            robotPoseMsgs[i].pose.position.x -= offsetPoseX;
            robotPoseMsgs[i].pose.position.y -= offsetPoseY;
/*
            double yaw;
            quat2rpy(robotPoseMsgs[i].pose.orientation.x, robotPoseMsgs[i].pose.orientation.y,
                robotPoseMsgs[i].pose.orientation.z, robotPoseMsgs[i].pose.orientation.w,
                NULL, NULL, &yaw);
            robotPoseMsgs[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw - offsetPoseTheta);
 */
        }
    }
}

double getTime(void) {
    struct timeval time{};
    gettimeofday(&time, nullptr);
    return ((double)(time.tv_sec * 1000.0) + (double)(time.tv_usec / 1000.0)) / 1000.0;
}

int main(int argc, char **argv) {
    if (argv[1] == NULL) {
        fprintf(stderr, "argv[1] must be CARMEN log file.\n");
        exit(1);
    }

    ros::init(argc, argv, "carmen_player_ros");
    ros::NodeHandle nh("~");

    nh.param("map_frame", mapFrame, mapFrame);
    nh.param("odom_frame", odomFrame, odomFrame);
    nh.param("base_link_frame", baseLinkFrame, baseLinkFrame);
    nh.param("flaser_frame", flaserFrame, flaserFrame);
    nh.param("rlaser_frame", rlaserFrame, rlaserFrame);
    nh.param("raw_laser1_frame", rawLaser1Frame, rawLaser1Frame);
    nh.param("raw_laser2_frame", rawLaser2Frame, rawLaser2Frame);
    nh.param("raw_laser3_frame", rawLaser3Frame, rawLaser3Frame);
    nh.param("raw_laser4_frame", rawLaser4Frame, rawLaser4Frame);
    nh.param("robot_laser0_frame", robotLaser0Frame, robotLaser0Frame);
    nh.param("robot_laser1_frame", robotLaser1Frame, robotLaser1Frame);
    nh.param("odom_name", odomName, odomName);
    nh.param("flaser_name", flaserName, flaserName);
    nh.param("rlaser_name", rlaserName, rlaserName);
    nh.param("raw_laser1_name", rawLaser1Name, rawLaser1Name);
    nh.param("raw_laser2_name", rawLaser2Name, rawLaser2Name);
    nh.param("raw_laser3_name", rawLaser3Name, rawLaser3Name);
    nh.param("raw_laser4_name", rawLaser4Name, rawLaser4Name);
    nh.param("robot_laser0_name", robotLaser0Name, robotLaser0Name);
    nh.param("robot_laser1_name", robotLaser1Name, robotLaser1Name);
    nh.param("laser_pose_name", laserPoseName, laserPoseName);
    nh.param("robot_pose_name", robotPoseName, robotPoseName);
    nh.param("broadcast_base_link_to_Laser_tf", broadcastBaseLink2LaserTF, broadcastBaseLink2LaserTF);
    nh.param("broadcast_odom_tf", broadcastOdomTF, broadcastOdomTF);
    nh.param("offset_pose", offsetPose, offsetPose);
    nh.param("skip_time", skipTime, skipTime);
    nh.param("finish_time", finishTime, finishTime);

    ros::Publisher flaserPub = nh.advertise<sensor_msgs::LaserScan>(flaserName, 1);
    ros::Publisher rlaserPub = nh.advertise<sensor_msgs::LaserScan>(rlaserName, 1);
    ros::Publisher rawLaser1Pub = nh.advertise<sensor_msgs::LaserScan>(rawLaser1Name, 1);
    ros::Publisher rawLaser2Pub = nh.advertise<sensor_msgs::LaserScan>(rawLaser2Name, 1);
    ros::Publisher rawLaser3Pub = nh.advertise<sensor_msgs::LaserScan>(rawLaser3Name, 1);
    ros::Publisher rawLaser4Pub = nh.advertise<sensor_msgs::LaserScan>(rawLaser4Name, 1);
    ros::Publisher robotLaser0Pub = nh.advertise<sensor_msgs::LaserScan>(robotLaser0Name, 1);
    ros::Publisher robotLaser1Pub = nh.advertise<sensor_msgs::LaserScan>(robotLaser1Name, 1);
    ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>(odomName, 1);
    ros::Publisher laserPosePub = nh.advertise<geometry_msgs::PoseStamped>(laserPoseName, 1);
    ros::Publisher robotPosePub = nh.advertise<geometry_msgs::PoseStamped>(robotPoseName, 1);

    parseLogFile(argv[1]);

    if (broadcastBaseLink2LaserTF) {
        std::string cmd;
        int retVal;
        cmd = "rosrun tf static_transform_publisher ";
        cmd += std::to_string(frontLaserOffset) + " " + "0 0 0 0 0 ";
        cmd += baseLinkFrame + " " + flaserFrame + " 100 ";
        cmd += "__name:=base_link_to_front_laser &";
        retVal = system(cmd.c_str());

        cmd = "rosrun tf static_transform_publisher ";
        cmd += std::to_string(rearLaserOffset) + " " + "0 0 0 0 0 ";
        cmd += baseLinkFrame + " " + rlaserFrame + " 100 ";
        cmd += "__name:=base_link_to_rear_laser &";
        retVal = system(cmd.c_str());

        cmd = "rosrun tf static_transform_publisher ";
        cmd += std::to_string(frontLaserOffset) + " " + "0 0 0 0 0 ";
        cmd += baseLinkFrame + " " + rawLaser1Frame + " 100 ";
        cmd += "__name:=base_link_to_raw_laser1 &";
        retVal = system(cmd.c_str());
    }

    int flaserIdx = 0, rlaserIdx = 0;
    int rawLaser1Idx = 0, rawLaser2Idx = 0, rawLaser3Idx = 0, rawLaser4Idx = 0;
    int robotLaser0Idx = 0, robotLaser1Idx = 0;
    int odomIdx = 0;
    int laserPoseIdx = 0, robotPoseIdx = 0;
    tf::TransformBroadcaster tfBroadcaster;
    ros::Rate loopRate(100.0);

    double startTime = getTime();
    while (ros::ok()) {
        ros::spinOnce();
        double elapsedTime = getTime() - startTime + skipTime;
        printf("elapsedTime = %.3lf [second]\n", elapsedTime);

        if (flaserIdx < (int)flaserMsgs.size()) {
            while (flaserMsgs[flaserIdx].header.stamp.toSec() < elapsedTime) {
                flaserMsgs[flaserIdx].header.stamp = ros::Time::now();
                flaserPub.publish(flaserMsgs[flaserIdx]);
                flaserIdx++;
                if (flaserIdx == (int)flaserMsgs.size())
                    break;
            }
        }
        if (rlaserIdx < (int)rlaserMsgs.size()) {
            while (rlaserMsgs[rlaserIdx].header.stamp.toSec() < elapsedTime) {
                rlaserMsgs[rlaserIdx].header.stamp = ros::Time::now();
                rlaserPub.publish(rlaserMsgs[rlaserIdx]);
                rlaserIdx++;
                if (rlaserIdx == (int)rlaserMsgs.size())
                    break;
            }
        }

        if (rawLaser1Idx < (int)rawLaser1Msgs.size()) {
            while (rawLaser1Msgs[rawLaser1Idx].header.stamp.toSec() < elapsedTime) {
                rawLaser1Msgs[rawLaser1Idx].header.stamp = ros::Time::now();
                rawLaser1Pub.publish(rawLaser1Msgs[rawLaser1Idx]);
                rawLaser1Idx++;
                if (rawLaser1Idx == (int)rawLaser1Msgs.size())
                    break;
            }
        }
        if (rawLaser2Idx < (int)rawLaser2Msgs.size()) {
            while (rawLaser2Msgs[rawLaser2Idx].header.stamp.toSec() < elapsedTime) {
                rawLaser2Msgs[rawLaser2Idx].header.stamp = ros::Time::now();
                rawLaser2Pub.publish(rawLaser2Msgs[rawLaser2Idx]);
                rawLaser2Idx++;
                if (rawLaser2Idx == (int)rawLaser2Msgs.size())
                    break;
            }
        }
        if (rawLaser3Idx < (int)rawLaser3Msgs.size()) {
            while (rawLaser3Msgs[rawLaser3Idx].header.stamp.toSec() < elapsedTime) {
                rawLaser3Msgs[rawLaser3Idx].header.stamp = ros::Time::now();
                rawLaser3Pub.publish(rawLaser3Msgs[rawLaser3Idx]);
                rawLaser3Idx++;
                if (rawLaser3Idx == (int)rawLaser3Msgs.size())
                    break;
            }
        }
        if (rawLaser4Idx < (int)rawLaser4Msgs.size()) {
            while (rawLaser4Msgs[rawLaser4Idx].header.stamp.toSec() < elapsedTime) {
                rawLaser4Msgs[rawLaser4Idx].header.stamp = ros::Time::now();
                rawLaser4Pub.publish(rawLaser4Msgs[rawLaser1Idx]);
                rawLaser4Idx++;
                if (rawLaser4Idx == (int)rawLaser4Msgs.size())
                    break;
            }
        }

        if (robotLaser0Idx < (int)robotLaser0Msgs.size()) {
            while (robotLaser0Msgs[robotLaser0Idx].header.stamp.toSec() < elapsedTime) {
                robotLaser0Msgs[robotLaser0Idx].header.stamp = ros::Time::now();
                robotLaser0Pub.publish(robotLaser0Msgs[robotLaser0Idx]);
                robotLaser0Idx++;
                if (robotLaser0Idx < (int)robotLaser0Msgs.size())
                    break;
            }
        }
        if (robotLaser1Idx < (int)robotLaser1Msgs.size()) {
            while (robotLaser1Msgs[robotLaser1Idx].header.stamp.toSec() < elapsedTime) {
                robotLaser1Msgs[robotLaser1Idx].header.stamp = ros::Time::now();
                robotLaser1Pub.publish(robotLaser1Msgs[robotLaser1Idx]);
                robotLaser1Idx++;
                if (robotLaser1Idx == (int)robotLaser1Msgs.size())
                    break;
            }
        }

        if (odomIdx < (int)odomMsgs.size()) {
            while (odomMsgs[odomIdx].header.stamp.toSec() < elapsedTime) {
                odomMsgs[odomIdx].header.stamp = ros::Time::now();
                odomPub.publish(odomMsgs[odomIdx]);
                if (broadcastOdomTF) {
                    geometry_msgs::TransformStamped odomTrans;
                    odomTrans.header.stamp = odomMsgs[odomIdx].header.stamp;
                    odomTrans.header.frame_id = odomMsgs[odomIdx].header.frame_id;
                    odomTrans.child_frame_id = odomMsgs[odomIdx].child_frame_id;
                    odomTrans.transform.translation.x = odomMsgs[odomIdx].pose.pose.position.x;
                    odomTrans.transform.translation.y = odomMsgs[odomIdx].pose.pose.position.y;
                    odomTrans.transform.translation.z = 0.0;
                    odomTrans.transform.rotation.x = odomMsgs[odomIdx].pose.pose.orientation.x;
                    odomTrans.transform.rotation.y = odomMsgs[odomIdx].pose.pose.orientation.y;
                    odomTrans.transform.rotation.z = odomMsgs[odomIdx].pose.pose.orientation.z;
                    odomTrans.transform.rotation.w = odomMsgs[odomIdx].pose.pose.orientation.w;
                    tfBroadcaster.sendTransform(odomTrans);
                }
                odomIdx++;
                if (odomIdx == (int)odomMsgs.size())
                    break;
            }
        }

        if (laserPoseIdx < (int)laserPoseMsgs.size()) {
            while (laserPoseMsgs[laserPoseIdx].header.stamp.toSec() < elapsedTime) {
                laserPoseMsgs[laserPoseIdx].header.stamp = ros::Time::now();
                laserPosePub.publish(laserPoseMsgs[laserPoseIdx]);
                laserPoseIdx++;
                if (laserPoseIdx == (int)laserPoseMsgs.size())
                    break;
            }
        }
        if (robotPoseIdx < (int)robotPoseMsgs.size()) {
            while (robotPoseMsgs[robotPoseIdx].header.stamp.toSec() < elapsedTime) {
                robotPoseMsgs[robotPoseIdx].header.stamp = ros::Time::now();
                robotPosePub.publish(robotPoseMsgs[robotPoseIdx]);
                robotPoseIdx++;
                if (robotPoseIdx == (int)robotPoseMsgs.size())
                    break;
            }
        }

        if (flaserIdx == (int)flaserMsgs.size() &&
            rlaserIdx == (int)rlaserMsgs.size() &&
            rawLaser1Idx == (int)rawLaser1Msgs.size() &&
            rawLaser2Idx == (int)rawLaser2Msgs.size() &&
            rawLaser3Idx == (int)rawLaser3Msgs.size() &&
            rawLaser4Idx == (int)rawLaser4Msgs.size() &&
            robotLaser0Idx == (int)robotLaser0Msgs.size() &&
            robotLaser1Idx == (int)robotLaser1Msgs.size() &&
            odomIdx == (int)odomMsgs.size() &&
            laserPoseIdx == (int)laserPoseMsgs.size() &&
            robotPoseIdx == (int)robotPoseMsgs.size()) {
            printf("All messages have been published.\n");
            break;            
        }

        if (finishTime > 0.0 && elapsedTime >= finishTime) {
            printf("Exceeded the finish time.\n");
            break;
        }

        loopRate.sleep();
    }

    if (broadcastBaseLink2LaserTF) {
        std::string cmd;
        int retVal;
        cmd = "rosnode kill /base_link_to_front_laser";
        retVal = system(cmd.c_str());
        cmd = "rosnode kill /base_link_to_rear_laser";
        retVal = system(cmd.c_str());
        cmd = "rosnode kill /base_link_to_raw_laser1";
        retVal = system(cmd.c_str());
    }

    return 0;
}