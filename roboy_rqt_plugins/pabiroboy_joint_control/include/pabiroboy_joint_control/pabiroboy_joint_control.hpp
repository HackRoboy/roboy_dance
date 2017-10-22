#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <pabiroboy_joint_control/ui_pabiroboy_joint_control.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/DarkRoomSensor.h>
#include <roboy_communication_middleware/InverseKinematics.h>
#include <roboy_communication_middleware/MusicBeat.h>
#include <geometry_msgs/Point.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QSlider>
#include <QLineEdit>
#include <QPushButton>
#include <map>

#endif

#define NUMBER_OF_JOINTS 4
#define NUMBER_OF_POINTS 9
#define NUMBER_OF_DOF 2
//#define TOPIC_DATA_NR 5

using namespace std;

class PaBiRoboyJointControl : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    PaBiRoboyJointControl();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);

public Q_SLOTS:
    void plotData();
    void activate();
    void activatePosCtrl();
    void activateTopicCtrl();
    void updateSetPointsJointControl();
    void updateSetPointsJointControlAll();
    void JointController();
    void PosController();
    void TopicController();
private:
    void JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg);
    void MusicBeat(const roboy_communication_middleware::MusicBeat::ConstPtr &msg);
    void DarkRoomSensor(const roboy_communication_middleware::DarkRoomSensor::ConstPtr &msg);
Q_SIGNALS:
    void newData();
private:
    Ui::PaBiRoboyJointControl ui;
    QWidget *widget_;

    QVector<double> time;
    int counter = 0;
    QVector<double> jointData[NUMBER_OF_JOINTS], jointDataSetpoint[NUMBER_OF_JOINTS],pointDataSetpoint[NUMBER_OF_DOF],jointTarget[NUMBER_OF_JOINTS];
    double jointTargets[NUMBER_OF_JOINTS];

    roboy_communication_middleware::MusicBeat topicDataSetpoint;

    geometry_msgs::Point pointData[NUMBER_OF_POINTS];
    geometry_msgs::Point leftAnkle,rightAnkle,beerBelly;
    int samples_per_plot = 300;
    QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher motorCommand;
    ros::Subscriber jointStatus;
    ros::Subscriber poseStatus;
    ros::Subscriber topicPose;
    ros::ServiceClient client;

    map<string,QLineEdit*> text;
    map<string,QPushButton*> button;
    map<string,QSlider*> slider;
    //QLineEdit lineX,lineY,lineZ;
    map<int,QLineEdit*> flexor, extensor;
    bool joint_control_active = false;
    bool position_control_active = false;
    bool topic_control_active = false;
    int sensorDataCount = 0,sensorDataCount2 = 0,sensorDataCount3 = 0;
    bool firstRead = true;
    double x,y,z,xl,yl,zl,xr,yr,zr;
};
