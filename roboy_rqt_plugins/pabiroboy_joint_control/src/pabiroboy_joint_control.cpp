#include <pabiroboy_joint_control/pabiroboy_joint_control.hpp>

PaBiRoboyJointControl::PaBiRoboyJointControl()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("PaBiRoboyJointControl");
}

void PaBiRoboyJointControl::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));

    text["joint_setpoint_0"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_0");
    text["joint_setpoint_1"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_1");
    text["joint_setpoint_2"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_2");
    text["joint_setpoint_3"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_3");
    text["joint_setpoint_all"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_all");

    text["point_setpoint_0"] = widget_->findChild<QLineEdit *>("point_setpoint_text_0");
    text["point_setpoint_1"] = widget_->findChild<QLineEdit *>("point_setpoint_text_1");

    text["Kp"] = widget_->findChild<QLineEdit *>("Kp");
    text["Ki"] = widget_->findChild<QLineEdit *>("Ki");
    text["Kd"] = widget_->findChild<QLineEdit *>("Kd");

    text["point_setpoint_text_x"] = widget_->findChild<QLineEdit *>("point_setpoint_text_x");
    text["point_setpoint_text_y"] = widget_->findChild<QLineEdit *>("point_setpoint_text_y");
    text["point_setpoint_text_z"] = widget_->findChild<QLineEdit *>("point_setpoint_text_z");


    flexor[0] = widget_->findChild<QLineEdit *>("flexor_0");
    flexor[1] = widget_->findChild<QLineEdit *>("flexor_1");
    flexor[2] = widget_->findChild<QLineEdit *>("flexor_2");
    flexor[3] = widget_->findChild<QLineEdit *>("flexor_3");
    extensor[0] = widget_->findChild<QLineEdit *>("extensor_0");
    extensor[1] = widget_->findChild<QLineEdit *>("extensor_1");
    extensor[2] = widget_->findChild<QLineEdit *>("extensor_2");
    extensor[3] = widget_->findChild<QLineEdit *>("extensor_3");

    slider["joint_setpoint_0"] = widget_->findChild<QSlider *>("joint_setpoint_0");
    slider["joint_setpoint_1"] = widget_->findChild<QSlider *>("joint_setpoint_1");
    slider["joint_setpoint_2"] = widget_->findChild<QSlider *>("joint_setpoint_2");
    slider["joint_setpoint_3"] = widget_->findChild<QSlider *>("joint_setpoint_3");
    slider["joint_setpoint_all"] = widget_->findChild<QSlider *>("joint_setpoint_all");
    slider["point_setpoint_0"] = widget_->findChild<QSlider *>("point_setpoint_0");
    slider["point_setpoint_1"] = widget_->findChild<QSlider *>("point_setpoint_1");

    button["activate"] = widget_->findChild<QPushButton *>("activate");
    button["activate"]->setStyleSheet("background-color: green");
    button["activatePosCtrl"] = widget_->findChild<QPushButton *>("activatePosCtrl");
    button["activatePosCtrl"]->setStyleSheet("background-color: green");


    button["activateTopicCtrl"] = widget_->findChild<QPushButton *>("activateTopicCtrl");
    button["activateTopicCtrl"]->setStyleSheet("background-color: green");

    // add graphs to plot for setpoint an current angle
    ui.joint0->addGraph();
    ui.joint0->graph(0)->setPen(QPen(color_pallette[0]));
    ui.joint0->addGraph();
    ui.joint0->graph(1)->setPen(QPen(color_pallette[1]));
    ui.joint0->addGraph();
    ui.joint0->graph(2)->setPen(QPen(color_pallette[2]));
    ui.joint0->yAxis->setLabel("degrees");
    ui.joint0->yAxis->setRange(-360, 360);

    ui.joint1->addGraph();
    ui.joint1->graph(0)->setPen(QPen(color_pallette[0]));
    ui.joint1->addGraph();
    ui.joint1->graph(1)->setPen(QPen(color_pallette[1]));
    ui.joint1->addGraph();
    ui.joint1->graph(2)->setPen(QPen(color_pallette[2]));
    ui.joint1->yAxis->setLabel("degrees");
    ui.joint1->yAxis->setRange(-360, 360);

    ui.joint2->addGraph();
    ui.joint2->graph(0)->setPen(QPen(color_pallette[0]));
    ui.joint2->addGraph();
    ui.joint2->graph(1)->setPen(QPen(color_pallette[1]));
    ui.joint2->addGraph();
    ui.joint2->graph(2)->setPen(QPen(color_pallette[2]));
    ui.joint2->yAxis->setLabel("degrees");
    ui.joint2->yAxis->setRange(-360, 360);

    ui.joint3->addGraph();
    ui.joint3->graph(0)->setPen(QPen(color_pallette[0]));
    ui.joint3->addGraph();
    ui.joint3->graph(1)->setPen(QPen(color_pallette[1]));
    ui.joint3->addGraph();
    ui.joint3->graph(2)->setPen(QPen(color_pallette[2]));
    ui.joint3->yAxis->setLabel("degrees");
    ui.joint3->yAxis->setRange(-360, 360);

    // connect slider to callback
    QObject::connect(slider["joint_setpoint_0"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["joint_setpoint_1"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["joint_setpoint_2"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["joint_setpoint_3"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["joint_setpoint_all"], SIGNAL(valueChanged(int)), this,
                     SLOT(updateSetPointsJointControlAll()));

    QObject::connect(slider["point_setpoint_0"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["point_setpoint_1"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));

    QObject::connect(ui.activate, SIGNAL(clicked()), this, SLOT(activate()));

    QObject::connect(ui.activatePosCtrl, SIGNAL(clicked()), this, SLOT(activatePosCtrl()));


    QObject::connect(ui.activateTopicCtrl, SIGNAL(clicked()), this, SLOT(activateTopicCtrl()));

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "pabiroboy_joint_control_rqt_plugin");
    }

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    motorCommand = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);

    jointStatus = nh->subscribe("/roboy/middleware/JointStatus", 1, &PaBiRoboyJointControl::JointStatus, this);

    poseStatus = nh->subscribe("/roboy/middleware/DarkRoom/sensor_location", 1, &PaBiRoboyJointControl::DarkRoomSensor, this);

    client = nh->serviceClient<roboy_communication_middleware::InverseKinematics>("/roboy/middleware/PaBiRoboy/inverseKinematics");

    topicPose = nh->subscribe("/roboy/middleware/MusicBeat", 1, &PaBiRoboyJointControl::MusicBeat, this);

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
}

void PaBiRoboyJointControl::shutdownPlugin() {
    motorCommand.shutdown();
}

void PaBiRoboyJointControl::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                         qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void PaBiRoboyJointControl::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                            const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void PaBiRoboyJointControl::DarkRoomSensor(const roboy_communication_middleware::DarkRoomSensor::ConstPtr &msg) {

    //ROS_INFO("darkroom %d",msg->ids[0]);


    for (uint point = 0; point < msg->ids.size(); point++) {
        //pointData[point].x=msg->position[point].x;
        //pointData[point].y=msg->position[point].y;
        //pointData[point].z=msg->position[point].z;
        switch (msg->ids[point]){
            case 14:{
                sensorDataCount++;
                if(sensorDataCount>1) {
                    pointData[0].x = pointData[0].x + msg->position[point].x;
                    pointData[0].y = pointData[0].y + msg->position[point].y;
                    pointData[0].z = pointData[0].z + msg->position[point].z;
                }
                else{
                    pointData[0].x=msg->position[point].x;
                    pointData[0].y=msg->position[point].y;
                    pointData[0].z=msg->position[point].z;
                }
                break;
            }
            case 15:{
                pointData[1].x=msg->position[point].x;
                pointData[1].y=msg->position[point].y;
                pointData[1].z=msg->position[point].z;
                break;
            }
            case 16:{
                pointData[2].x=msg->position[point].x;
                pointData[2].y=msg->position[point].y;
                pointData[2].z=msg->position[point].z;
                break;
            }
            case 17:{
                pointData[3].x=msg->position[point].x;
                pointData[3].y=msg->position[point].y;
                pointData[3].z=msg->position[point].z;
                break;
            }
            case 18:{
                pointData[4].x=msg->position[point].x;
                pointData[4].y=msg->position[point].y;
                pointData[4].z=msg->position[point].z;
                break;
            }
            case 19:{
                sensorDataCount3++;
                if(sensorDataCount3>1) {
                    pointData[5].x = pointData[5].x + msg->position[point].x;
                    pointData[5].y = pointData[5].y + msg->position[point].y;
                    pointData[5].z = pointData[5].z + msg->position[point].z;
                }
                else{
                    pointData[5].x=msg->position[point].x;
                    pointData[5].y=msg->position[point].y;
                    pointData[5].z=msg->position[point].z;
                }
                break;
            }
            case 20:{
                sensorDataCount2++;
                if(sensorDataCount2>1) {
                    pointData[6].x = pointData[6].x + msg->position[point].x;
                    pointData[6].y = pointData[6].y + msg->position[point].y;
                    pointData[6].z = pointData[6].z + msg->position[point].z;
                }
                else{
                    pointData[6].x=msg->position[point].x;
                    pointData[6].y=msg->position[point].y;
                    pointData[6].z=msg->position[point].z;
                }
                break;
            }
            case 21:{
                pointData[7].x=msg->position[point].x;
                pointData[7].y=msg->position[point].y;
                pointData[7].z=msg->position[point].z;
                break;
            }

            case 22:{
                pointData[8].x=msg->position[point].x;
                pointData[8].y=msg->position[point].y;
                pointData[8].z=msg->position[point].z;
                break;
            }
        }
        if(sensorDataCount==10) {
            text["point_setpoint_text_x"]->setText(QString::number((int) (100*pointData[0].x/10)));
            text["point_setpoint_text_y"]->setText(QString::number((int) (100*pointData[0].y/10)));
            text["point_setpoint_text_z"]->setText(QString::number((int) (100*pointData[0].z/10)));
            sensorDataCount=0;
            beerBelly.x=pointData[0].x/10;
            beerBelly.y=pointData[0].y/10;
            beerBelly.z=pointData[0].z/10;
            //pointData[0].x=pointData[0].y=pointData[0].z=0;
            /*ui->point_setpoint_text_x->setText(QString::number(msg->position[point].x));
            ui->point_setpoint_text_y->setText(QString::number(msg->position[point].y));
            ui->point_setpoint_text_z->setText(QString::number(msg->position[point].z));*/
        }
        if(sensorDataCount2==10) {

            sensorDataCount2 = 0;
            leftAnkle.x = pointData[6].x / 10;
            leftAnkle.y = pointData[6].y / 10;
            leftAnkle.z = pointData[6].z / 10;
        }
        if(sensorDataCount3==10) {

            sensorDataCount3 = 0;
            rightAnkle.x = pointData[5].x / 10;
            rightAnkle.y = pointData[5].y / 10;
            rightAnkle.z = pointData[5].z / 10;
        }
    }

}

void PaBiRoboyJointControl::JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
    ROS_INFO_THROTTLE(5, "receiving joint status");
    time.push_back(counter++);

    jointDataSetpoint[0].push_back(slider["joint_setpoint_0"]->value());
    jointDataSetpoint[1].push_back(slider["joint_setpoint_1"]->value());
    jointDataSetpoint[2].push_back(slider["joint_setpoint_2"]->value());
    jointDataSetpoint[3].push_back(slider["joint_setpoint_3"]->value());

    pointDataSetpoint[0].push_back(slider["point_setpoint_0"]->value());
    pointDataSetpoint[1].push_back(slider["point_setpoint_1"]->value());

    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        jointTarget[joint].push_back(jointTargets[joint]);
        if (jointTarget[joint].size() > samples_per_plot) {
            jointTarget[joint].pop_front();
        }
    }

    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        jointData[joint].push_back(msg->relAngles[joint]/4096.0*360-180.0);
        if (jointData[joint].size() > samples_per_plot) {
            jointData[joint].pop_front();
        }
        if (jointDataSetpoint[joint].size() > samples_per_plot) {
            jointDataSetpoint[joint].pop_front();
        }
    }
    if (time.size() > samples_per_plot)
        time.pop_front();

    if (counter % 3 == 0)
            Q_EMIT newData();

    if(joint_control_active)
        JointController();
    if(position_control_active)
        PosController();
    if(topic_control_active)
        TopicController();
}

void PaBiRoboyJointControl::MusicBeat(const roboy_communication_middleware::MusicBeat::ConstPtr &msg) {

    topicDataSetpoint.x=msg->x;
    topicDataSetpoint.z=msg->z;
}

void PaBiRoboyJointControl::plotData() {
    ui.joint0->graph(0)->setData(time, jointData[0]);
    ui.joint1->graph(0)->setData(time, jointData[1]);
    ui.joint2->graph(0)->setData(time, jointData[2]);
    ui.joint3->graph(0)->setData(time, jointData[3]);

    ui.joint0->graph(1)->setData(time, jointDataSetpoint[0]);
    ui.joint1->graph(1)->setData(time, jointDataSetpoint[1]);
    ui.joint2->graph(1)->setData(time, jointDataSetpoint[2]);
    ui.joint3->graph(1)->setData(time, jointDataSetpoint[3]);

    ui.joint0->graph(2)->setData(time, jointTarget[0]);
    ui.joint1->graph(2)->setData(time, jointTarget[1]);
    ui.joint2->graph(2)->setData(time, jointTarget[2]);
    ui.joint3->graph(2)->setData(time, jointTarget[3]);

    ui.joint0->xAxis->rescale();
    ui.joint1->xAxis->rescale();
    ui.joint2->xAxis->rescale();
    ui.joint3->xAxis->rescale();

    ui.joint0->replot();
    ui.joint1->replot();
    ui.joint2->replot();
    ui.joint3->replot();
}

void PaBiRoboyJointControl::activate() {
    if (button["activate"]->isChecked()) {
        button["activate"]->setStyleSheet("background-color: red");
        joint_control_active = true;
    } else {
        button["activate"]->setStyleSheet("background-color: green");
        joint_control_active = false;
    }
}
void PaBiRoboyJointControl::activatePosCtrl() {
    if (button["activatePosCtrl"]->isChecked()) {
        button["activatePosCtrl"]->setStyleSheet("background-color: red");
        position_control_active = true;
    } else {
        button["activatePosCtrl"]->setStyleSheet("background-color: green");
        position_control_active = false;
    }
}

void PaBiRoboyJointControl::activateTopicCtrl() {
    if (button["activateTopicCtrl"]->isChecked()) {
        button["activateTopicCtrl"]->setStyleSheet("background-color: red");
        topic_control_active = true;
    } else {
        button["activateTopicCtrl"]->setStyleSheet("background-color: green");
        topic_control_active = false;
    }
}

void PaBiRoboyJointControl::updateSetPointsJointControl() {
    text["joint_setpoint_0"]->setText(QString::number(slider["joint_setpoint_0"]->value()));
    text["joint_setpoint_1"]->setText(QString::number(slider["joint_setpoint_1"]->value()));
    text["joint_setpoint_2"]->setText(QString::number(slider["joint_setpoint_2"]->value()));
    text["joint_setpoint_3"]->setText(QString::number(slider["joint_setpoint_3"]->value()));
    text["point_setpoint_0"]->setText(QString::number(slider["point_setpoint_0"]->value()));
    text["point_setpoint_1"]->setText(QString::number(slider["point_setpoint_1"]->value()));
}

void PaBiRoboyJointControl::updateSetPointsJointControlAll() {
    text["joint_setpoint_0"]->setText(QString::number(slider["joint_setpoint_all"]->value()));
    text["joint_setpoint_1"]->setText(QString::number(slider["joint_setpoint_all"]->value()));
    text["joint_setpoint_2"]->setText(QString::number(slider["joint_setpoint_all"]->value()));
    text["joint_setpoint_3"]->setText(QString::number(slider["joint_setpoint_all"]->value()));
    text["joint_setpoint_all"]->setText(QString::number(slider["joint_setpoint_all"]->value()));

    slider["joint_setpoint_0"]->setValue(slider["joint_setpoint_all"]->value());
    slider["joint_setpoint_1"]->setValue(slider["joint_setpoint_all"]->value());
    slider["joint_setpoint_2"]->setValue(slider["joint_setpoint_all"]->value());
    slider["joint_setpoint_3"]->setValue(slider["joint_setpoint_all"]->value());
}

void PaBiRoboyJointControl::JointController() {
    ROS_INFO_THROTTLE(5, "joint control active");
    float error[NUMBER_OF_JOINTS];
    float integral[NUMBER_OF_JOINTS];
    float integral_max = 360;
    const float smooth_distance = 50;
    const float offset = 20;
    static float error_previous[NUMBER_OF_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool ok;
    roboy_communication_middleware::MotorCommand msg;
    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        error[joint] = jointDataSetpoint[joint].back() - jointData[joint].back();

        bool flexor_id_valid_ok, extensor_id_valid_ok;
        int flexor_muscle = flexor[joint]->text().toInt(&flexor_id_valid_ok);
        int extensor_muscle = extensor[joint]->text().toInt(&extensor_id_valid_ok);

        msg.motors.push_back(flexor_muscle);
        msg.motors.push_back(extensor_muscle);

        float pterm = text["Kp"]->text().toFloat(&ok) * error[joint];
        float dterm = text["Kd"]->text().toFloat(&ok) *
                      (error[joint] - error_previous[joint]);
        integral[joint] += text["Ki"]->text().toFloat(&ok) * error[joint];
        if (integral[joint] >= integral_max) {
            integral[joint] = integral_max;
        } else if (integral[joint] <= integral_max) {
            integral[joint] = -integral_max;
        }
        float result = pterm + dterm + integral[joint];
        if (result <= -smooth_distance) {
            msg.setPoints.push_back(offset - result);
            msg.setPoints.push_back(offset);
        } else if (result < smooth_distance) {
            msg.setPoints.push_back(offset + powf(result - smooth_distance, 2.0f) /
                                             (4.0f * smooth_distance));
            msg.setPoints.push_back(offset + powf(result + smooth_distance, 2.0f) /
                                             (4.0f * smooth_distance));
        } else {
            msg.setPoints.push_back(offset);
            msg.setPoints.push_back(offset + result);
        }
        error_previous[joint] = error[joint];
    }
    motorCommand.publish(msg);
}

void PaBiRoboyJointControl::PosController() {

    bool ok;
    //double x=1.530,y=2.089,z=-1.434;
    //double xl=1.203,yl=2.027,zl=-1.821;
    //double xr=1.944,yr=2.017,zr=-1.815;


    if(firstRead){
        firstRead=false;
        /*
        x=(text["point_setpoint_text_x"]->text().toFloat(&ok))/100.0;
        y=(text["point_setpoint_text_y"]->text().toFloat(&ok))/100.0;
        z=(text["point_setpoint_text_z"]->text().toFloat(&ok))/100.0;
         */
        x=beerBelly.x;
        y=beerBelly.y;
        z=beerBelly.z;

        xl=leftAnkle.x;
        yl=leftAnkle.y;
        zl=leftAnkle.z;

        xr=rightAnkle.x;
        yr=rightAnkle.y;
        zr=rightAnkle.z;

    }



    roboy_communication_middleware::InverseKinematics srv;

    /*
    geometry_msgs/Vector3 targetPosition
    geometry_msgs/Vector3 ankle_left
    geometry_msgs/Vector3 ankle_right_sensor
    int32 lighthouse_sensor_id
    float64[] initial_angles
    bool inspect
    bool visualize_initial
    bool visualize_result
     */

    srv.request.targetPosition.x=x+pointDataSetpoint[0].back()/100.0;
    srv.request.targetPosition.y=z+pointDataSetpoint[1].back()/100.0;
    srv.request.targetPosition.z=0;

    srv.request.ankle_left.x=xl;
    srv.request.ankle_left.y=zl;
    srv.request.ankle_left.z=yl;

    srv.request.ankle_right_sensor.x=xr;
    srv.request.ankle_right_sensor.y=zr;
    srv.request.ankle_right_sensor.z=yr;

    srv.request.lighthouse_sensor_id=4;
    srv.request.initial_angles.resize(5);
    srv.request.initial_angles[0]=jointData[0].back();
    srv.request.initial_angles[1]=jointData[1].back();
    srv.request.initial_angles[2]=jointData[2].back();
    srv.request.initial_angles[3]=jointData[3].back();
    srv.request.initial_angles[4]=0.0;

    srv.request.inspect=true;
    srv.request.visualize_initial=true;
    srv.request.visualize_result=true;

    if (client.call(srv)){
        ROS_INFO("Service call");
    }
    else {
        ROS_ERROR("Failed to call service");
        return;
    }
    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        jointTargets[joint]=srv.response.angles[joint];
        ROS_INFO("joint %d %f",joint, srv.response.angles[joint] );
    }




    //ROS_INFO_THROTTLE(5, "joint control active");
    float error[NUMBER_OF_JOINTS];
    float integral[NUMBER_OF_JOINTS];
    float integral_max = 360;
    const float smooth_distance = 50;
    const float offset = 20;
    static float error_previous[NUMBER_OF_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f};
    //bool ok;
    roboy_communication_middleware::MotorCommand msg;
    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        error[joint] = srv.response.angles[joint] - jointData[joint].back();

        bool flexor_id_valid_ok, extensor_id_valid_ok;
        int flexor_muscle = flexor[joint]->text().toInt(&flexor_id_valid_ok);
        int extensor_muscle = extensor[joint]->text().toInt(&extensor_id_valid_ok);

        msg.motors.push_back(flexor_muscle);
        msg.motors.push_back(extensor_muscle);

        float pterm = text["Kp"]->text().toFloat(&ok) * error[joint];
        float dterm = text["Kd"]->text().toFloat(&ok) *
                      (error[joint] - error_previous[joint]);
        integral[joint] += text["Ki"]->text().toFloat(&ok) * error[joint];
        if (integral[joint] >= integral_max) {
            integral[joint] = integral_max;
        } else if (integral[joint] <= integral_max) {
            integral[joint] = -integral_max;
        }
        float result = pterm + dterm + integral[joint];
        if (result <= -smooth_distance) {
            msg.setPoints.push_back(offset - result);
            msg.setPoints.push_back(offset);
        } else if (result < smooth_distance) {
            msg.setPoints.push_back(offset + powf(result - smooth_distance, 2.0f) /
                                             (4.0f * smooth_distance));
            msg.setPoints.push_back(offset + powf(result + smooth_distance, 2.0f) /
                                             (4.0f * smooth_distance));
        } else {
            msg.setPoints.push_back(offset);
            msg.setPoints.push_back(offset + result);
        }
        error_previous[joint] = error[joint];
    }
    motorCommand.publish(msg);
}

void PaBiRoboyJointControl::TopicController() {

    bool ok;
    //double x=1.530,y=2.089,z=-1.434;
    //double xl=1.203,yl=2.027,zl=-1.821;
    //double xr=1.944,yr=2.017,zr=-1.815;


    if(firstRead){
        firstRead=false;
        /*
        x=(text["point_setpoint_text_x"]->text().toFloat(&ok))/100.0;
        y=(text["point_setpoint_text_y"]->text().toFloat(&ok))/100.0;
        z=(text["point_setpoint_text_z"]->text().toFloat(&ok))/100.0;
         */
        x=beerBelly.x;
        y=beerBelly.y;
        z=beerBelly.z;

        xl=leftAnkle.x;
        yl=leftAnkle.y;
        zl=leftAnkle.z;

        xr=rightAnkle.x;
        yr=rightAnkle.y;
        zr=rightAnkle.z;

    }



    roboy_communication_middleware::InverseKinematics srv;

    /*
    geometry_msgs/Vector3 targetPosition
    geometry_msgs/Vector3 ankle_left
    geometry_msgs/Vector3 ankle_right_sensor
    int32 lighthouse_sensor_id
    float64[] initial_angles
    bool inspect
    bool visualize_initial
    bool visualize_result
     */

    srv.request.targetPosition.x=x+topicDataSetpoint.x;
    srv.request.targetPosition.y=z+topicDataSetpoint.z;
    srv.request.targetPosition.z=0;

    srv.request.ankle_left.x=xl;
    srv.request.ankle_left.y=zl;
    srv.request.ankle_left.z=yl;

    srv.request.ankle_right_sensor.x=xr;
    srv.request.ankle_right_sensor.y=zr;
    srv.request.ankle_right_sensor.z=yr;

    srv.request.lighthouse_sensor_id=4;
    srv.request.initial_angles.resize(5);
    srv.request.initial_angles[0]=jointData[0].back();
    srv.request.initial_angles[1]=jointData[1].back();
    srv.request.initial_angles[2]=jointData[2].back();
    srv.request.initial_angles[3]=jointData[3].back();
    srv.request.initial_angles[4]=0.0;

    srv.request.inspect=true;
    srv.request.visualize_initial=true;
    srv.request.visualize_result=true;

    if (client.call(srv)){
        ROS_INFO("Service call");
    }
    else {
        ROS_ERROR("Failed to call service");
        return;
    }
    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        jointTargets[joint]=srv.response.angles[joint];
        ROS_INFO("joint %d %f",joint, srv.response.angles[joint] );
    }




    //ROS_INFO_THROTTLE(5, "joint control active");
    float error[NUMBER_OF_JOINTS];
    float integral[NUMBER_OF_JOINTS];
    float integral_max = 360;
    const float smooth_distance = 50;
    const float offset = 20;
    static float error_previous[NUMBER_OF_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f};
    //bool ok;
    roboy_communication_middleware::MotorCommand msg;
    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        error[joint] = srv.response.angles[joint] - jointData[joint].back();

        bool flexor_id_valid_ok, extensor_id_valid_ok;
        int flexor_muscle = flexor[joint]->text().toInt(&flexor_id_valid_ok);
        int extensor_muscle = extensor[joint]->text().toInt(&extensor_id_valid_ok);

        msg.motors.push_back(flexor_muscle);
        msg.motors.push_back(extensor_muscle);

        float pterm = text["Kp"]->text().toFloat(&ok) * error[joint];
        float dterm = text["Kd"]->text().toFloat(&ok) *
                      (error[joint] - error_previous[joint]);
        integral[joint] += text["Ki"]->text().toFloat(&ok) * error[joint];
        if (integral[joint] >= integral_max) {
            integral[joint] = integral_max;
        } else if (integral[joint] <= integral_max) {
            integral[joint] = -integral_max;
        }
        float result = pterm + dterm + integral[joint];
        if (result <= -smooth_distance) {
            msg.setPoints.push_back(offset - result);
            msg.setPoints.push_back(offset);
        } else if (result < smooth_distance) {
            msg.setPoints.push_back(offset + powf(result - smooth_distance, 2.0f) /
                                             (4.0f * smooth_distance));
            msg.setPoints.push_back(offset + powf(result + smooth_distance, 2.0f) /
                                             (4.0f * smooth_distance));
        } else {
            msg.setPoints.push_back(offset);
            msg.setPoints.push_back(offset + result);
        }
        error_previous[joint] = error[joint];
    }
    motorCommand.publish(msg);
}

PLUGINLIB_DECLARE_CLASS(pabiroboy_joint_control, PaBiRoboyJointControl, PaBiRoboyJointControl, rqt_gui_cpp::Plugin)
