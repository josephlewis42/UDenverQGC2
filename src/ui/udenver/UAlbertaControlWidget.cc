/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *
 *
 *   @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 *   @date February 5, 2012: Class Creation
 *
 *   @author Joseph Lewis <joseph@josephlewis.net>
 *   @date 2014-08-05: Update to work with qgc 2
 *
 */

#include "UAlbertaControlWidget.h"

#include <QString>
#include <QMessageBox>
#include <QStringList>

#include <UASManager.h>
#include <UAlbertaMAV.h>

UAlbertaControlWidget::UAlbertaControlWidget(QWidget *parent) :
	QWidget(parent),
    uas(0)
{
	ui.setupUi(this);

    setActiveUAS(UASManager::instance()->getActiveUAS());
    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setActiveUAS(UASInterface*)));

	connect(ui.servo_source_button, SIGNAL(clicked()), this, SLOT(setServoSource()));
	connect(ui.control_mode_button, SIGNAL(clicked()), this, SLOT(setControlMode()));
	connect(ui.ned_origin_button, SIGNAL(clicked()), this, SLOT(setOrigin()));
	connect(ui.shutdown_button, SIGNAL(clicked()), this, SLOT(shutdown()));
	connect(ui.reset_filter_button, SIGNAL(clicked()), this, SLOT(resetFilter()));
	connect(ui.set_attitude_button, SIGNAL(clicked()), this, SLOT(initAttitude()));
	connect(ui.attitude_source_button, SIGNAL(clicked()), this, SLOT(setAttitudeSource()));
	connect(ui.setRefPos, SIGNAL(clicked()), this, SLOT(setReferencePosition()));
    connect(ui.new_log_point_button, SIGNAL(clicked()), this, SLOT(startNewLog()));


	// note string appear in same order as mavlink enum (necessary for indexing)
	QStringList servo_sources;
	servo_sources << "Direct Manual" << "Scaled Manual" << "Automatic Control";
	ui.servo_source_box->addItems(servo_sources);

	QStringList controller_modes;
	controller_modes << "Attitude PID" << "Translation PID" << "Translation SBF";
	ui.control_mode_box->addItems(controller_modes);


	QStringList attitude_sources;
	attitude_sources << "Nav Filter" << "AHRS";
	ui.attitude_source_box->addItems(attitude_sources);

}

UAlbertaControlWidget::~UAlbertaControlWidget()
{

}

void UAlbertaControlWidget::setServoSource()
{

	UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(this->uas));
	if (mav)
	{
		qDebug() << "Set servo source to " << ui.servo_source_box->currentText() << " with index: " << ui.servo_source_box->currentIndex();
		mavlink_message_t msg;
		mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_SET_SERVO_SOURCE, ui.servo_source_box->currentIndex());
		mav->sendMessage(msg);
	}
}

void UAlbertaControlWidget::setControlMode()
{

	UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(uas));
	if (mav)
	{
		qDebug() << "Set Control Mode to " << ui.control_mode_box->currentText() << " with index: " << ui.control_mode_box->currentIndex();
		mavlink_message_t msg;
		mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_SET_CONTROL_MODE, ui.control_mode_box->currentIndex());
		mav->sendMessage(msg);
	}
}

void UAlbertaControlWidget::setOrigin()
{

	QMessageBox msgbox(QMessageBox::Warning,
			tr("Confirm Set Origin"),
			tr("Are you sure you want to set a new origin?"),
			QMessageBox::Yes|QMessageBox::No);

	if (msgbox.exec() == QMessageBox::Yes)
	{
		UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(uas));
		if (mav)
		{
			qDebug() << "Set Origin";
			mavlink_message_t msg;
			mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_SET_ORIGIN, 0);
			mav->sendMessage(msg);
		}

	}
}

void UAlbertaControlWidget::resetFilter()
{
	QMessageBox msgbox(QMessageBox::Warning,
			tr("Confirm Filter Reset"),
			tr("Are you sure you want to reset the navigation filter?"),
			QMessageBox::Yes|QMessageBox::No);

	if (msgbox.exec() == QMessageBox::Yes)
	{
		UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(uas));
		if (mav)
		{
			qDebug() << "Reset Filter";
			mavlink_message_t msg;
			mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_RESET_FILTER, 0);
			mav->sendMessage(msg);
		}
	}
}

void UAlbertaControlWidget::setAttitudeSource()
{
	qDebug() << "Set Attitude Source";
	UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(uas));
	if (mav)
	{
		mavlink_message_t msg;
		mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_SET_ATTITUDE_SOURCE, ui.attitude_source_box->currentIndex());
		mav->sendMessage(msg);
	}
}

void UAlbertaControlWidget::setReferencePosition()
{
	qDebug() << "Set reference position";
	UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(uas));
	if (mav)
	{
		mavlink_message_t msg;
		mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_SET_REF_POS, 0);
		mav->sendMessage(msg);
	}
}

void UAlbertaControlWidget::startNewLog()
{
    qDebug() << "Start new log files";
    UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(uas));
    if (mav)
    {
        mavlink_message_t msg;
        mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_NEW_LOG_POINT, 0);
        mav->sendMessage(msg);
    }
}

void UAlbertaControlWidget::shutdown()
{
	QMessageBox msgbox(QMessageBox::Critical,
			tr("Confirm Shutdown"),
			tr("Are you sure you want to kill the autopilot?"),
			QMessageBox::Yes|QMessageBox::No);

	if (msgbox.exec() == QMessageBox::Yes)
	{
		qDebug() << "Shutdown Autopilot";
		UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(uas));
		if (mav)
		{
			mavlink_message_t msg;
			mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_SHUTDOWN, 0);
			mav->sendMessage(msg);
		}
	}
	else
		qDebug() << "Don't Shutdown";
}

void UAlbertaControlWidget::initAttitude()
{
	UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(uas));
	if (mav)
	{
		mavlink_message_t msg;
		mavlink_msg_ualberta_action_pack(uas, 0, &msg, UALBERTA_INIT_ATTITUDE, 0);
		mav->sendMessage(msg);
	}
}

void UAlbertaControlWidget::setActiveUAS(UASInterface* uas)
{
	qDebug() << "setting ualberta UAS";
    qDebug() << "\n\n\n\n\n\n\n\n\n\n\n\n";

    // disconnect current mav
	if (this->uas > 0)
    {
		UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(UASManager::instance()->getUASForId(this->uas));
		if (mav)
        {
			disconnect(mav, SIGNAL(servo_source(QString)), ui.servo_source_label, SLOT(setText(QString)));
			disconnect(mav, SIGNAL(controlMode(QString)), ui.control_mode_label, SLOT(setText(QString)));
			disconnect(mav, SIGNAL(pilotMode(QString)), ui.pilot_mode_label, SLOT(setText(QString)));
			disconnect(mav, SIGNAL(gx3Message(QString)), ui.gx3_message_label, SLOT(setText(QString)));
			disconnect(mav, SIGNAL(gx3Status(QString)), ui.gx3_state_label, SLOT(setText(QString)));
			disconnect(mav, SIGNAL(positionChanged(QVector<float>, QVector<float>)), this, SLOT(displayPosition(QVector<float>, QVector<float>)));
			disconnect(mav, SIGNAL(velocityChanged(QVector<float>)), this, SLOT(displayVelocity(QVector<float>)));
			disconnect(mav, SIGNAL(originChanged(QVector<float>)), this, SLOT(displayOrigin(QVector<float>)));
			disconnect(ui.ned_origin_button, SIGNAL(clicked()), mav, SLOT(setOrigin()));
			disconnect(mav, SIGNAL(eulerChanged(QVector<float>, QVector<float>)), this, SLOT(displayVelocity(QVector<float>, QVector<float>)));
			disconnect(mav, SIGNAL(attitudeSource(QString)), ui.attitude_source_label, SLOT(setText(QString)));
			disconnect(mav, SIGNAL(rpm(int,int)), this, SLOT(displayRPM(int,int)));
			disconnect(mav, SIGNAL(collective(float)), this, SLOT(displayCollective(float)));
            disconnect(mav, SIGNAL(altimeter(float)), this, SLOT(displayAltimeter(float)));
			disconnect(mav, SIGNAL(voltages(float,float)), this, SLOT(displayVoltages(float,float)));
			disconnect(mav, SIGNAL(novatel_satellites(int)),this, SLOT(displaySatellites(int)));
			disconnect(mav,SIGNAL(novatel_gps_position(int,int,QVector<float>)),this,SLOT(displayPositionError(int,int,QVector<float>)));
			disconnect(mav,SIGNAL(novatel_gps_velocity(int,QVector<float>)),this,SLOT(displayVelocityError(int,QVector<float>)));

		}
	}

    if(uas && uas->getAutopilotType() == MAV_AUTOPILOT_UALBERTA) {

        UAlbertaMAV* mav = dynamic_cast<UAlbertaMAV*>(uas);
        if (mav)
        {
            connect(mav, SIGNAL(servoSource(QString)), ui.servo_source_label, SLOT(setText(QString)));
            connect(mav, SIGNAL(controlMode(QString)), ui.control_mode_label, SLOT(setText(QString)));
            connect(mav, SIGNAL(pilotMode(QString)), ui.pilot_mode_label, SLOT(setText(QString)));
            connect(mav, SIGNAL(gx3Message(QString)), ui.gx3_message_label, SLOT(setText(QString)));
            connect(mav, SIGNAL(gx3Status(QString)), ui.gx3_state_label, SLOT(setText(QString)));
            connect(mav, SIGNAL(positionChanged(QVector<float>, QVector<float>)), this, SLOT(displayPosition(QVector<float>, QVector<float>)));
            connect(mav, SIGNAL(velocityChanged(QVector<float>)), this, SLOT(displayVelocity(QVector<float>)));
            connect(mav, SIGNAL(originChanged(QVector<float>)), this, SLOT(displayOrigin(QVector<float>)));
            connect(ui.ned_origin_button, SIGNAL(clicked()), mav, SLOT(setOrigin()));
            connect(mav, SIGNAL(eulerChanged(QVector<float>, QVector<float>)), this, SLOT(displayEuler(QVector<float>, QVector<float>)));
            connect(mav, SIGNAL(attitudeSource(QString)), ui.attitude_source_label, SLOT(setText(QString)));
            connect(mav, SIGNAL(rpm(int,int)), this, SLOT(displayRPM(int,int)));
            connect(mav, SIGNAL(collective(float)), this, SLOT(displayCollective(float)));
            connect(mav, SIGNAL(altimeter(float)), this, SLOT(displayAltimeter(float)));
            connect(mav, SIGNAL(refPosChanged(QVector<float>)), this, SLOT(displaySetpoint(QVector<float>)));
            connect(mav, SIGNAL(voltages(float,float)), this, SLOT(displayVoltages(float,float)));
            connect(mav, SIGNAL(novatel_satellites(int)),this, SLOT(displaySatellites(int)));
            connect(mav,SIGNAL(novatel_gps_position(int,int,QVector<float>)),this,SLOT(displayPositionError(int,int,QVector<float>)));
            connect(mav,SIGNAL(novatel_gps_velocity(int,QVector<float>)),this,SLOT(displayVelocityError(int,QVector<float>)));
        }

        this->uas = uas->getUASID();

        qDebug() << "set ualberta UAS!";
        qDebug() << "\n\n\n\n\n\n\n\n\n\n\n\n";

    } else {
        qDebug() << "got autopilot that wasn't ualberta!";
        this->uas = -1;
    }

}

void UAlbertaControlWidget::displayRPM(int engine,int rotor)
{
        ui.engine_rpm_label->setText(QString("%1").arg(engine));
        ui.rotor_rpm_label->setText(QString("%1").arg(rotor));
}

void UAlbertaControlWidget::displayVoltages(float receiver,float avionics)
{
        ui.receiver_bat_label->setText(QString("%1 V").arg(receiver));
        ui.avionics_bat_label->setText(QString("%1 V").arg(avionics));
}

void UAlbertaControlWidget::displaySatellites(int sats)
{
    ui.satellites_label->setText(QString("%1").arg(sats));
}

QString UAlbertaControlWidget::getStrVelocityPositionErr(int type)
{
    QString velocityErrorType;
    switch(type)
    {
    case 0:
        velocityErrorType = "No solution";
        break;
    case 1:
        velocityErrorType = "Fixed Position";
        break;
    case 2:
        velocityErrorType = "Fixed Height";
        break;
    case 8:
        velocityErrorType = "Doppler Velocity";
        break;
    case 16:
        velocityErrorType = "Single Point";
        break;
    case 17:
        velocityErrorType = "Pseudorange Diff";
        break;
    case 18:
        velocityErrorType = "WAAS";
        break;
    case 19:
        velocityErrorType = "Propagated";
        break;
    case 20:
        velocityErrorType = "OmniSTAR";
        break;
    case 32:
        velocityErrorType = "L1 float";
        break;
    case 33:
        velocityErrorType = "Ionosphere free float";
        break;
    case 34:
        velocityErrorType = "Narrow float";
        break;
    case 48:
        velocityErrorType = "L1 Integer";
        break;
    case 50:
        velocityErrorType = "Narrow integer";
        break;
    case 64:
        velocityErrorType = "OmniSTAR HP";
        break;
    case 65:
        velocityErrorType = "OmniSTAR XP";
        break;
    default:
        velocityErrorType = QString("%1").arg(type);
    }

    return velocityErrorType;
}

void UAlbertaControlWidget::displayPositionError(int type, int statusInt, QVector<float> error)
{
    ui.pos_sol_type_label->setText(getStrVelocityPositionErr(type));

    QString status = QString("%1").arg(statusInt);

    switch(statusInt)
    {
    case 0:
        status = QString("Solution Computed");
        break;
    case 1:
        status = QString("Insufficient Observations");
        break;
    case 2:
        status = QString("No Convergence");
        break;
    case 3:
        status = QString("Singularity");
        break;
    case 4:
        status = QString("Covariance Trace");
        break;
    case 5:
        status = QString("Cold Start");
        break;
    case 6:
        status = QString("Height/Vel. Limit Exceeded");
        break;
    case 7:
        status = QString("Variance");
        break;
    case 8:
        status = QString("Integrity Warning");
        break;
    case 9:
        status = QString("Pending");
        break;
    case 10:
        status = QString("Invalid Fixed Positoin");
        break;
    case 11:
        status = QString("Unauthorized");
        break;
    default:
        status = QString("%1").arg(statusInt);
    }

    ui.pos_sol_status_label->setText(status);
    ui.pos_std_label->setText(QString("%1 %2 %3").arg(error[0],7,'f',4).arg(error[1],7,'f',4).arg(error[2],7,'f',4));
}

void UAlbertaControlWidget::displayVelocityError(int type, QVector<float> error)
{
    ui.vel_sol_type_label->setText(getStrVelocityPositionErr(type));
    ui.vel_std_label->setText(QString("%1 %2 %3").arg(error[0],7,'f',4).arg(error[1],7,'f',4).arg(error[2],7,'f',4));
}

void UAlbertaControlWidget::displayPosition(QVector<float> llh, QVector<float> ned)
{
    ui.ned_position_label->setText(QString("%1, %2, %3").arg(ned[0],7,'f',4).arg(ned[1],7,'f',4).arg(ned[2],7,'f',4));
    ui.llh_position_label->setText(QString("%1, %2, %3").arg(llh[0],8,'f',4).arg(llh[1],8,'f',4).arg(llh[2],8,'f',4));
}

void UAlbertaControlWidget::displayOrigin(QVector<float> origin)
{
	ui.ned_origin_label->setText(QString("%1, %2, %3").arg(origin[0]).arg(origin[1]).arg(origin[2]));
}

void UAlbertaControlWidget::displaySetpoint(QVector<float> setpoint)
{
	ui.position_reference_label->setText(QString("%1, %2, %3").arg(setpoint[0]).arg(setpoint[1]).arg(setpoint[2]));
}

void UAlbertaControlWidget::displayCollective(float collective)
{
	ui.collective_pitch_label->setText(QString("%1").arg(collective));
}

void UAlbertaControlWidget::displayAltimeter(float dist)
{
    ui.altimeter_height_label->setText(QString("%1").arg(dist));
}

void UAlbertaControlWidget::displayVelocity(QVector<float> vel)
{
    ui.ned_velocity_label->setText(QString("%1, %2, %3").arg(vel[0],6,'f',3).arg(vel[1],6,'f',3).arg(vel[2],6,'f',3));
}

void UAlbertaControlWidget::displayEuler(QVector<float> ahrs, QVector<float> nav)
{
    qDebug() << "got new euler angles";
    ui.ahrs_euler_label->setText(QString("%1, %2, %3").arg(ahrs[0],8, 'f', 3).arg(ahrs[1], 8, 'f', 3).arg(ahrs[2], 8, 'f', 3));
    ui.nav_euler_label->setText(QString("%1, %2, %3").arg(nav[0],8,'f',3).arg(nav[1],8,'f',3).arg(nav[2],8,'f',3));
}
