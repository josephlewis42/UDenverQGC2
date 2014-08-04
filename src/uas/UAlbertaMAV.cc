/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009 - 2011 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

This file is part of the QGROUNDCONTROL project

    QGROUNDCONTROL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QGROUNDCONTROL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/
#include "UAlbertaMAV.h"
#include "ualberta.h"

UAlbertaMAV::UAlbertaMAV(MAVLinkProtocol* protocol, QThread* thread,  int id)
:UAS(protocol, thread,  id)
{


}

void UAlbertaMAV::receiveMessage(LinkInterface* link, mavlink_message_t message)
{

	if (message.sysid == uasId)
	{
		switch(message.msgid)
		{
        case MAVLINK_MSG_ID_UALBERTA_ALTIMETER:
        {
            mavlink_ualberta_altimeter_t altimeter_data;
            mavlink_msg_ualberta_altimeter_decode(&message, &altimeter_data);
            float meters = altimeter_data.dist / 100;
            emit altimeter(meters);
            emit valueChanged(uasId, "Altimeter", "m", meters, getUnixTime());
            //qDebug() << "Got message \n" << altimeter_data.dist;
            break;
        }
		case MAVLINK_MSG_ID_UALBERTA_SYS_STATUS:
		{
//			qDebug() << "Got UALBERTA sys status";
			mavlink_ualberta_sys_status_t status;
			mavlink_msg_ualberta_sys_status_decode(&message, &status);

			emit voltages(status.receiver_voltage,status.avionics_voltage);
			emit rpm(status.engine_rpm,status.rotor_rpm);
			emit collective(status.collective);
			emit valueChanged(uasId, "Engine RPM", "RPM", status.engine_rpm, 0);
			emit valueChanged(uasId, "Main Rotor RPM", "RPM", status.rotor_rpm, 0);

			switch (status.mode)
			{
			case UALBERTA_MODE_MANUAL_DIRECT:
				emit servoSource("Direct Manual");
				break;
			case UALBERTA_MODE_MANUAL_SCALED:
				emit servoSource("Scaled Manual");
				break;
			case UALBERTA_MODE_AUTOMATIC_CONTROL:
				emit servoSource("Automatic Control");
				break;
			default:
				emit servoSource("Unknown Mode");
			}

			switch(status.gx3_mode)
			{
			case UALBERTA_GX3_STARTUP:
				emit gx3Status("Startup");
				break;
			case UALBERTA_GX3_INIT:
				emit gx3Status("Initialization");
				break;
			case UALBERTA_GX3_RUNNING_VALID:
				emit gx3Status("Running");
				break;
			case UALBERTA_GX3_RUNNING_ERROR:
				emit gx3Status("Solution Error");
				break;
			default:
				emit gx3Status("Unknown Mode");
				break;
			}

			switch (status.pilot_mode)
			{
			case UALBERTA_PILOT_MANUAL:
				emit pilotMode("Manual");
				break;

			case UALBERTA_PILOT_AUTO:
				emit pilotMode("Computer");
				break;
			default:
				emit pilotMode("Unknown Mode");
				break;
			}

			switch(status.control_mode)
			{
			case UALBERTA_ATTITUDE_PID:
				emit controlMode("Attitude PID");
				break;
			case UALBERTA_TRANSLATION_PID:
				emit controlMode("Translation PID");
				break;
			case UALBERTA_TRANSLATION_SBF:
				emit controlMode("Translation SBF");
				break;
			default:
				emit controlMode("Unknown Mode");
				break;
			}

			switch(status.attitude_source)
			{
			case UALBERTA_NAV_FILTER:
				emit attitudeSource("Nav Filter");
				break;
			case UALBERTA_AHRS:
				emit attitudeSource("AHRS");
				break;
			default:
				emit attitudeSource("Unknown Mode");
			}

			break;
		}
                case MAVLINK_MSG_ID_NOVATEL_GPS_RAW:
                {
                        mavlink_novatel_gps_raw_t status;
                        mavlink_msg_novatel_gps_raw_decode(&message,&status);

                        emit novatel_satellites(status.num_sats);

                        QVector<float> pos_error;
                        QVector<float> vel_error;
                        for (int i=0; i<MAVLINK_MSG_ID_NOVATEL_GPS_RAW_LEN; ++i)
                                {
                                    pos_error << status.pos_error[i];
                                    vel_error << status.vel_error[i];
                                }

                        emit novatel_gps_position(status.pos_type,status.pos_status,pos_error);
                        emit novatel_gps_velocity(status.vel_type,vel_error);

                        break;

                }
		case MAVLINK_MSG_ID_UALBERTA_GX3_MESSAGE:
		{
			mavlink_ualberta_gx3_message_t status;
			mavlink_msg_ualberta_gx3_message_decode(&message, &status);
//			qDebug() << "sending message: " << status.message;
			emit gx3Message(status.message);
			break;
		}
		case MAVLINK_MSG_ID_RADIO_CALIBRATION:
		{
			mavlink_radio_calibration_t radioMsg;
			mavlink_msg_radio_calibration_decode(&message, &radioMsg);
			QVector<uint16_t> aileron;
			QVector<uint16_t> elevator;
			QVector<uint16_t> rudder;
			QVector<uint16_t> gyro;
			QVector<uint16_t> pitch;
			QVector<uint16_t> throttle;

			for (int i=0; i<MAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_LEN; ++i)
				aileron << radioMsg.aileron[i];
			for (int i=0; i<MAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_LEN; ++i)
				elevator << radioMsg.elevator[i];
			for (int i=0; i<MAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_LEN; ++i)
				rudder << radioMsg.rudder[i];
			for (int i=0; i<MAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_LEN; ++i)
				gyro << radioMsg.gyro[i];
			for (int i=0; i<MAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_LEN; ++i)
				pitch << radioMsg.pitch[i];
			for (int i=0; i<MAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_LEN; ++i)
				throttle << radioMsg.throttle[i];

			QPointer<RadioCalibrationData> radioData = new RadioCalibrationData(aileron, elevator, rudder, gyro, pitch, throttle);
			emit radioCalibrationReceived(radioData);
			delete radioData;
			break;
		}
		case MAVLINK_MSG_ID_UALBERTA_POSITION:
		{
			mavlink_ualberta_position_t pos;
			mavlink_msg_ualberta_position_decode(&message, &pos);
			QVector<float> llh_pos;
			for (int i=0; i<3; ++i)llh_pos << pos.llh_pos[i];
			QVector<float> ned_pos;
			for (int i=0; i<3; i++)ned_pos << pos.ned_pos[i];
			QVector<float> origin;
			for (int i=0; i<3; i++)origin << pos.ned_origin[i];
			QVector<float> vel;
			for (int i=0; i<3; i++)vel << pos.ned_vel[i];
			QVector<float> ref_pos;
			for (int i=0; i<3; i++) ref_pos << pos.reference_position[i];
			QVector<float> body_error;
			for (int i=0; i<3; i++) body_error << pos.position_error_body[i];
			QVector<float> ned_error;
			for (int i=0; i<3; i++) ned_error << pos.position_error_ned[i];

			emit positionChanged(llh_pos, ned_pos);
			emit velocityChanged(vel);
			emit originChanged(origin);
			emit refPosChanged(ref_pos);

			quint64 time = getUnixReferenceTime(pos.time_boot_ms);

			// plot llh position
			emit valueChanged(uasId, "Latitude", "deg", llh_pos[0], time);
			emit valueChanged(uasId, "Longitude", "deg", llh_pos[1], time);
			emit valueChanged(uasId, "Height", "m", llh_pos[2], time);

			// plot ned position
			emit valueChanged(uasId, "NED x", "m", ned_pos[0], time);
			emit valueChanged(uasId, "NED y", "m", ned_pos[1], time);
			emit valueChanged(uasId, "NED z", "m", ned_pos[2], time);

			// plot ned velocity
			emit valueChanged(uasId, "NED VEL x", "m/s", vel[0], time);
			emit valueChanged(uasId, "NED VEL y", "m/s", vel[1], time);
			emit valueChanged(uasId, "NED VEL z", "m/s", vel[2], time);

			// plot position errors
			emit valueChanged(uasId, "NED x Reference", "m", ref_pos[0], time);
			emit valueChanged(uasId, "NED y Reference", "m", ref_pos[1], time);
			emit valueChanged(uasId, "Body x error", "m", body_error[0], time);
			emit valueChanged(uasId, "Body y error", "m", body_error[1], time);
			emit valueChanged(uasId, "NED x error", "m", ned_error[0], time);
			emit valueChanged(uasId, "NED y error", "m", ned_error[1], time);

			break;
		}
		case MAVLINK_MSG_ID_UALBERTA_ATTITUDE:
		{
			mavlink_ualberta_attitude_t attitude;
			mavlink_msg_ualberta_attitude_decode(&message, &attitude);
			quint64 time = getUnixReferenceTime(attitude.time_boot_ms);
//			lastAttitude = time;

			// copy out euler angles
			QVector<float> nav_euler;
			for (int i=0; i<3; ++i) nav_euler << attitude.nav_euler[i]*180/M_PI;
			QVector<float> ahrs_euler;
			for (int i=0; i<3; ++i) ahrs_euler << attitude.ahrs_euler[i]*180/M_PI;
			QVector<float> attitude_reference;
			for (int i=0; i<2; ++i) attitude_reference << attitude.attitude_reference[i]*180/M_PI;


//			roll = QGC::limitAngleToPMPIf(nav_euler[0]);
//			pitch = QGC::limitAngleToPMPIf(nav_euler[1]);
//			yaw = QGC::limitAngleToPMPIf(nav_euler[2]);
			emit valueChanged(uasId, "nav roll", "deg", nav_euler[0], time);
			emit valueChanged(uasId, "nav pitch", "deg", nav_euler[1], time);
			emit valueChanged(uasId, "nav yaw", "deg", nav_euler[2], time);
			emit valueChanged(uasId, "nav rollspeed", "deg/s", attitude.nav_euler_rate[0]*180/M_PI, time);
			emit valueChanged(uasId, "nav pitchspeed", "deg/s", attitude.nav_euler_rate[1]*180/M_PI, time);
			emit valueChanged(uasId, "nav yawspeed", "deg/s", attitude.nav_euler_rate[2]*180/M_PI, time);
			emit valueChanged(uasId, "Reference Roll", "deg", attitude_reference[0], time);
			emit valueChanged(uasId, "Reference Pitch", "deg", attitude_reference[1], time);

			// Emit in angles

			// Convert yaw angle to compass value
			// in 0 - 360 deg range
//			float compass = (yaw/M_PI)*180.0+360.0f;
//			while (compass > 360.0f) {
//				compass -= 360.0f;
//			}

//			attitudeKnown = true;

			emit valueChanged(uasId, "ahrs roll", "deg", ahrs_euler[0], time);
			emit valueChanged(uasId, "ahrs pitch", "deg", ahrs_euler[1], time);
			emit valueChanged(uasId, "ahrs yaw", "deg", ahrs_euler[2], time);
			emit valueChanged(uasId, "ahrs rollspeed", "deg/s", attitude.ahrs_euler_rate[0]/M_PI*180.0, time);
			emit valueChanged(uasId, "ahrs pitchspeed", "deg/s", attitude.ahrs_euler_rate[1]/M_PI*180.0, time);
			emit valueChanged(uasId, "ahrs yawspeed", "deg/s", attitude.ahrs_euler_rate[2]/M_PI*180.0, time);

//			emit attitudeChanged(this, roll, pitch, yaw, time);
//			emit attitudeSpeedChanged(uasId, attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed, time);

			emit eulerChanged(ahrs_euler, nav_euler);
			break;
		}
		case MAVLINK_MSG_ID_UALBERTA_CONTROL_EFFORT:
		{
			//                    qDebug() << "Received control effort";
			mavlink_ualberta_control_effort_t control;
			mavlink_msg_ualberta_control_effort_decode(&message, &control);
			QVector<float> effort;
			for (int i =0; i<6; i++) effort << control.normalized_control_effort[i];

			valueChanged(uasId, "Aileron", "norm", effort[0], 0);
			valueChanged(uasId, "Elevator", "norm", effort[1], 0);
		}
		break;
		case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
		{
			mavlink_rc_channels_scaled_t channels;
			mavlink_msg_rc_channels_scaled_decode(&message, &channels);
			emit remoteControlRSSIChanged(channels.rssi/255.0f);
			emit remoteControlChannelScaledChanged(0, channels.chan1_scaled/10000.0f);
			emit valueChanged(uasId, "Pilot Aileron", "norm", channels.chan1_scaled/10000.0f, 0);
			emit remoteControlChannelScaledChanged(1, channels.chan2_scaled/10000.0f);
			emit valueChanged(uasId, "Pilot Elevator", "norm", channels.chan2_scaled/10000.0f, 0);
			emit remoteControlChannelScaledChanged(2, channels.chan3_scaled/10000.0f);
			emit remoteControlChannelScaledChanged(3, channels.chan4_scaled/10000.0f);
			emit remoteControlChannelScaledChanged(4, channels.chan5_scaled/10000.0f);
			emit remoteControlChannelScaledChanged(5, channels.chan6_scaled/10000.0f);
			emit remoteControlChannelScaledChanged(6, channels.chan7_scaled/10000.0f);
			emit remoteControlChannelScaledChanged(7, channels.chan8_scaled/10000.0f);

            emit valueChanged(uasId, "Channel 1 scaled", "", channels.chan1_scaled/10000.0f, getUnixTime());
            emit valueChanged(uasId, "Channel 2 scaled", "", channels.chan2_scaled/10000.0f, getUnixTime());
            emit valueChanged(uasId, "Channel 3 scaled", "", channels.chan3_scaled/10000.0f, getUnixTime());
            emit valueChanged(uasId, "Channel 4 scaled", "", channels.chan4_scaled/10000.0f, getUnixTime());
            emit valueChanged(uasId, "Channel 5 scaled", "", channels.chan5_scaled/10000.0f, getUnixTime());
            emit valueChanged(uasId, "Channel 6 scaled", "", channels.chan6_scaled/10000.0f, getUnixTime());
            emit valueChanged(uasId, "Channel 7 scaled", "", channels.chan7_scaled/10000.0f, getUnixTime());
            emit valueChanged(uasId, "Channel 8 scaled", "", channels.chan8_scaled/10000.0f, getUnixTime());

		}
		break;

		case MAVLINK_MSG_ID_UDENVER_CPU_USAGE:
		{
    		// saves the CPU usage from mavlink
    		mavlink_udenver_cpu_usage_t cpu;

		// converts the generic message we got to the CPU one we understand
    		mavlink_msg_udenver_cpu_usage_decode(&message, &cpu);

    		// emit a "CPU usage" event, which allows us to graph the usage
    		// we use the current UNIX time as the timestamp, although you may
    		// want to get this from the system instead, left as an exercise
    		// for the reader.
    		emit valueChanged(uasId, "CPU usage", "", cpu.cpu_usage, getUnixTime());
    		emit valueChanged(uasId, "Total RAM", "mb", cpu.mem_total_kb, getUnixTime());
       		emit valueChanged(uasId, "Free RAM", "mb", cpu.mem_free_kb, getUnixTime());
    			break;
		}

		default:
			UAS::receiveMessage(link, message);
			break;
		}

	}
}

void UAlbertaMAV::setOrigin()
{
	mavlink_message_t msg;
	mavlink_msg_ualberta_action_pack(uasId, 100, &msg, UALBERTA_SET_ORIGIN,0);
	sendMessage(msg);
}
