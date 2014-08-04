/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009, 2010 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

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

/**
 * @file
 *   @brief Definition of class UAlbertaControlWidget
 *
 *   @author Bryan Godbolt <godbolt@ece.ualberta.ca
 *
 */

#ifndef _UALBERTACONTROLWIDGET_H_
#define _UALBERTACONTROLWIDGET_H_

#include <QWidget>
#include <UASInterface.h>
#include <ui_UAlbertaControlWidget.h>

/**
 * @brief Widget controlling one MAV
 */
class UAlbertaControlWidget : public QWidget
{
    Q_OBJECT

public:
    UAlbertaControlWidget(QWidget *parent = 0);
    ~UAlbertaControlWidget();

public slots:
    /** @brief Set the system this widget controls */
    void setUAS(UASInterface* uas);
    /// slot for servo source button
	void setServoSource();
	/// slot for control mode button
	void setControlMode();
	/// slot for origin button
	void setOrigin();
	/// slot to set attitude source
	void setAttitudeSource();
	/// slot to kill the autopilot
	void shutdown();
	/// slot to reset the nav filter
	void resetFilter();
	/// slot to set the initial attitude
	void initAttitude();
	/// slot to set the position setpoint
	void setReferencePosition();
    /// slot to start new log files
    void startNewLog();

	/// update position display
	void displayPosition(QVector<float> llh, QVector<float> ned);
	/// update attitude display
	void displayEuler(QVector<float> ahrs, QVector<float> nav);
	/// update origin display
	void displayOrigin(QVector<float> origin);
	/// update velocity display
	void displayVelocity(QVector<float> vel);
	/// display RPM
	void displayRPM(int engine,int rotor);
	/// display main rotor collective pitch
	void displayCollective(float collective);
    /// display altimeter height
    void displayAltimeter(float dist);
	/// display the position setpoint
	void displaySetpoint(QVector<float> setpoint);
	/// display Voltages
	void displayVoltages(float receiver,float avionics);
	/// display Number of Satellites
	void displaySatellites(int sats);
	/// display Position Standard Deviation
	void displayPositionError(int type, int status, QVector<float> error);
	/// display Velocity Standard Deviation
	void displayVelocityError(int type, QVector<float> error);




protected:
    int uas;              ///< Reference to the current uas

private:
    Ui::UAlbertaControlWidget ui;

    /// get the velocity position string from the error type
    QString getStrVelocityPositionErr(int type);

};

#endif
