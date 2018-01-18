/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package redbacks.robot;

import com.ctre.CANTalon;

import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	private static final CtrlCANTalon
		talon2	= new CtrlCANTalon(2),
		talon3	= new CtrlCANTalon(3),
		talon4	= new CtrlCANTalon(4),
		talon5	= new CtrlCANTalon(5),
		talon6	= new CtrlCANTalon(6);

	public static final CtrlCANTalon
		idMotDriveL	 = talon2,
		idMotDriveR	 = talon3,
		idMotArm	 = talon4,
		idMotIntakeL = talon5,
		idMotIntakeR = talon6;
}