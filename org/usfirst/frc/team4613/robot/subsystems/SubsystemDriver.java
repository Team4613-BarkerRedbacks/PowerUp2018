package org.usfirst.frc.team4613.robot.subsystems;

import org.usfirst.frc.team4613.robot.RobotMap;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.lib.motors.CtrlDrive;
import redbacks.arachne.lib.motors.CtrlDrivetrain;

public class SubsystemDriver extends SubsystemBase {
	public CtrlDrive left = new CtrlDrive(RobotMap.idMotDriveL);
	public CtrlDrive right = new CtrlDrive(RobotMap.idMotDriveR);	
	public CtrlDrivetrain drivetrain = new CtrlDrivetrain(left, right);
}
