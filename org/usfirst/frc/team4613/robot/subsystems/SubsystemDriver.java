package org.usfirst.frc.team4613.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.lib.motors.CtrlDrive;
import redbacks.arachne.lib.motors.CtrlDrivetrain;

public class SubsystemDriver extends SubsystemBase 
{
	public CtrlDrive left = new CtrlDrive(new CtrlCANTalon(1));
	public CtrlDrive right = new CtrlDrive(new CtrlCANTalon(2));	
	public CtrlDrivetrain drivetrain = new CtrlDrivetrain(left, right);
	
}
