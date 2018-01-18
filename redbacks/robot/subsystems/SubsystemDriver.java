package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.lib.motors.CtrlDrive;
import redbacks.arachne.lib.motors.CtrlDrivetrain;
import redbacks.robot.RobotMap;

public class SubsystemDriver extends SubsystemBase {
	public CtrlDrive left = new CtrlDrive(RobotMap.idMotDriveL);
	public CtrlDrive right = new CtrlDrive(RobotMap.idMotDriveR);	
	public CtrlDrivetrain drivetrain = new CtrlDrivetrain(left, right);
}
