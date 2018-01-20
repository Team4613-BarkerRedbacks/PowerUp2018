package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlDrive;
import redbacks.arachne.lib.motors.CtrlDrivetrain;
import redbacks.robot.RobotMap;

import static redbacks.robot.RobotMap.*;

public class SubsystemDriver extends SubsystemBase {
	public CtrlDrive leftMotor = new CtrlDrive(RobotMap.idMotDriveL1);
	public CtrlDrive rightMotor = new CtrlDrive(RobotMap.idMotDriveR1);	
	public CtrlDrivetrain drivetrain = new CtrlDrivetrain(leftMotor, rightMotor);
	
	public SubsystemDriver() {
		super();

		idMotDriveL2.follow(idMotDriveL1);
		idMotDriveL3.follow(idMotDriveL1);

		idMotDriveR2.follow(idMotDriveR1);
		idMotDriveR3.follow(idMotDriveR1);
	}
}
