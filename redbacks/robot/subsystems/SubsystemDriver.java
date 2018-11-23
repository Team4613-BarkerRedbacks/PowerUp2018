package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlDrive;
import redbacks.arachne.lib.motors.CtrlDrivetrain;
import redbacks.arachne.lib.solenoids.SolSingle;
import redbacks.robot.RobotMap;

import static redbacks.robot.RobotMap.*;

public class SubsystemDriver extends SubsystemBase {
	public CtrlDrive leftMotor = new CtrlDrive(RobotMap.idMotDriveL);
	public CtrlDrive rightMotor = new CtrlDrive(RobotMap.idMotDriveR);	
	public CtrlDrivetrain drivetrain = new CtrlDrivetrain(leftMotor, rightMotor);
	
	public SolSingle centreEncoderSol = new SolSingle(idSolCentreEncoder);
}
