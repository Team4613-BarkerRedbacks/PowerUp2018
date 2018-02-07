package redbacks.robot.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.Robot;

public class AcReadSensors extends Action {
	
	public AcReadSensors() {
		super(new ChFalse());
	}
	
	public void onRun() {
		SmartDashboard.putNumber("Arm Encoder", Robot.sensors.armEncoder.get());
		SmartDashboard.putNumber("Drivetrain Centre Encoder", Robot.sensors.driveCentreEncoder.get());
		SmartDashboard.putNumber("Drivetrain Left Encoder", Robot.sensors.driveLeftEncoder.get());
		SmartDashboard.putNumber("Drivetrain Right Encoder", Robot.sensors.driveRightEncoder.get());
	}
}
