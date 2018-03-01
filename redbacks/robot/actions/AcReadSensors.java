package redbacks.robot.actions;

import static redbacks.robot.RobotMap.sideEncoderTicksPerMetre;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.Robot;

public class AcReadSensors extends Action
{
	public AcReadSensors() {
		super(new ChFalse());
	}
	
	public void onRun() {
		SmartDashboard.putNumber("Arm Encoder", Robot.sensors.armEncoder.get());
		SmartDashboard.putNumber("Drive Centre Encoder", Robot.sensors.driveCentreEncoder.get());
		
		Robot.sensors.averageEncoder.setScaleFactor((double) MotionSettings2.encoderTicksPerMetre / sideEncoderTicksPerMetre);
		SmartDashboard.putNumber("Drive Average Encoder", Robot.sensors.averageEncoder.get());
		
		SmartDashboard.putNumber("Drive Left Encoder", Robot.sensors.driveLeftEncoder.get());
		SmartDashboard.putNumber("Drive Right Encoder", Robot.sensors.driveRightEncoder.get());

		SmartDashboard.putNumber("Heading", Robot.sensors.yaw.get());
		SmartDashboard.putNumber("Heading Rate", Robot.sensors.rateYaw.get());
		
		SmartDashboard.putNumber("Speed Forward", Robot.sensors.speedForward.get());
	}
}
