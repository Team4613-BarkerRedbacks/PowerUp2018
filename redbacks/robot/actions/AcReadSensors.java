package redbacks.robot.actions;

import static redbacks.robot.RobotMap.sideEncoderTicksPerMetre;
import static redbacks.robot.RobotMap.limelightTable;
import static redbacks.robot.Robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.override.MotionSettings2;

public class AcReadSensors extends Action
{
	public AcReadSensors() {
		super(new ChFalse());
	}
	
	public void onRun() {
		limelightTable.getEntry("ledMode").forceSetValue(1);

		SmartDashboard.putNumber("Cube found", limelightTable.getEntry("tv").getDouble(0));
		SmartDashboard.putNumber("Cube offset", limelightTable.getEntry("ty").getDouble(0));
		
		SmartDashboard.putNumber("Arm Encoder", sensors.armEncoder.get());
		SmartDashboard.putNumber("Drive Centre Encoder", sensors.driveCentreEncoder.get());
		
		sensors.averageEncoder.setScaleFactor((double) MotionSettings2.encoderTicksPerMetre / sideEncoderTicksPerMetre);
		SmartDashboard.putNumber("Drive Average Encoder", sensors.averageEncoder.get());
		
		SmartDashboard.putNumber("Drive Left Encoder", sensors.driveLeftEncoder.get());
		SmartDashboard.putNumber("Drive Right Encoder", sensors.driveRightEncoder.get());

		SmartDashboard.putNumber("Heading", sensors.yaw.get());
		SmartDashboard.putNumber("Heading Rate", sensors.rateYaw.get());
		
		SmartDashboard.putNumber("Speed Forward", sensors.speedForward.get());
	}
}
