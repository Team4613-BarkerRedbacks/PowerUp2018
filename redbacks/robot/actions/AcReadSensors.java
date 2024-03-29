package redbacks.robot.actions;

import static redbacks.robot.RobotMap.limelightTable;
import static redbacks.robot.Robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.Robot;
import redbacks.robot.subsystems.SubsystemSensors;

/**
 * Runs constantly on the sensors subsystem. Outputs relevant data to the SmartDashboard.
 *
 * @author Darin Huang, Mitchell Barker, Sean Zammit
 */
public class AcReadSensors extends Action
{
	public AcReadSensors() {
		super(new ChFalse());
	}
	
	public void onRun() {
		limelightTable.getEntry("ledMode").setNumber(1);
		limelightTable.getEntry("camMode").setNumber(Robot.isLimelightVision ? 0 : 1);

		SmartDashboard.putNumber("Cube found", limelightTable.getEntry("tv").getDouble(0));
		SmartDashboard.putNumber("Cube offset", limelightTable.getEntry("ty").getDouble(0));
		SmartDashboard.putNumber("Camera mode", limelightTable.getEntry("camMode").getDouble(-1));
		
		SmartDashboard.putNumber("Arm Encoder", sensors.armEncoder.get());
		SmartDashboard.putNumber("Drive Distance Encoder", sensors.distanceEncoder.get());
		SmartDashboard.putNumber("Metres", sensors.distanceEncoder.get() / MotionSettings2.encoderTicksPerMetre);
		
		SmartDashboard.putNumber("Drive Average Encoder", sensors.averageEncoder.get());

		SmartDashboard.putNumber("Drive Left Encoder", sensors.driveLeftEncoder.get());
		SmartDashboard.putNumber("Drive Right Encoder", sensors.driveRightEncoder.get());

		SmartDashboard.putNumber("Heading", sensors.yaw.get());
		SmartDashboard.putNumber("Heading Rate", sensors.rateYaw.get());

		SmartDashboard.putNumber("Speed Forward", sensors.driveSpeed.get());
		
//		sensors.colorSensor.outputAll();
//		sensors.colorSensor.setOperationMode(Mode.PASSIVE);
		
		SmartDashboard.putNumber("RED", sensors.colorSensor.getRed());
		SmartDashboard.putNumber("BLUE", sensors.colorSensor.getBlue());
		SmartDashboard.putNumber("GREEN", sensors.colorSensor.getGreen());
		SmartDashboard.putNumber("ALPHA", sensors.colorSensor.getAlpha());
		SmartDashboard.putBoolean("On White Line", SubsystemSensors.isWhite(
				sensors.colorSensor.getRed(),
				sensors.colorSensor.getGreen(),
				sensors.colorSensor.getBlue()
		));
		SmartDashboard.putNumber("Color Command Type", sensors.colorSensor.getCommand());
	}
}
