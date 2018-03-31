package redbacks.robot.actions;

import static redbacks.arachne.lib.override.MotionSettings2.*;

import edu.wpi.first.wpilibj.PIDSourceType;
import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.logic.GettableNumber;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.arachne.lib.sensors.NumericSensor;
import redbacks.arachne.lib.trajectories.AcPIDDynamicControl;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.arachne.lib.trajectories.Path;
import redbacks.robot.Robot;

/**
 * @author Sean Zammit
 */
public class AcNCDrive extends AcPath
{
	public boolean shouldReset;
	public AcPIDDynamicControl acSpeedControl;
	public double targetSpeed;
	
	public AcNCDrive(double distance, double angle, NumericSensor encoder, boolean shouldReset) {
		super(new ChFalse(), true, new Path(new double[]{distance, angle, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, encoder, false, new Tolerances.Absolute(0.02 * MotionSettings2.encoderTicksPerMetre));
		this.shouldReset = shouldReset;
	}
	
	public AcNCDrive(double distance, double angle, NumericSensor encoder, boolean shouldReset, GettableNumber minOut, GettableNumber maxOut) {
		this(distance, angle, encoder, shouldReset);
		this.minOut = minOut;
		this.maxOut = maxOut;

		this.acSpeedControl =
			new AcPIDDynamicControl(
				new ChFalse(), false,
				drivePIDMotorkP, drivePIDMotorkI, drivePIDMotorkD, 0,
				new GettableNumber() {
					public double get() {
						return 0;
					}
				},
				new Tolerances.Absolute(0), gyro,
				true, -180, 180,
				PIDSourceType.kDisplacement, -1, 1, rotationOut
		);
	}
	
	public void onStart() {
		path.reset();
		if(shouldReset) encoder.set(0);
		ArachneRobot.isIndivDriveControl = false;
		acLinear.initialise(command);
		acRotation.initialise(command);
	}
	
	public void onRun() {
		acLinear.execute();
		acRotation.execute();
		
		targetSpeed = linearOut.output * 10000;
		
//		double linearOutput = linearOut.output;
//		if(minOut != null && maxOut != null) linearOutput = Math.max(Math.min(linearOutput, maxOut.get()), minOut.get());
//		
//		if(Math.abs(linearOutput) < MotionSettings2.driveMinVoltage) {
//			if(linearOutput >= 0) linearOutput = MotionSettings2.driveMinVoltage;
//			else linearOutput = -MotionSettings2.driveMinVoltage;
//		}
//		
//		drivetrain.tankDrive(linearOutput + rotationOut.output, linearOutput - rotationOut.output);
	}
	
	public void onFinish() {
		super.onFinish();
		
		System.out.println(encoder.get());
	}
	
	public static class ChangeMinMax implements GettableNumber
	{
		public NumericSensor encoder;
		public int triggerDistance;
		public double newSpeed;
		
		public ChangeMinMax(NumericSensor encoder, int triggerDistance, double newSpeed) {
			this.encoder = encoder;
			this.triggerDistance = triggerDistance;
			this.newSpeed = newSpeed;
		}
		
		public double get() {
			return Math.abs(encoder.get()) > triggerDistance ? newSpeed : (newSpeed < 0 ? trajectoryMaxNegSpeed : trajectoryMaxPosSpeed);
		}
	}
	
	public static class ChangeMinMaxNeg extends ChangeMinMax
	{
		public ChangeMinMaxNeg(NumericSensor encoder, int triggerDistance, double newSpeed) {
			super(encoder, triggerDistance, newSpeed);
		}
		
		public double get() {
			return Math.abs(encoder.get()) < triggerDistance ? newSpeed : (newSpeed < 0 ? trajectoryMaxNegSpeed : trajectoryMaxPosSpeed);
		}
	}
}