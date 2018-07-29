package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.logic.GettableNumber;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.arachne.lib.sensors.NumericSensor;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.arachne.lib.trajectories.Path;
import redbacks.robot.Robot;

/**
 * @author Sean Zammit
 */
public class AcStraightFinishOnTarget extends AcPath
{
	public boolean shouldReset;
	
	public AcStraightFinishOnTarget(double distance, double angle, NumericSensor encoder, boolean shouldReset) {
		super(new ChFalse(), true, new Path(new double[]{distance, angle + (distance > 0 ? 1 : -1) * 3, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, encoder, false, new Tolerances.Absolute(0.02 * MotionSettings2.encoderTicksPerMetre));
		this.shouldReset = shouldReset;
	}
	
	public AcStraightFinishOnTarget(double distance, double angle, NumericSensor encoder, boolean shouldReset, GettableNumber minOut, GettableNumber maxOut) {
		this(distance, angle, encoder, shouldReset);
		this.minOut = minOut;
		this.maxOut = maxOut;
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
		
		double linearOutput = linearOut.output;
		if(minOut != null && maxOut != null) linearOutput = Math.max(Math.min(linearOutput, maxOut.get()), minOut.get());
		
		if(Math.abs(linearOutput) < MotionSettings2.driveMinVoltage) {
			if(linearOutput >= 0) linearOutput = MotionSettings2.driveMinVoltage;
			else linearOutput = -MotionSettings2.driveMinVoltage;
		}
		
		drivetrain.tankDrive(linearOutput + rotationOut.output, linearOutput - rotationOut.output);
	}
	
	public void onFinish() {
		super.onFinish();
		
		System.out.println(encoder.get());
	}
}