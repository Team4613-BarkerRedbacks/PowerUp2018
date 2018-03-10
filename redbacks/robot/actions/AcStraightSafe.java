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
public class AcStraightSafe extends AcPath
{
	public boolean shouldReset;
	
	public AcStraightSafe(double distance, double angle, NumericSensor encoder, boolean shouldReset) {
		super(new ChFalse(), true, new Path(new double[]{distance, angle, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, encoder, false, new Tolerances.Absolute(0.20 * MotionSettings2.encoderTicksPerMetre));
		this.shouldReset = shouldReset;
	}
	
	public AcStraightSafe(double distance, double angle, NumericSensor encoder, boolean shouldReset, GettableNumber minOut, GettableNumber maxOut) {
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
		
		drivetrain.tankDrive(linearOutput + rotationOut.output, linearOutput - rotationOut.output);
	}
}