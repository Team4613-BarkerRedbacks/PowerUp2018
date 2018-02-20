package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChMulti;
import redbacks.arachne.lib.checks.analog.ChGettableNumber;
import redbacks.arachne.lib.logic.ListOperators;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.arachne.lib.sensors.NumericSensor;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.arachne.lib.trajectories.Path;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;

/**
 * @author Sean Zammit
 */
public class AcStraight extends AcPath
{
	public boolean shouldReset;
	
	public AcStraight(double distance, double angle, NumericSensor encoder, boolean shouldReset) {
		super(new ChMulti(
				ListOperators.ORDER,
				new ChGettableNumber(RobotMap.stoppedMoveThreshold * 10, Robot.sensors.driveSpeed, true, true),
				new ChGettableNumber(RobotMap.stoppedMoveThreshold, Robot.sensors.driveSpeed, false, true)
		), true, new Path(new double[]{distance, angle, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, encoder, false, new Tolerances.Absolute(0.20 * MotionSettings2.encoderTicksPerMetre));
		this.shouldReset = shouldReset;
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
		
		drivetrain.tankDrive(linearOut.output + rotationOut.output, linearOut.output - rotationOut.output);
	}
}