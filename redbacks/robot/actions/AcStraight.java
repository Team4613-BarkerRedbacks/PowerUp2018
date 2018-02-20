package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChMulti;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.checks.analog.ChGettableNumber;
import redbacks.arachne.lib.logic.LogicOperators;
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
	public AcStraight(double distance, double angle, NumericSensor encoder, boolean shouldReset) {
		super(new ChMulti(
				LogicOperators.AND,
				new ChTime(0.2),
				new ChGettableNumber(RobotMap.stoppedMoveThreshold, Robot.sensors.driveSpeed, false, true)
		), true, new Path(new double[]{distance, angle, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, encoder, shouldReset, new Tolerances.Absolute(0.1 * MotionSettings2.encoderTicksPerMetre));
	}
	
	public void onStart() {
		path.reset();
		ArachneRobot.isIndivDriveControl = false;
		acLinear.initialise(command);
		acRotation.initialise(command);
	}
	
	public void onRun() {
		acLinear.execute();
		
		drivetrain.tankDrive(linearOut.output, linearOut.output);
	}
}