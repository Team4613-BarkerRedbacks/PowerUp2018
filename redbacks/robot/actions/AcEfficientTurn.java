package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChMulti;
import redbacks.arachne.lib.checks.analog.ChGettableNumber;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.logic.ListOperators;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.sensors.SenTimer;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.arachne.lib.trajectories.Path;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;

/**
 * @author Sean Zammit
 */
public class AcEfficientTurn extends AcPath
{
	public AcEfficientTurn(double angleTarget) {
		super(new ChMulti(
				LogicOperators.OR,
				new ChMulti(
						LogicOperators.AND,
						new ChNumSen(angleTarget + 3, Robot.sensors.yaw, false, false, false),
						new ChNumSen(angleTarget - 3, Robot.sensors.yaw, true, false, false)
				),
				new ChMulti(
						ListOperators.ORDER,
						new ChGettableNumber(RobotMap.stoppedTurnThreshold * 10, Robot.sensors.rateYaw, true, true),
						new ChNumSen(0.5, new SenTimer()),
						new ChGettableNumber(RobotMap.stoppedTurnThreshold, Robot.sensors.rateYaw, false, true)
				)
		), false, new Path(new double[]{0, angleTarget, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, Robot.sensors.distanceEncoder, false, new Tolerances.Absolute(0));
	}
	
	public void onStart() {
		path.reset();
		ArachneRobot.isIndivDriveControl = false;
		acLinear.initialise(command);
		acRotation.initialise(command);
	}
	
	public void onRun() {
		acRotation.execute();
		
		if(path.getAngleFromDistance(0) > gyro.get()) drivetrain.tankDrive(rotationOut.output, 0.1);
		else drivetrain.tankDrive(0.1, - rotationOut.output);
	}
}