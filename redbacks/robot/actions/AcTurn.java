package redbacks.robot.actions;

import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChMulti;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.checks.analog.ChGettableNumber;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.arachne.lib.trajectories.Path;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;

/**
 * @author Sean Zammit
 */
public class AcTurn extends AcPath
{
	public AcTurn(double angleTarget) {
		super(new ChMulti(
				LogicOperators.AND,
				new ChTime(0.2),
				new ChGettableNumber(RobotMap.stoppedTurnThreshold, Robot.sensors.rateYaw, false, true)
		), false, new Path(new double[]{0, angleTarget, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, Robot.sensors.driveCentreEncoder, false, new Tolerances.Absolute(5));
	}
	
	public void onRun() {
		acRotation.execute();
		
		drivetrain.tankDrive(rotationOut.output, - rotationOut.output);
	}
}