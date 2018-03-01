package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.arachne.lib.trajectories.Path;
import redbacks.robot.Robot;

/**
 * @author Sean Zammit
 */
public class AcTurnPrecise extends AcPath
{
	double angleTarget;
	
	public AcTurnPrecise(double angleTarget) {
		super(new ChFalse(), false, new Path(new double[]{0, angleTarget, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, Robot.sensors.driveCentreEncoder, false, new Tolerances.Absolute(0));
		
		this.angleTarget = angleTarget;
	}
	
	public void onStart() {
		path.reset();
		ArachneRobot.isIndivDriveControl = false;
		acLinear.initialise(command);
		acRotation.initialise(command);
	}
	
	public void onRun() {
		acRotation.execute();
		
		drivetrain.tankDrive(rotationOut.output, - rotationOut.output);
	}
	
	public boolean isDone() {
		return gyro.get() < angleTarget + 3 && gyro.get() > angleTarget - 3;
	}
}