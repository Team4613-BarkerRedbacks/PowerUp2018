package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;
import static redbacks.robot.RobotMap.limelightTable;


/**
 * 
 * 
 * @author Ben Schwarz, Sean Zammit
 */
public class AcMovetoCube extends Action
{
	public double sp;
	public long lastSeen;

	public AcMovetoCube(double speed) {
		super(new ChFalse());
		this.sp = speed;
	}

	public void onRun() {
		double x = limelightTable.getEntry("tx").getDouble(0);
		
		//TODO Finalise kP
		double rot = x * RobotMap.cubeTrackKP * (Robot.sensors.armEncoder.get() > 0 ? 1 : -1);
		
		Robot.driver.drivetrain.tankDrive(sp - rot, sp + rot);
		
		if(limelightTable.getEntry("tv").getDouble(0) == 1) lastSeen = System.currentTimeMillis();
	}
	
	public boolean isDone() {
		return System.currentTimeMillis() - lastSeen > 500;
	}
}
