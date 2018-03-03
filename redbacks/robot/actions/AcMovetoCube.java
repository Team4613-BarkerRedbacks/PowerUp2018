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
	
	public void onStart() {
		Robot.isLimelightVision = true;
		lastSeen = System.currentTimeMillis();
	}

	public void onRun() {
		double offset = limelightTable.getEntry("ty").getDouble(0);
		double rot = offset * RobotMap.cubeTrackKP, dirMult = Robot.sensors.armEncoder.get() > 0 ? 1 : -1;
		
		Robot.driver.drivetrain.tankDrive((sp + rot) * dirMult, (sp - rot) * dirMult);
		
		if(limelightTable.getEntry("tv").getDouble(0) == 1) lastSeen = System.currentTimeMillis();
	}
	
	public void onFinish() {
		Robot.isLimelightVision = false;
	}
	
	public boolean isDone() {
		return System.currentTimeMillis() - lastSeen > 250 || limelightTable.getEntry("ta").getDouble(0) > 90;
	}
}
