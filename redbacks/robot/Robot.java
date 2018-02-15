package redbacks.robot;

import redbacks.arachne.core.ArachneRobot;

/**
 * @author Sean Zammit, Matthew Brian, Ben Schwartz, Lucas Parker, Mitchell Barker, Darin Huang
 */

import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.subsystems.*;
import static redbacks.robot.CommandList.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends ArachneRobot
{
	public static double 
		posX = 0,
		posY = 0;
	
	public static SubsystemSensors		sensors		= new SubsystemSensors();
	public static SubsystemDriver		driver		= new SubsystemDriver();
	public static SubsystemIntake		intake		= new SubsystemIntake();
//	public static SubsystemClimber		climber		= new SubsystemClimber();
	public static SubsystemArm			arm			= new SubsystemArm();
	public static SubsystemShooter		shooter		= new SubsystemShooter();
	
	public static OI oi = new OI();
	
	public void initDefaultCommands() {
		driver.setDefaultCommand(drive.c());
		sensors.setDefaultCommand(readSensors.c());
		arm.setDefaultCommand(armLoop.c());
		oi.mapOperations();
	}

	public CommandBase getAutonomous(int autoID) {
		return Auto.getAutonomous(autoID);
	}
	
	public void initialiseRobot() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		
		table.getEntry("ledMode").forceSetValue(1);
		
		MotionSettings2.encoderTicksPerMetre = 26713;
//		MotionSettings2.trajectoryAngleForesight = 4;

		Robot.sensors.yawMonitor.set(0);
		Robot.sensors.driveMonitorEncoderL.set(0);
		Robot.sensors.driveMonitorEncoderR.set(0);
		Thread t = new Thread(() -> {
			double oldDis = 0, oldAng = 0;
			long lastTime = System.currentTimeMillis();
			
            while (!Thread.interrupted()) {
            	long newTime = System.currentTimeMillis();
            	SmartDashboard.putNumber("Timer Hz", newTime - lastTime);
            	lastTime = newTime;
            	
            	double newDis = Robot.sensors.driveMonitorEncoderL.get() + Robot.sensors.driveMonitorEncoderR.get();
        		double newAng = Robot.sensors.yawMonitor.get();
        		
        		double disPos = newDis - oldDis;
        		double avgAng = Math.abs((oldAng + newAng) / 2);
        		
        		if(Math.abs(oldAng - newAng) > 180) avgAng += 180;
        		
        		posX += disPos * Math.cos(Math.toRadians(avgAng));
        		posY += disPos * Math.sin(Math.toRadians(avgAng));
        		
        		SmartDashboard.putNumber("X Position", posX / 30000);
        		SmartDashboard.putNumber("Y Position", posY / 30000);
        		
        		oldDis = newDis;
        		oldAng = newAng;
            }
        });
        t.start();
	}
}
