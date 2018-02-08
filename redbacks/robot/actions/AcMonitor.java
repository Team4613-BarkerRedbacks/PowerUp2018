package redbacks.robot.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.Robot;

/**
 * 
 * 
 * @author Lucas Parker, Sean Zammit, Mitchell Barker
 */

public class AcMonitor extends Action {
	
	public static double 
		posX = 0,
		posY = 0,
		oldDis = 0,
		oldAng = 0;
	
	public AcMonitor() {
		super(new ChFalse());
	}

	public void onRun() {
		
		// DO NOT DELETE ----------------------------------------------------------------------------------------------------------------------------------------------------------
		
		// Detect the x and y position of the Robot on the field
		// using the Encoder and the Gyro
		// Encoder: How many wheel rotations
		// Gyro: Angle of robot
		
		// Method ----------------------------------------------------------------------------------------------------------------------------------------------------------
		// At every movement, record the current angle of the robot
		// At every movement, add the distance based off the wheel rotations
		// At every movement, update its position on the plane
		
		// Assume that facing right is 0 degrees (like a Unit circle), and that the range is 179.999 to -179.999
		
		// Things we need:
		// Diameter / radius of wheel
		// Number of encoder positions per one full rotation 
		// Reading from Gyro
		// Reading from Encoder
		
		// For 1m how many encoder positions? Set of 10m and find the amount of encoder clicks. Or, for 1 encoder position how many metres?
		
		// In the code, the robot will move in many straight lines that eventually average out to look like a curve in real life
		
		// OFFICIAL METHOD ----------------------------------------------------------------------------------------------------------------------------------------------------------
		// Detecting the Gyro angle and Encoder position of wheel, add x and y values to current x and y values
		// Measuring theta from the horizontal axis
		// x = EncoderPos * cos ( Gyro angle 1 + Gyro angle 2 / 2 )
		// y = EncoderPos * sin ( Gyro angle 1 + Gyro angle 2 / 2 )
		// For next x and y: add the previous x and y positions as well as the previous formulae
		
		// Encoder position is the distance between the revolution numbers
		
		// Let metres per 1 encoder tick = G
		// The distance travelled is encoder ticks * G - previous encoder ticks * G
		
		// MATH ----------------------------------------------------------------------------------------------------------------------------------------------------------
		
		// Drive center encoder ticks / 5 m: 133566 2/3
		// Left encoder ticks / 5m : 77301 2/3
		// Right encoder ticks / 5m : 69741 1/3
		
		// Center metres / 1 tick: 0.0000374m
		// Left m / 1 tick: 0.0000647
		// Right m / 1 tick: 0.0000717
		
		// Code ----------------------------------------------------------------------------------------------------------------------------------------------------------
		
		double newDis = Robot.sensors.driveCentreEncoder.get();
		double newAng = Robot.sensors.yaw.get();
		
		double disPos = newDis - oldDis;
		double avgAng = Math.abs((oldAng + newAng) / 2);
		
		if(Math.abs(oldAng - newAng) > 180) avgAng += 180;
		
		posX += disPos * Math.cos(Math.toRadians(avgAng));
		posY += disPos * Math.sin(Math.toRadians(avgAng));
		
		SmartDashboard.putNumber("X Position", Math.round(posX/100));
		SmartDashboard.putNumber("Y Position", Math.round(posY/100));
		
		oldDis = newDis;
		oldAng = newAng;
	}
}
// NOTES - line 87 - 88 'Math.round(posX/100)); is temporary, only there to make numbers easier to check 
// bug - only occurs when the robot rotates and returns to zero, bug will not occur when you rotate, stop and then do the reverse
// bug - x axis will return to robot on dash board if robot is on zero, Y will instead increase when you turn and the (0,0) will increase in the Y axis  
// bug - every time you rotate past 90 degrees from the starting point (0,0) the bug starts 