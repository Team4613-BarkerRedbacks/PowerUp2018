package redbacks.robot.actions;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;
/**
 * 
 * @author Ben Schwarz
 *
 */
public class AcMovetoCube extends Action {
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	public double x, sp;
	
	public AcMovetoCube(Double speed, Check check) {
		super(check);
		this.sp = speed;
	
	}
	public void onstart() {
		table.getEntry("ledMode").setValue(1);
	}
	
	public void onRun() {
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ta = table.getEntry("ta");
		
		double x = tx.getDouble(0);
		double area = ta.getDouble(0);
		//TODO Finalise kP
		Robot.driver.drivetrain.tankDrive(
				sp - (x) * RobotMap.cubeTrackkp,
				sp + (x) * RobotMap.cubeTrackkp
		);
	}
}
