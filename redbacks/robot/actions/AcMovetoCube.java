package redbacks.robot.actions;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.Robot;

public class AcMovetoCube extends Action {
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	public double x, sp;
	
	protected AcMovetoCube(Double speed, Check check) {
		super(check);
		this.sp = speed;
		// TODO Auto-generated constructor stub
	}
	public void onstart() {
		table.getEntry("ledMode").setValue(1);
	}
	
	public void onRun() {
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ta = table.getEntry("ta");
		
		double x = tx.getDouble(0);
		double area = ta.getDouble(0);
		
		Robot.driver.drivetrain.tankDrive(
				sp - (x) * MotionSettings2.drivePIDGyrokP,
				sp + (x) * MotionSettings2.drivePIDGyrokP
		);
	}
}
