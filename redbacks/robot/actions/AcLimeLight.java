package redbacks.robot.actions;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.Check;
import redbacks.robot.Robot;

public class AcLimeLight extends Action {
	
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	public double x, y;
	
	public AcLimeLight() {
		super(new ChFalse());
		
	}
	public void onStart() {
		table.getEntry("ledMode").setValue(1);
	}
	
	public void onRun() {
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");
		double target = tv.getDouble(0);
		double x = tx.getDouble(0);
		double y = ty.getDouble(0);
		double area = ta.getDouble(0);
		
		if (target == 1) {
			if (Robot.sensors.armEncoder.get() < 0) {
				if (y > 10) Robot.driver.drivetrain.arcadeDrive(0, -0.6); 
				else if (y < -10) Robot.driver.drivetrain.arcadeDrive(0, 0.6);
			} else {
				if (y > 10) Robot.driver.drivetrain.arcadeDrive(0, 0.6); 
				else if (y < -10) Robot.driver.drivetrain.arcadeDrive(0, -0.6);
			}
		}
		SmartDashboard.putNumber("tv: ",tv.getDouble(2468));
		SmartDashboard.putNumber("tx: ",tx.getDouble(2468));
		SmartDashboard.putNumber("ty: ",ty.getDouble(2468));
		SmartDashboard.putNumber("ta: ",ta.getDouble(2468));
	}
}
