package redbacks.robot.actions;

import static redbacks.robot.Robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;

public class AcReadSensors extends Action
{
	public AcReadSensors() {
		super(new ChFalse());
	}
	
	public void onRun() {
		SmartDashboard.putNumber("Arm Encoder", sensors.armEncoder.get());
	}
}
