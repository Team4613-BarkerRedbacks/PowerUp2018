package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.OI;
import redbacks.robot.Robot;

public class AcArm extends Action
{
	public AcArm() {
		super(new ChFalse());
	}
	
	public void onRun() {
		Robot.arm.armMotor.set(OI.axis_o_LY.get(), command);
//		System.out.println(OI.axis_o_LY.get());
	}
	
	public double calculateCompensation() {
		int range = 900;
		double angle = Math.max(-range, Math.min(range, Robot.sensors.armEncoder.get())) / range * Math.PI / 2;
		
		return Math.sin(angle) * 0.1;
	}
}
