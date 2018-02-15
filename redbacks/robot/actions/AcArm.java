package redbacks.robot.actions;

import redbacks.arachne.ext.motion.MotionExtender;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;
import static redbacks.robot.Robot.*;

import edu.wpi.first.wpilibj.PIDSourceType;

public class AcArm extends Action
{
	public AcArm() {
		super(new ChFalse());
	}

	public void onStart() {
		sensors.armEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		
		arm.armPIDControl.setInputRange(0, 0);
		arm.armPIDControl.setContinuous(false);
		arm.armPIDControl.setOutputRange(-RobotMap.armMaxSpeed, RobotMap.armMaxSpeed);
		arm.armPIDControl.setSetpoint(0);
		arm.armPIDControl.setAbsoluteTolerance(0);
		arm.armPIDControl.enable();

		MotionExtender.getInstance().activePIDs.add(arm.armPIDControl);
	}
	
	public void onRun() {
		System.out.println(arm.armPIDControl.get() + ", " + calculateCompensation());
		arm.armMotor.set(arm.armRawOutput.output - calculateCompensation(), command);
	}

	public void onFinish() {
		arm.armPIDControl.disable();
		MotionExtender.getInstance().activePIDs.remove(arm.armPIDControl);
	}
	
	public double calculateCompensation() {
		int range = 900;
		double angle = Math.max(-range, Math.min(range, Robot.sensors.armEncoder.get())) / range * Math.PI / 2;
		
		return Math.sin(angle) * 0.1;
	}
}
