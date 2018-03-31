package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.Check;

import static redbacks.robot.RobotMap.idMotIntakeL;
import static redbacks.robot.RobotMap.idMotIntakeR;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class AcSplitIntakeControl extends Action
{
	double lSpeed, rSpeed;
	
	public AcSplitIntakeControl(Check check, double lSpeed, double rSpeed) {
		super(check);
		this.lSpeed = lSpeed;
		this.rSpeed = rSpeed;
	}

	public void onStart() {
		idMotIntakeL.set(ControlMode.PercentOutput, lSpeed);
		idMotIntakeR.set(ControlMode.PercentOutput, rSpeed);
	}
	
	public void onFinish() {
		idMotIntakeR.follow(idMotIntakeL);
		idMotIntakeL.set(ControlMode.PercentOutput, 0);
	}
}
