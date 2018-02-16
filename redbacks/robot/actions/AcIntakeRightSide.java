package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.Check;
import static redbacks.robot.RobotMap.idMotIntakeL;
import static redbacks.robot.RobotMap.idMotIntakeR;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class AcIntakeRightSide extends Action
{
	public AcIntakeRightSide(Check check) {
		super(check);
	}

	public void onStart() {
		idMotIntakeR.set(ControlMode.PercentOutput, 0.8);
		idMotIntakeL.set(ControlMode.PercentOutput, 0.5);
	}
	
	public void onFinish() {
		idMotIntakeR.follow(idMotIntakeL);
		idMotIntakeL.set(ControlMode.PercentOutput, 0);
	}
}
