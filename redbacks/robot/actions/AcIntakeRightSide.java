package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.Check;
import redbacks.robot.RobotMap;

import static redbacks.robot.RobotMap.idMotIntakeL;
import static redbacks.robot.RobotMap.idMotIntakeR;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class AcIntakeRightSide extends Action
{
	public AcIntakeRightSide(Check check) {
		super(check);
	}

	public void onStart() {
		idMotIntakeL.set(ControlMode.PercentOutput, RobotMap.intakeSlowSpeed);
		idMotIntakeR.set(ControlMode.PercentOutput, RobotMap.intakeSpeed);
	}
	
	public void onFinish() {
		idMotIntakeR.follow(idMotIntakeL);
		idMotIntakeL.set(ControlMode.PercentOutput, 0);
	}
}
