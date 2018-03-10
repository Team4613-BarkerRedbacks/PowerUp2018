package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.Check;
import redbacks.robot.RobotMap;

import static redbacks.robot.RobotMap.idMotIntakeL;
import static redbacks.robot.RobotMap.idMotIntakeR;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class AcOuttakeLeftSide extends Action
{
	public AcOuttakeLeftSide(Check check) {
		super(check);
	}

	public void onStart() {
		idMotIntakeL.set(ControlMode.PercentOutput, 0);
		idMotIntakeR.set(ControlMode.PercentOutput, -RobotMap.intakeFastSpeed);
	}
	
	public void onFinish() {
		idMotIntakeR.follow(idMotIntakeL);
		idMotIntakeL.set(ControlMode.PercentOutput, 0);
	}
}
