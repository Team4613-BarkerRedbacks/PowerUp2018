package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.IMotorController;

public class SubsystemIntake extends SubsystemBase
{
	public CtrlMotor intakeMotor = new CtrlMotor(idMotIntakeL);

	public SolSingle intakeLeftSol = new SolSingle(idSolLeftIntake);
	public SolSingle intakeRightSol = new SolSingle(idSolRightIntake);

	public SubsystemIntake(SubsystemBase... childSystems) {
		super(childSystems);
		
		idMotIntakeR.follow((IMotorController) intakeMotor.controller);
	}
}
