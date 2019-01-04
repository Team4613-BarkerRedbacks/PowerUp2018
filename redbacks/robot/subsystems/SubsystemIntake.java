package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.IMotorController;

/**
 * Subsystem to control the intake and side-kick solenoids on the arm.
 * The secondary intake motor is slaved here.
 *
 * @author Lucas Parker, Mitchell Barker, Sean Zammit
 */
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
