package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.idMotIntakeL;
import static redbacks.robot.RobotMap.idMotIntakeR;

import com.ctre.CANTalon.TalonControlMode;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.lib.motors.CtrlMotor;
import static redbacks.robot.RobotMap.*;

public class SubsystemClimber extends SubsystemBase {
	
	// 2x 775 Motors
	public CtrlMotor climberMotor = new CtrlMotor(idMotClimberL);
	
	public SubsystemClimber() {
		super();
		
		idMotClimberR.setControlMode(TalonControlMode.Follower.value);
		idMotClimberR.set(((CtrlCANTalon) climberMotor.controller).getDeviceID());
	}
}
