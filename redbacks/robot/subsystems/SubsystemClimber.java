package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.lib.motors.CtrlMotor;
import static redbacks.robot.RobotMap.*;

public class SubsystemClimber extends SubsystemBase {
	
	// 2x 775 Motors
	public CtrlMotor climberMotor = new CtrlMotor(idMotClimberL);
	
	public SubsystemClimber() { 
		super();
		
		// Slave motor (idMotClimberR copying idMotClimberL)
		idMotClimberR.follow((CtrlCANTalon) climberMotor.controller);
	}
}
