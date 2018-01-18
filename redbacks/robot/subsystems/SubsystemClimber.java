package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.idMotIntakeL;
import static redbacks.robot.RobotMap.idMotIntakeR;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import static redbacks.robot.RobotMap.*;

public class SubsystemClimber extends SubsystemBase {
	
	// 2x 775 Motors
	public CtrlMotor climberLeftMotor = new CtrlMotor(idMotClimberL);
	public CtrlMotor climberRightMotor = new CtrlMotor(idMotClimberR);
	
	// protected CANTalon lslave1 = idMotClimberL;
	// protected CANTalon rslave1 = idMotClimberR;
	
	
	public SubsystemClimber() {
		super();
		
		// lslave1.setControlMode(TalonControlMode.Follower.value);
		// lslave1.(((CANTalon) left.controller).getDeviceID());
		
		// rslave1.setControlMode(TalonControlMode.Follower.value);
		// rslave1.(((CANTalon) right.controller).getDeviceID());
	}
}
