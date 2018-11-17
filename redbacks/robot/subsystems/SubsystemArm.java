package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.PIDController;
import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.motion.pid.AcMultiPID.PIDAxis;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.robot.Robot;
/**
 * 
 * Arm subsystem controls the rotational arm using a pid loop.
 *
 */
public class SubsystemArm extends SubsystemBase
{
	public CtrlMotor armMotor = new CtrlMotor(idMotArm);
	public PIDAxis armRawOutput = new PIDAxis(1);
	
	public PIDController armPIDControl = new PIDController(armKP, armKI, armKD, Robot.sensors.armEncoder, armRawOutput);

	public SubsystemArm(SubsystemBase... childSystems) {
		super(childSystems);
	}
}
