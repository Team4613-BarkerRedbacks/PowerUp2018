package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.PIDController;
import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.motion.pid.AcMultiPID.PIDAxis;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.robot.Robot;
import redbacks.robot.actions.AcArm;

/**
 * Subsystem to control the rotation of the arm.
 * The arm motor is sent instructions based on a {@link PIDController}'s output to a {@link PIDAxis} using {@link AcArm}.
 * 
 * @author Sean Zammit
 */
public class SubsystemArm extends SubsystemBase
{
	public CtrlMotor armMotor = new CtrlMotor(idMotArm);
	public PIDAxis armRawOutput = new PIDAxis(1);
	
	public PIDController armPIDControl = new PIDController(armKP, armKI, armKD, Robot.sensors.armEncoder, armRawOutput);
}
