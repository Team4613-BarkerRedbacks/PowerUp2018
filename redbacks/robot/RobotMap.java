package redbacks.robot;

import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.ext.ctre.controllers.CtrlCANVictor;

public class RobotMap
{
	public static final double
		armMaxSpeed = 1,
		armKP = 1.5E-3, armKI = 1E-6, armKD = 2E-3;
	
	public static final int
		armSwitchPos = 350, armBasePos = 1025;
	
	private static final CtrlCANTalon
		talon2	= new CtrlCANTalon(2),
		talon3	= new CtrlCANTalon(3),
		talon4	= new CtrlCANTalon(4),
		talon5	= new CtrlCANTalon(5),
		talon6	= new CtrlCANTalon(6),
		talon7  = new CtrlCANTalon(7),
		talon8	= new CtrlCANTalon(8);
	
	private static final CtrlCANVictor
		victor9		= new CtrlCANVictor(9),
		victor10	= new CtrlCANVictor(10),
		victor11	= new CtrlCANVictor(11),
		victor12	= new CtrlCANVictor(12);
		
	public static final CtrlCANTalon
		idMotDriveR1 = talon2,
		idMotDriveL1 = talon3,
		idMotDriveR2 = talon4,
		idMotDriveL2 = talon5,
		idMotDriveR3 = talon6,
		idMotDriveL3 = talon7,
		idMotArm	 = talon8;
	
	public static final CtrlCANVictor
	//FIXME Check LR on intake controllers
		idMotIntakeL = victor9,
		idMotIntakeR = victor10,
		idMotClimb1 = victor11,
		idMotClimb2 = victor12;
		
	public static final int
		idSolCentreEncoder = 1,//
		idSolRightIntake = 5,
		idSolLeftIntake = 4,
		idSolShooter = 0,
		idSolLock = 6,
		idSolClimbRelease = 3;
}