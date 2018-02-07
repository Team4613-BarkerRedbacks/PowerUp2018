package redbacks.robot;

import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;

public class RobotMap
{
	public static final double armSpeed = 1;
	
	private static final CtrlCANTalon
		talon2	= new CtrlCANTalon(2),
		talon3	= new CtrlCANTalon(3),
		talon4	= new CtrlCANTalon(4),
		talon5	= new CtrlCANTalon(5),
		talon6	= new CtrlCANTalon(6),
		talon7  = new CtrlCANTalon(7),
		talon8	= new CtrlCANTalon(8),
		talon9	= new CtrlCANTalon(9),
		talon10	= new CtrlCANTalon(10),
		talon11	= new CtrlCANTalon(11),
		talon12  = new CtrlCANTalon(12);
		
	public static final CtrlCANTalon
		idMotDriveR1 = talon2,
		idMotDriveL1 = talon3,
		idMotDriveR2 = talon4,
		idMotDriveL2 = talon5,
		idMotDriveR3 = talon6,
		idMotDriveL3 = talon7,
		idMotArm	 = talon8,
		idMotIntakeL = talon9,
		idMotIntakeR = talon10,
		idMotClimberL = talon11,
		idMotClimberR = talon12;
		
	public static final int
		idSolCentreEncoder = 0,
		idSolShooter = 1,
		idSolLock = 2,
		idSolRightIntake = 3,
		idSolLeftIntake = 4;
}