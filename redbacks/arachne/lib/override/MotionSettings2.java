package redbacks.arachne.lib.override;

import redbacks.arachne.ext.motion.MotionSettings;

/**
 * 
 *
 * @author Sean Zammit
 */
public class MotionSettings2 extends MotionSettings
{
	public static final double
		drivePIDMotorkP = 4.0E-5, drivePIDMotorkI = 1.0E-8, drivePIDMotorkD = 7.0E-6,
		//gyro kp 4 -> 3
		drivePIDGyrokP = 4.0E-2, drivePIDGyrokI = 3.0E-6, drivePIDGyrokD = 7.0E-3,
		
		/**
		 * This variable has a running changelog. Do not edit it without adding an entry.
		 * 23/3: Set to initial value of 0.35
		 * 28/3: 0.35 -> 0.3 -> 0.2 -> 0.35 -> 0.3
		 */
		driveMinVoltage = 0.35,
		
		
		driveSlowVoltage = 0.4;
}
