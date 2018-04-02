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
		//drive pid (4e-5,1e-8,7e-6) -> (2e-5,0,1e-5)
		drivePIDMotorkP = 2.0E-5, drivePIDMotorkI = 0, drivePIDMotorkD = 1.0E-5,
		drivePIDGyrokP = 5.0E-2, drivePIDGyrokI = 3.0E-6, drivePIDGyrokD = 4.0E-3,
		
		/**
		 * This variable has a running changelog. Do not edit it without adding an entry.
		 * 23/3: Set to initial value of 0.35
		 * 28/3: 0.35 -> 0.3 -> 0.2 -> 0.35
		 */
		driveMinVoltage = 0.35,
		driveMinVoltagePro = 0.4,
		
		driveSlowVoltage = 0.6,
		driveSlowVoltagePro = 0.5;
}
