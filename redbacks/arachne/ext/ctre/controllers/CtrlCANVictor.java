package redbacks.arachne.ext.ctre.controllers;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * This class has the same purpose as CtrlCANTalon, and will be added to Arachne for 2019.
 *
 * @author Sean Zammit
 */
public class CtrlCANVictor extends WPI_VictorSPX
{
	public CtrlCANVictor(int deviceNumber) {
		super(deviceNumber);
	}
}
