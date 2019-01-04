package redbacks.arachne.lib.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A minimum functionality class to read the Modern Robotics color sensor.
 * Will be added to Arachne once functionality is complete (likely 2020).
 *
 * @author Matthew Brian, Sean Zammit
 */
public class ModernRoboticsColorSensor extends SensorBase
{
	private static final byte
		kAddress = 0x1E,
		
		//Commands
		kActiveMode = 0x00,
		kPassiveMode = 0x01,
		
		kOperate50Hz = 0x35,
		kOperate60Hz = 0x36,
		
		kCalibrateBlack = 0x42,
		kCalibrateWhite = 0x43,
		
		//Registers
		kCommandRegister = 0x03,
		
//		kColorNumberRegister = 0x04,
//		kRedValRegister = 0x05,
//		kGreenValRegister = 0x06,
//		kBlueValRegister = 0x07,
		kWhiteValRegister = 0x08,
		
		RedValRegister = 0x16,
		GreenValRegister = 0x18,
		BlueValRegister = 0x1a;
	
	protected I2C m_i2c;
	
	public static enum Mode {
		ACTIVE(kActiveMode),
		PASSIVE(kPassiveMode),
		CAL_BLACK(kCalibrateBlack),
		CAL_WHITE(kCalibrateWhite);
		
		public byte command;
		
		Mode(byte command) {
			this.command = command;
		}
	}
	
	public static enum Frequency {
		HZ_50(kOperate50Hz),
		HZ_60(kOperate60Hz);
		
		public byte command;
		
		Frequency(byte command) {
			this.command = command;
		}
	}

	public static enum ColorComponent {
		RED(RedValRegister),
		GREEN(GreenValRegister),
		BLUE(BlueValRegister),
		ALPHA(kWhiteValRegister);
		
		public byte register;
		
		ColorComponent(byte register) {
			this.register = register;
		}
	}
	
	public static class ColorData {
		public short r, g, b, a;
	}
	
	public ModernRoboticsColorSensor(I2C.Port port, Frequency freq) {
		this(port, freq, kAddress);
	}

	public ModernRoboticsColorSensor(I2C.Port port, Frequency freq, int deviceAddress) {
		m_i2c = new I2C(port, deviceAddress);

		setFreq(freq);

		m_i2c.write(kCommandRegister, kActiveMode);

		setName("Modern_Robotics_Color_Sensor", port.value);
	}
	
	public void outputAll() {
		for(int i = 0; i < 0xff; i += 2) {
			ByteBuffer rawData = ByteBuffer.allocate(2);
			m_i2c.read(i, 2, rawData);

			rawData.order(ByteOrder.BIG_ENDIAN);
			SmartDashboard.putNumber(String.valueOf(i), rawData.getShort(0));
		}
	}
	
	public int getCommand() {
		ByteBuffer rawData = ByteBuffer.allocate(2);
		m_i2c.read(kCommandRegister, 2, rawData);

		rawData.order(ByteOrder.BIG_ENDIAN);
		return rawData.getShort(0);
	}

	@Override
	public void free() {
		super.free();
		m_i2c.free();
	}
	
	public void setOperationMode(Mode mode) {
		m_i2c.write(kCommandRegister, mode.command);
	}

	public void setFreq(Frequency freq) {
		m_i2c.write(kCommandRegister, freq.command);
	}

	public double getRed() {
		return getColorValue(ColorComponent.RED);
	}

	public double getGreen() {
		return getColorValue(ColorComponent.GREEN);
	}

	public double getBlue() {
		return getColorValue(ColorComponent.BLUE);
	}

	public double getAlpha() {
		return getColorValue(ColorComponent.ALPHA);
	}

//	public Color getColor() {
//		ColorData data = new ColorData();
//		ByteBuffer rawData = ByteBuffer.allocate(8);
//		m_i2c.read(kColorNumberRegister, 8, rawData);
//
//		rawData.order(ByteOrder.BIG_ENDIAN);
//		data.r = rawData.getShort(0);
//		data.g = rawData.getShort(2);
//		data.b = rawData.getShort(4);
//		data.a = rawData.getShort(6);
//		return new Color(data.r, data.g, data.b, data.a);
//	}

	public double getColorValue(ColorComponent color) {
		
		ByteBuffer rawColor = ByteBuffer.allocate(2);
		
		m_i2c.read(color.register, 2, rawColor);
		
		rawColor.order(ByteOrder.LITTLE_ENDIAN);
		
		return rawColor.getShort(0);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		//TODO Implement
	}
}
