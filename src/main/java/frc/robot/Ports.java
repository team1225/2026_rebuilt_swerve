package frc.robot;

/**
 * Contains the definitions of all the ports
 */
public class Ports {

		// TO DO: Check on these addresses
		// IP (v4) addresses
		// The purpose of this section is to serve as a reminder of what static IP (v4) addresses are used so they are consistent
		// between the competition and practice robots.
		//
		// The radio is automatically set to 10.12.25.1
		// The Rio is set to static 10.12.25.2, mask 255.255.255.0
		// The Limelight is set to 10.12.25.11, mask 255.255.255.0, gateway 10.12.25.1
		// but note that pressing the reset button will revert to DHCP.
		// The Raspberry Pi running FRCVision is set to static 10.12.25.12, mask 255.255.255.0, gateway 10.12.25.1, DNS blank
		//
		// If a device cannot be accessed (e.g. because its address was somehow obtained via DHCP and mDNS is not working),
		// use Angry IP Scanner to find it!


		/**
		 * Digital ports
		 */
		public static class Digital {
		}
		
		/**
		 * Analog ports
		 */
		public static class Analog {
		}
		
		/**
		 * Relays
		 */
		public static class Relay {
		}
		
		/**
		 * CAN Ids
		 */
		public static class CAN {
			
			//2026 Serve module drive base
			public static final int RIO = 0;
			public static final int PDP = 1;	

			public static final int pigeon2 = 2;

			//Spark Max CAN IDs
			public static final int FRONT_LEFT_DRIVING = 6;
			public static final int REAR_LEFT_DRIVING = 4;
			public static final int FRONT_RIGHT_DRIVING = 8;
			public static final int REAR_RIGHT_DRIVING = 10;

			public static final int FRONT_LEFT_TURNING = 5;
			public static final int REAR_LEFT_TURNING = 3;
			public static final int FRONT_RIGHT_TURNING = 7;
			public static final int REAR_RIGHT_TURNING = 9;

			public static final int INTAKE = 14;
			public static final int AGITATOR = 15;
			public static final int SHOOTER = 16;
		}
		
		/**
		 * USB ports
		 */
		public static class USB {
			public static final int DRIVER_CONTROLLER = 0;
			public static final int CODRIVER_CONTROLLER = 1;
			public static final int CHARACTERIZATION_CONTROLLER = 2;
		}
		
		/**
		 * PCM ports
		 */
		public static class PCM {
		}

		/**
		 * PWM ports
		 */
		public static class PWM {
			public static final int LED_STRIP = 9;
		}

		/**
		 * USB cameras
		 */
		public static class UsbCamera {
			public static final int FLOOR_CAMERA = 0;
			public static final int SHOOTER_CAMERA = 1;
			public static final int TOP_CAMERA = 2;
		}
}
