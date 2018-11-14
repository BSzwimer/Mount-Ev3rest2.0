package ca.mcgill.ecse211.mountev3rest.controller;

import java.io.IOException;
import java.util.Map;
import org.json.simple.parser.ParseException;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometerException;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.util.ArmController;
import ca.mcgill.ecse211.mountev3rest.util.CoordinateMap;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Performs all the high level tasks required to complete the overall routine.
 * 
 * The {@code MetaController} class represents the high level of the controller.
 * This class's concern is to ensure that the robot completes the entire routine
 * consisting of reaching a tree of rings, obtaining the rings, and returning
 * them back to the initial location. To achieve this goal, the
 * {@code MetaController} determines which calls should be made to the
 * {@code DomainController} and how the robot should react to the outcome of
 * those calls. The {@code DomainController} class in turn handles the
 * subtleties of each subtask.
 * 
 * Additionally, the {@code MetaController} makes sure that the appropriate
 * external conditions are met before requesting that a given subtask is
 * performed. These external conditions are defined by the executor of the
 * subtask (usually the {@code DomainController}).
 * 
 * @see MetaController
 * 
 * @author angelortiz
 *
 */
public class MetaController {

	// Constants
	private static final String SERVER_IP = "192.168.2.23";
	private static final int TEAM_NUMBER = 11;
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	public static void main(String[] args) throws OdometerException, PollerException {

		// System.out.println(String.format("Tunnel: LL[%d, %d] UR[%d, %d]",
		// map.TN_LL_x, map.TN_LL_y,
		// map.TN_UR_x, map.TN_UR_y));

		/*double[][] points = { { 3.5, 1.5 }, { 3.5, 3.5 } };

		TextLCD lcd = LocalEV3.get().getTextLCD();

		lcd.drawString("Press any button", 0, 3);
		lcd.drawString("    to start.   ", 0, 4);

		Button.waitForAnyPress();

		// DomainController controller = new DomainController(map);
		// controller.testNavigation(true, 0, points, lcd);
		// controller.grabRings(0);
		System.exit(0);*/
		run();
	}

	// Attributes
	/*DomainController domainController;
	CoordinateMap map;
	int ringCapacity;
	int ringsLeft;
	boolean ringsIdentified;
	int[][] ringMap;*/

	/**
	 * Creates a high level controller that can perform ring collection as a set of
	 * subtasks.
	 * 
	 * @param totalRings
	 *            Total rings to be found in the tree.
	 */
	public MetaController(int totalRings) {

		// domainController = new DomainController();
		// map = new CoordinateMap();
	}

	/**
	 * Performs all the required subtasks involved in the ring collection routine.
	 * The method runs on an infinite loop that exits when all the rings have been
	 * collected.
	 * 
	 * @throws PollerException
	 * @throws OdometerException
	 */
	public static void run() throws OdometerException, PollerException {
		// Get Wi-Fi information.
		CoordinateMap map = getWiFiData();

		// Create controller.
		DomainController domainController = new DomainController(map);

		// START RING SEARCH ROUTINE

		domainController.localize();
		domainController.crossTunnel();
		//domainController.approachTree(Map.T_LL, Map.T_UR);
		//domainController.grabRings();
		//domainController.crossTunnel();
		System.exit(0);
	}

	@SuppressWarnings("rawtypes")
	private static CoordinateMap getWiFiData() {
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
		Map data = null;
		try {
			data = conn.getData();
		} catch (IOException | ParseException e) {
			e.printStackTrace();
		}

		return new CoordinateMap(data, TEAM_NUMBER);
	}

}
