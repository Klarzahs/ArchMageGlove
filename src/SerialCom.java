import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.Enumeration;

import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;


public class SerialCom implements SerialPortEventListener {
	SerialPort serialPort;
        /** The port we're normally going to use. */
	private static final String PORT_NAMES[] = { 
			"COM3", // Windows
	};
	private BufferedReader input;
	private OutputStream output;
	private static final int TIME_OUT = 2000;
	private static final int DATA_RATE = 38400;
	
	
	private long lastStep;
	
	private Main main;

	public void initialize(Main m) {
		main = m;
		
		CommPortIdentifier portId = null;
		Enumeration portEnum = CommPortIdentifier.getPortIdentifiers();

		//First, Find an instance of serial port as set in PORT_NAMES.
		while (portEnum.hasMoreElements()) {
			CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
			for (String portName : PORT_NAMES) {
				if (currPortId.getName().equals(portName)) {
					portId = currPortId;
					break;
				}
			}
		}
		if (portId == null) {
			System.out.println("Could not find COM port.");
			return;
		}

		try {
			serialPort = (SerialPort) portId.open(this.getClass().getName(),
					TIME_OUT);

			serialPort.setSerialPortParams(DATA_RATE,
					SerialPort.DATABITS_8,
					SerialPort.STOPBITS_1,
					SerialPort.PARITY_NONE);

			input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
			output = serialPort.getOutputStream();

			serialPort.addEventListener(this);
			serialPort.notifyOnDataAvailable(true);
			
			lastStep = System.currentTimeMillis();
		} catch (Exception e) {
			System.err.println(e.toString());
		}
	}

	public synchronized void close() {
		if (serialPort != null) {
			serialPort.removeEventListener();
			serialPort.close();
		}
	}

	public synchronized void serialEvent(SerialPortEvent oEvent) {
		
		if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
			try {
				long dt = lastStep - System.currentTimeMillis();
				lastStep = System.currentTimeMillis();
				
				String inputLine=input.readLine();
				
				if(inputLine.length() > 0){
					String[] arr = inputLine.split("x");
					
					if(arr.length == 7){
						int owner = Integer.parseInt(arr[0]);
						
						float eulerX = Float.parseFloat(arr[1]);
						float eulerY = Float.parseFloat(arr[2]);
						float eulerZ = Float.parseFloat(arr[3]);
						
						float accX = Float.parseFloat(arr[4]);
						float accY = Float.parseFloat(arr[5]);
						float accZ = Float.parseFloat(arr[6]);
						
						if(owner == 0){
							main.getFilter(0).update(eulerX, eulerY, eulerZ, accX, accY, accZ, (float)dt);
						}
					} else{
						System.out.println("received != 7");
					}
				}
				else{
					System.out.println("received <= 0");
				}
				
			} catch (Exception e) {
				System.err.println(e.toString());
			}
		}
	}

}