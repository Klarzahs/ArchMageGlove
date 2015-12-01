
public class Main {
	private SerialCom sCom;
	private Thread keepAlive;
	private KalmanFilter2[][] filter = new KalmanFilter2[1][3];
	
	public Main(){
		sCom = new SerialCom();
		sCom.initialize(this);
		
		for(int i = 0; i < filter[0].length; i++){
			filter[0][i] = new KalmanFilter2();
		}
		
		
		initThread();
	}
	
	
	
	private void initThread(){
		keepAlive=new Thread() {
			private boolean isRunning = true;
			public void halt(){ isRunning = false;}
			
			public void run() {
				try {
					while(isRunning) Thread.sleep(1);
				} catch (InterruptedException ie) {}
			}
		};
		keepAlive.start();
		System.out.println("Started");
	}
	
	public KalmanFilter2 getFilter(int nr){
		if(nr < filter[0].length) return filter[0][nr];
		return null;
	}
	
	
	
	public static void main(String[] args) throws Exception {
		Main m = new Main();
	}
}
