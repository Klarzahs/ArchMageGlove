
public class Main {
	private SerialCom sCom;
	private Thread keepAlive;
	private ComplementaryFilter[] filter = new ComplementaryFilter[3];
	
	public Main(){
		sCom = new SerialCom();
		sCom.initialize(this);
		
		for(int i = 0; i < filter.length; i++){
			filter[i] = new ComplementaryFilter(i);
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
	
	public ComplementaryFilter getFilter(int nr){
		if(nr < filter.length) return filter[nr];
		return null;
	}
	
	
	
	public static void main(String[] args) throws Exception {
		Main m = new Main();
	}
}
