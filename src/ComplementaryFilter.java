import javax.swing.text.html.MinimalHTMLWriter;

public class ComplementaryFilter {

	// Use the following global variables and access functions to help store the overall
	// rotation angle of the sensor
	private long last_read_time;
	private float last_x_angle;  // These are the filtered angles
	private float last_y_angle;
	private float last_z_angle;  
	private float last_gyro_x_angle;  // Store the gyro angles to compare drift
	private float last_gyro_y_angle;
	private float last_gyro_z_angle;
	
	private float base_x_gyro;
	private float base_y_gyro;
	private float base_z_gyro;
	
	private int ID;
	
	public ComplementaryFilter(int i){
		ID = i;
	}

	public void set_last_read_angle_data(long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
	  last_read_time = time;
	  last_x_angle = x;
	  last_y_angle = y;
	  last_z_angle = z;
	  last_gyro_x_angle = x_gyro;
	  last_gyro_y_angle = y_gyro;
	  last_gyro_z_angle = z_gyro;
	}

	private long get_last_time() {return last_read_time;}
	private float get_last_x_angle() {return last_x_angle;}
	private float get_last_y_angle() {return last_y_angle;}
	private float get_last_z_angle() {return last_z_angle;}
	private float get_last_gyro_x_angle() {return last_gyro_x_angle;}
	private float get_last_gyro_y_angle() {return last_gyro_y_angle;}
	private float get_last_gyro_z_angle() {return last_gyro_z_angle;}

	int num_readings = 100;
	
	private int lineCount = 0;
	
	public void input(String[] line){
		try{
			float[] input = parseLine(line);
			if(lineCount < num_readings){
				calibrate_sensors(input, lineCount);
				last_read_time = System.currentTimeMillis();
			}else{
				filterStep(input);
			}
			lineCount++;
		}catch(Exception e){
			System.out.println("CF#"+ID+": "+e.getMessage());
		}
	}
	
	public float[] parseLine(String[] line){
		float[] ret = new float[line.length];
		for (int i = 0; i < ret.length; i++){
			ret[i] = Float.parseFloat(line[i]);
		}
		return ret;
	}

	// The sensor should be motionless on a horizontal surface 
	//  while calibration is happening
	private float x_gyro = 0;
	private float y_gyro = 0;
	private float z_gyro = 0;
	
	private void calibrate_sensors(float[] input, int i) {
	    x_gyro += input[3];
	    y_gyro += input[4];
	    z_gyro += input[5];
	    
	    // Store the raw calibration values globally
	    if(i == num_readings - 1){
	  	   	x_gyro /= num_readings;
	  	   	y_gyro /= num_readings;
	  	   	z_gyro /= num_readings;
	  	   	
	  	   	base_x_gyro = x_gyro;
	  	   	base_y_gyro = y_gyro;
	  	   	base_z_gyro = z_gyro;
	  	   	System.out.println("Sensor calibrated");
	    }
	}
	
	private void filterStep(float[] input){
		 // Convert gyro values to degrees/sec
		float FS_SEL = 131;
		float gyro_x = (input[3] - base_x_gyro)/FS_SEL;
		float gyro_y = (input[4] - base_y_gyro)/FS_SEL;
		float gyro_z = (input[5] - base_z_gyro)/FS_SEL;
	  
	  
		// Get raw acceleration values
		//float G_CONVERT = 16384;
		float accel_x = input[0];
		float accel_y = input[1];
		float accel_z = input[2];
	  
		// Get angle values from accelerometer
		float RADIANS_TO_DEGREES = 180f/3.14159f;
		
		//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
		float accel_angle_y = (float) Math.atan(-1*accel_x/Math.sqrt(Math.pow(accel_y,2) + Math.pow(accel_z,2)))*RADIANS_TO_DEGREES;
		float accel_angle_x = (float) Math.atan(accel_y/Math.sqrt(Math.pow(accel_x,2) + Math.pow(accel_z,2)))*RADIANS_TO_DEGREES;
		float accel_angle_z = (float) Math.atan(Math.sqrt(Math.pow(accel_x,2) + Math.pow(accel_y,2))/accel_z)*RADIANS_TO_DEGREES;
	  
		// Compute the (filtered) gyro angles
		float dt = (System.currentTimeMillis() - get_last_time())/1000.0f;
		float gyro_angle_x = gyro_x*dt + get_last_x_angle();
		float gyro_angle_y = gyro_y*dt + get_last_y_angle();
		float gyro_angle_z = gyro_z*dt + get_last_z_angle();
	  
		// Compute the drifting gyro angles
		float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
		float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
		float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
	  
		// Apply the complementary filter to figure out the change in angle - choice of alpha is
		// estimated now.  Alpha depends on the sampling rate...
		float alpha = 0.96f;
		float angle_x = alpha*gyro_angle_x + (1.0f - alpha)*accel_angle_x;
		float angle_y = alpha*gyro_angle_y + (1.0f - alpha)*accel_angle_y;
		float angle_z = alpha*gyro_angle_z + (1.0f - alpha)*accel_angle_z;
	  

		// Update the saved data with the latest values
		set_last_read_angle_data(System.currentTimeMillis(), angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
		System.out.println(ID+": "+angle_x+" | "+angle_y+" | "+angle_z);
	}
	
	  
	  
}
