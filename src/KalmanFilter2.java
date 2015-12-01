/*
 * Code adapted from
 * http://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?article=1114&context=aerosp
 */
public class KalmanFilter2 {
	
	private long timer;
	private int countInitSteps = 0;
	
	private float Gyro_cal_x_sample = 0;
	private float Gyro_cal_y_sample = 0;
	private float Gyro_cal_z_sample = 0;
	private float Accel_cal_x_sample = 0;
	private float Accel_cal_y_sample = 0;
	private float Accel_cal_z_sample = 0;
	private float Gyro_cal_x = 0;
	private float Gyro_cal_y = 0;
	private float Gyro_cal_z = 0;
	private float Accel_cal_x = 0;
	private float Accel_cal_y = 0;
	private float Accel_cal_z = 0;
	
	// ---- Kalman Variables ----
	float Gyro_pitch = 0; 			//Initialize value
	float Accel_pitch = 0; 			//Initial Estimate
	float Predicted_pitch = 0; 		//Initial Estimate
	float Gyro_roll = 0; 			//Initialize value
	float Accel_roll = 0; 			//Initialize value
	float Predicted_roll = 0; 		//Output of Kalman filter
	float Q = 0.1f; 				// Prediction Estimate Initial Guess
	float R = 5; 					// Prediction Estimate Initial Guess
	float P00 = 0.1f; 				// Prediction Estimate Initial Guess
	float P11 = 0.1f; 				// Prediction Estimate Initial Guess
	float P01 = 0.1f; 				// Prediction Estimate Initial Guess
	float Kk0, Kk1;
	float Comp_pitch;
	float Comp_roll;
	
	public KalmanFilter2(){
		
		
	}
	
	public void update(float gx, float gy, float gz, float ax, float ay, float az, float dt){
		if (countInitSteps < 100) {
			initStep(gx, gy, gz, ax, ay, az);
			countInitSteps++;
			if (countInitSteps == 100){
				calculateMedian();
				System.out.print("Fully initialized");
			}
		} 
		else {
			kalmanStep(gx, gy, gz, ax, ay, az, dt);
		}
	}
	
	private void calculateMedian(){
		 Gyro_cal_x = Gyro_cal_x_sample / 100;
		 Gyro_cal_y = Gyro_cal_y_sample / 100;
		 Gyro_cal_z = Gyro_cal_z_sample / 100;
		 Accel_cal_x = Accel_cal_x_sample / 100;
		 Accel_cal_y = Accel_cal_y_sample / 100;
		 Accel_cal_z = (Accel_cal_z_sample / 100) - 256; // Raw Accel output in z-direction is in 256 LSB/g due to gravity
	}
	
	private void initStep(float gx, float gy, float gz, float ax, float ay, float az){
		 Gyro_cal_x_sample += gx;
		 Gyro_cal_y_sample += gy;
		 Gyro_cal_z_sample += gz;
		 Accel_cal_x_sample += ax;
		 Accel_cal_y_sample += ay;
		 Accel_cal_z_sample += az;
	}
	
	private void kalmanStep(float gx, float gy, float gz, float ax, float ay, float az, float dt){

		 Accel_pitch = (float) (Math.atan2((ay - Accel_cal_y) / 256, (az - Accel_cal_z) / 256) * 180 / Math.PI);
		 Gyro_pitch = (float) (Gyro_pitch + ((gx - Gyro_cal_x) / 14.375) * dt);
		 //if(Gyro_pitch<180) Gyro_pitch+=360; // Keep within range of 0-180 deg to match Accelerometer output
		 //if(Gyro_pitch>=180) Gyro_pitch-=360;
		 Comp_pitch = Gyro_pitch;
		 Predicted_pitch = (float) (Predicted_pitch + ((gx - Gyro_cal_x) / 14.375) * dt); 							// Time Update step 1
		 Accel_roll = (float) (Math.atan2((ax - Accel_cal_x) / 256, (az - Accel_cal_z) / 256) * 180 / Math.PI);
		 Gyro_roll = (float) (Gyro_roll - ((gy - Gyro_cal_y) / 14.375) * dt);
		 //if(Gyro_roll<180) Gyro_roll+=360; // Keep within range of 0-180 deg
		 //if(Gyro_roll>=180) Gyro_roll-=360;
		 Comp_roll = Gyro_roll;
		 Predicted_roll = (float) (Predicted_roll - ((gy - Gyro_cal_y) / 14.375) * dt); 							// Time Update step 1
		 P00 += dt * (2 * P01 + dt * P11);																			// Projected error covariance terms from derivation result: Time Update step 2
		 P01 += dt * P11; 																							
		 P00 += dt * Q; 																							
		 P11 += dt * Q; 																							
		 Kk0 = P00 / (P00 + R); 																					// Measurement Update step 1
		 Kk1 = P01 / (P01 + R); 																	
		 Predicted_pitch += (Accel_pitch - Predicted_pitch) * Kk0; 													// Measurement Update step 2
		 Predicted_roll += (Accel_roll - Predicted_roll) * Kk0; 
		 P00 *= (1 - Kk0); 																							// Measurement Update step 3
		 P01 *= (1 - Kk1); 
		 P11 -= Kk1 * P01; 
		 float alpha = 0.98f;
		 Comp_pitch = (float) (alpha*(Comp_pitch+Comp_pitch*dt) + (1.0 - alpha)*Accel_pitch); 						// Complimentary filter
		 Comp_roll = (float) (alpha*(Comp_roll+Comp_roll*dt) + (1.0 - alpha)*Accel_roll); 
		 
		 System.out.print(Gyro_pitch);
		 System.out.print("\t");
		 System.out.print(Accel_pitch);
		 System.out.print("\t");
		 System.out.print(Comp_pitch);
		 System.out.print("\t");
		 System.out.print(Predicted_pitch);
		 System.out.print("\t");
		 System.out.print(Gyro_roll);
		 System.out.print("\t");
		 System.out.print(Accel_roll);
		 System.out.print("\t");
		 System.out.print(Comp_roll);
		 System.out.print("\t");
		 System.out.print(Predicted_roll);
		 System.out.print("\n");
	}

}
