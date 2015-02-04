import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;


public class InverseKinematics {
	
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);

    static double armLenC = (8*29);//in mm
    static double armLenF = (8*14);// in mm

	public static void main(String[] args) {
//		int temp = 100;
//		while(temp < 800){
//			goHere(temp, 200);
//			temp+= 100;
//		}
//		int[]angles =goHere(100, -50);
//		motorFar.rotate(angles[1]);
//		motorClose.rotate(angles[0]);
//		Button.waitForAnyPress();
		midPoint();
	}
	
	
	public static int[] goHere(double ye, double xe){
		
		double d, theta1, theta2;
		
		xe += (armLenC+armLenF);
		
		//translate to zero, zero
		
		d = ((xe*xe)+(ye*ye)-(armLenC*armLenC)-(armLenF*armLenF))/(2*armLenC*armLenF);
		
		theta2 = Math.atan2(Math.sqrt(1-(d*d)), d);
		theta1 = Math.atan2(ye, xe) - Math.atan2(armLenF*Math.sin(theta2),(armLenC+(armLenF*Math.cos(theta2))));

		theta1 = theta1*180/Math.PI;
		theta2 = theta2*180/Math.PI;
		
//		System.out.println("d: "+d);
//		System.out.println("y: "+ye);
		System.out.println(theta1);
		System.out.println(theta2);
		
		int[] solution = {(int)theta1*7,(int)theta2};
		return solution;
		
	}
	
	
	public static void midPoint(){

		double d, theta1, theta2, midx, midy, mvx, mvy;
		TwoDOFAnalyticalSolution first = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		TwoDOFAnalyticalSolution second = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		
		System.out.println(motorFar.getRotationSpeed());
		Button.waitForAnyPress();

		first.setThetaC(motorClose.getTachoCount()/7);
		first.setThetaF(motorFar.getTachoCount());

		System.out.println("X=" + first.getXe());
		System.out.println("Y=" + first.getYe());
		
        System.out.println("Press to Record 2");
		Button.waitForAnyPress();
		
		second.setThetaC(motorClose.getTachoCount()/7);
		second.setThetaF(motorFar.getTachoCount());
		
		midx = (first.getXe() + second.getXe()) /2;
		midy = (first.getYe() + second.getYe()) /2;
		
		int[]angles =goHere(midy, midx-(armLenC+armLenF));

		motorFar.setSpeed(30);
		
		motorFar.rotateTo(0);
		motorClose.rotateTo(0);
		
		Button.waitForAnyPress();
		
		motorFar.rotateTo(angles[1]);
		motorClose.rotateTo(angles[0]);
		
		System.out.println("X=" + (int)angles[1]);
		System.out.println("Y=" + (int)angles[0]);
		Button.waitForAnyPress();
		
	}
}
