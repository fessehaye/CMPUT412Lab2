
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;


public class ForwardKinematics {
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);

    static double armLenC = (8*29);//in mm
    static double armLenF = (8*14);// in mm

	public static void main(String[] args) {

		getPosition(90,0);
//		distanceMeasure();
		
	}
	
	public static void getPosition(double degC, double degF ){
		TwoDOFAnalyticalSolution first = new TwoDOFAnalyticalSolution(armLenC,armLenF);

		motorClose.rotateTo((int)degC*7);
		motorFar.rotateTo((int)degF);

		first.setThetaC(motorClose.getTachoCount()/7);
		first.setThetaF(motorFar.getTachoCount());
		
        System.out.println("x: " + (int)(first.getXe()-(armLenC+armLenF)) + "mm");
        System.out.println("y: " + (int)first.getYe() + "mm");
	}
	
	public static void distanceMeasure(){
		
		TwoDOFAnalyticalSolution first = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		TwoDOFAnalyticalSolution second = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		TwoDOFAnalyticalSolution third = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		double d1, d2, d3, a1, a2, a3, xd, yd;

        System.out.println("Press to Record 1");
		Button.waitForAnyPress();

		first.setThetaC(motorClose.getTachoCount()/7);
		first.setThetaF(motorFar.getTachoCount());

        System.out.println("Press to Record 2");
		Button.waitForAnyPress();
		
		second.setThetaC(motorClose.getTachoCount()/7);
		second.setThetaF(motorFar.getTachoCount());
		
	System.out.println("Press to Record 3");
		Button.waitForAnyPress();
		
		third.setThetaC(motorClose.getTachoCount()/7);
		third.setThetaF(motorFar.getTachoCount());

		yd = second.getYe()-first.getYe();
		xd = second.getXe()-first.getXe();
		d1 = Math.sqrt((xd*xd)+(yd*yd));

		yd = third.getYe()-second.getYe();
		xd = third.getXe()-second.getXe();
		d2 = Math.sqrt((xd*xd)+(yd*yd));

		yd = first.getYe()-third.getYe();
		xd = first.getXe()-third.getXe();
		d3 = Math.sqrt((xd*xd)+(yd*yd));
		
		a3 = Math.acos(((d1*d1)-(d2*d2)-(d3*d3))/(2*d2*d3))*180/Math.PI;
		a2 = Math.acos(((d3*d3)-(d2*d2)-(d1*d1))/(2*d2*d1))*180/Math.PI;
		a1 = 180-a3-a2;

        System.out.println("1to2 " + (int)d1 + " " + (int)a1);
        System.out.println("2to3 " + (int)d2 + " " + (int)a2);
        System.out.println("3to1 " + (int)d3 + " " + (int)a3);
        System.out.println("Distance: " + (int)distance + "mm");

		Button.waitForAnyPress();
        
	}
	
	
	
	
}
