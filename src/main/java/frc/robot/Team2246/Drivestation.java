// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Team2246;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Drivestation extends SubsystemBase {
  private static GenericHID buttonboardA, buttonboardB;
  private static Joystick lJoy , rJoy;

  /** Creates a new Drivestation. */
  public Drivestation(
    int bbaPort, 
    int bbbPort, 
    int lJoyPort, 
    int rJoyPort
  ) {
    buttonboardA = new GenericHID(bbaPort);
    buttonboardB = new GenericHID(bbbPort);
    lJoy = new Joystick(lJoyPort);
    rJoy = new Joystick(rJoyPort);
  }

  private static double tune(double x){return x*x*Math.signum(x);}
  
  //ButtonBoard
  public final Trigger s00  = new Trigger(()->buttonboardA.getRawButton(0));
  public final Trigger s01  = new Trigger(()->buttonboardA.getRawButton(1));
  public final Trigger s02  = new Trigger(()->buttonboardA.getRawButton(2));
  public final Trigger s03  = new Trigger(()->buttonboardA.getRawButton(3));
  public final Trigger s10  = new Trigger(()->buttonboardA.getRawButton(4));
  public final Trigger s11  = new Trigger(()->buttonboardA.getRawButton(5));
  public final Trigger s12  = new Trigger(()->buttonboardA.getRawButton(6));
  public final Trigger s13  = new Trigger(()->buttonboardA.getRawButton(7));
 
  public final Trigger b00  = new Trigger(()->buttonboardB.getRawButton( 0));
  public final Trigger b01  = new Trigger(()->buttonboardB.getRawButton( 1));
  public final Trigger b02  = new Trigger(()->buttonboardB.getRawButton( 2));
  public final Trigger b03  = new Trigger(()->buttonboardB.getRawButton( 3));
  public final Trigger b10  = new Trigger(()->buttonboardB.getRawButton( 4));
  public final Trigger b11  = new Trigger(()->buttonboardB.getRawButton( 5));
  public final Trigger b12  = new Trigger(()->buttonboardB.getRawButton( 6));
  public final Trigger b13  = new Trigger(()->buttonboardB.getRawButton( 7));
  public final Trigger b20  = new Trigger(()->buttonboardB.getRawButton( 8));
  public final Trigger b21  = new Trigger(()->buttonboardB.getRawButton( 9));
  public final Trigger b22  = new Trigger(()->buttonboardB.getRawButton(10));
  public final Trigger b23  = new Trigger(()->buttonboardB.getRawButton(11));

  //Left Stick
  public final Trigger ls0  = new Trigger(()->lJoy.getRawButton(0));
  public final Trigger ls1  = new Trigger(()->lJoy.getRawButton(1));
  public final Trigger ls2  = new Trigger(()->lJoy.getRawButton(2));
  public final Trigger ls3  = new Trigger(()->lJoy.getRawButton(3));
  public final Trigger ls4  = new Trigger(()->lJoy.getRawButton(4));
  public final Trigger ls5  = new Trigger(()->lJoy.getRawButton(5));
  public final Trigger ls6  = new Trigger(()->lJoy.getRawButton(6));
  public final Trigger ls7  = new Trigger(()->lJoy.getRawButton(7));
  public final Trigger ls8  = new Trigger(()->lJoy.getRawButton(8));
  public final Trigger ls9  = new Trigger(()->lJoy.getRawButton(9));
  public final Trigger ls10 = new Trigger(()->lJoy.getRawButton(10));

  public double getLeftX(){return tune(lJoy.getX());}
  //public double getLeftY(){return limiter.calculate(tune(leftStick.getY()));}
  public double getLeftY(){
    //if(slewImposer()){
    //  return limiter.calculate(tune(leftStick.getY()));
    //}else{
    //  return tune(leftStick.getY());
    //}
    return tune(lJoy.getY());
  }
  public double getLeftSlider(){return lJoy.getThrottle();}

  //right joystick
  public final Trigger rs0  = new Trigger(()->rJoy.getRawButton(0));
  public final Trigger rs1  = new Trigger(()->rJoy.getRawButton(1));
  public final Trigger rs2  = new Trigger(()->rJoy.getRawButton(2));
  public final Trigger rs3  = new Trigger(()->rJoy.getRawButton(3));
  public final Trigger rs4  = new Trigger(()->rJoy.getRawButton(4));
  public final Trigger rs5  = new Trigger(()->rJoy.getRawButton(5));
  public final Trigger rs6  = new Trigger(()->rJoy.getRawButton(6));
  public final Trigger rs7  = new Trigger(()->rJoy.getRawButton(7));
  public final Trigger rs8  = new Trigger(()->rJoy.getRawButton(8));
  public final Trigger rs9  = new Trigger(()->rJoy.getRawButton(9));
  public final Trigger rs10 = new Trigger(()->rJoy.getRawButton(10));
  public final Trigger rs11 = new Trigger(()->rJoy.getRawButton(11));

  public final Trigger rsPOVup    = new Trigger(()->getRightPov()==0);
  public final Trigger rsPOVright = new Trigger(()->getRightPov()==90);
  public final Trigger rsPOVdown  = new Trigger(()->getRightPov()==180);
  public final Trigger rsPOVleft  = new Trigger(()->getRightPov()==270);

  public double getRightX(){return tune(rJoy.getX());}
  //public double getRightY(){return limiter.calculate(tune(rightStick.getY()));}
  public double getRightY(){
    //if(slewImposer()){
    //  return limiter.calculate(tune(rightStick.getY()));
    //}else{
    //  return tune(rightStick.getY());
    //}
    return tune(rJoy.getY());
  }
  public double getRightZ(){return tune(rJoy.getZ());}
  public double getRightSlider(){return rJoy.getThrottle();}
  public int getRightPov(){return rJoy.getPOV();}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
