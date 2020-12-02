/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.nio.ByteBuffer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorWheelConstants;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class ColorWheelSubsystem extends SubsystemBase {
 /**
  * Creates a new ExampleSubsystem.
  */
  
  private final WPI_TalonFX colorWheel;
  private ColorSensorV3 colorSensor;
  private DigitalInput touchSensor = new DigitalInput(0);
  //private final WPI_TalonSRX slideMotor;
  
  
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  

  public ColorWheelSubsystem() {

    this.colorWheel = new WPI_TalonFX(ColorWheelConstants.kColorWheelPort);
    // this.slideMotor = new WPI_TalonSRX(ColorWheelConstants.kSlideMotorPort);
    this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard); //todo: find sensor I2C address
    this.colorMatcher.addColorMatch(this.kBlueTarget);
    this.colorMatcher.addColorMatch(this.kGreenTarget);
    this.colorMatcher.addColorMatch(this.kRedTarget);
    this.colorMatcher.addColorMatch(this.kYellowTarget);    
        
    }

  public String getColor() {
    Color detectedColor = this.colorSensor.getColor();

    String colorString;
    ColorMatchResult match = this.colorMatcher.matchClosestColor(detectedColor);
    
    if (match.color == this.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == this.kRedTarget) {
      colorString = "Red";
    } else if (match.color == this.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == this.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

      return colorString;
    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = this.colorSensor.getColor();

    String colorString;
    ColorMatchResult match = this.colorMatcher.matchClosestColor(detectedColor);
  
    if (match.color == this.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == this.kRedTarget) {
      colorString = "Red";
    } else if (match.color == this.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == this.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    }

  public void setWheel(double power) {
      this.colorWheel.set(power);
  }
  public boolean getTouching() {
    return this.touchSensor.get();
  }
  public double getEncoderCount() {
      return this.colorWheel.getSensorCollection().getIntegratedSensorAbsolutePosition();
  }
  public void resetEncoders() {
      this.colorWheel.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }
  public void spin() {
      this.colorWheel.set(ColorWheelConstants.kColorWheelPower);
  }
  /*public void stop() {
    this.colorWheel.set(0);
    slideMotor.set(0);
  }
  public void extend() {
    slideMotor.set(ColorWheelConstants.kSlideMotorPower);
  }
  public void retract() {
    slideMotor.set(-ColorWheelConstants.kSlideMotorPower);
  }
  public void safeExtend() {
    while(!getTouching()) {
      extend();
    }
    stop();
  }
  public void safeRetract() {
    while(!getTouching()) {
      retract();
    }
    stop();
  }
  //will spin to a color so that the correct color is lined up with the game color sensor
  public void spinToColor(String color) {
    if(color.equals("Red")) {
      while(!getColor().equals("Blue")) {
          spin();
      }
      stop();
    }
    else if(color.equals("Blue")) {
      while(!getColor().equals("Red")) {
        spin();
      }
      stop();
    }
    else if(color.equals("Green")) {
      while(!getColor().equals("Yellow")) {
        spin();
      }
      stop();
    }
    else if(color.equals("Yellow")) {
      while(!getColor().equals("Green")) {
        spin();
      }
      stop();
    }
  }
    
  public void spinAmount(int times) {
    String color = getColor();
    boolean countAgain=false;
    int passCount = 0;
    spin();
    while(passCount<times*2) {
      if(!getColor().equals(color)) {
        countAgain=true;
      }
      else{
        if(countAgain) {
          passCount++;
          countAgain=false;
        }
      }
    }
    stop();
  }*/
}
