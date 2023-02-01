// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick; //imports the libary for joystick control
import com.revrobotics.CANSparkMax; //imports the libary for Spark Max control over CAN
import com.revrobotics.CANSparkMaxLowLevel; //additional functionality for Spark motor controllers
import com.revrobotics.SparkMaxPIDController; //allows for motor controller PID control
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //allows for brushless motor control
import com.revrobotics.RelativeEncoder; //allows the NEO ecoders to be used
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //enables the smart dashboard
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private RelativeEncoder armEncoder; //declares the encoder on the arm
 public boolean calState; //stores the state of caliberation
  
  final Joystick leftStick = new Joystick(0); //creates a new joystick named leftStick and conects it to USB port 0
  final CANSparkMax armMotor = new CANSparkMax(6, MotorType.kBrushless); //creates and names a motor controller CAN ID 6 and makes it brushless
 DigitalInput calSwitch = new DigitalInput(0); //attaches a switch to DIO 0
 private SparkMaxPIDController armPID; //creates a PID controller for the arm
 public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput; //creates vairables to store PID values in 

  @Override
  public void robotInit() {
   armEncoder = armMotor.getEncoder(); //attaches the arm encoder to the armMotor motor
   armMotor.restoreFactoryDefaults(); //resets the defaults for the armMotor motor controller for PID
   armPID = armMotor.getPIDController(); //attaches arm PID settings to the armMotor controller
   calState=false; //sets in the initial caliberation state to false
 
   //PID Coefficients
   kP = 0.1; 
   kI = 0;
   kD = 0; 
   kIz = 0; 
   kFF = 0; 
   kMaxOutput = 1; 
   kMinOutput = -1;

//Set PID Coefficients, can combine this with above if you don't need the variables
armPID.setP(kP);
    armPID.setI(kI);
    armPID.setD(kD);
    armPID.setIZone(kIz);
    armPID.setFF(kFF);
    armPID.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void robotPeriodic() {
SmartDashboard.putNumber("Position", armEncoder.getPosition()); //puts the postion on the motor (in revolutions) on the dashboard
SmartDashboard.putBoolean("Calibration State", calState); //displays the state of caliberation
SmartDashboard.putBoolean("Switch Output", calSwitch.get()); //displays the real time state of the calibration swtich



  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    while(calState==false){ //if the robot is not caliberated
      armMotor.set(.2); //run the motor into the caliberation switch
      if(calSwitch.get()==true){ //when the switch is pressed
        calState=true; //set the calibiration state to true
        armMotor.set(0); //stop the motor
        armEncoder.setPosition(0); //set the encoder to 0
      }
  }
  
}
  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
if(leftStick.getRawButton(1)==true){
  armPID.setReference(-20, CANSparkMax.ControlType.kPosition);
} 
else if(leftStick.getRawButton(2)==true){
  armPID.setReference(-50, CANSparkMax.ControlType.kPosition); 
}
else{
  armPID.setReference(-30, CANSparkMax.ControlType.kPosition); 
}
}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
public void testInit() {
}
  @Override
  public void testPeriodic() {
    armMotor.set(-leftStick.getY());

  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
