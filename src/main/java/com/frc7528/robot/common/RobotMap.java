package com.frc7528.robot.common;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotMap {
    //Drivetrain Motor Controllers
    public static WPI_TalonFX m_leftAft = new WPI_TalonFX(1);
    public static WPI_TalonFX m_leftFront = new WPI_TalonFX(2);
    public static WPI_TalonFX m_rightAft = new WPI_TalonFX(3);
    public static WPI_TalonFX m_rightFront = new WPI_TalonFX(4);

    //Conveyor Belt Motor Controllers
    public static TalonSRX ConveyorMotor1 = new TalonSRX(7);
    public static TalonSRX ConveyorMotor2 = new TalonSRX(15);

    //Control Panel Controller
    public static WPI_TalonSRX controlPanelWheel = new WPI_TalonSRX(0);
    
    //Turret Motor Controller
    public static TalonSRX Turret = new TalonSRX(51); // This should be turret 20

    //FlyWheel Motor Controllers
    public static VictorSPX FlyL_Aft = new VictorSPX(9);
    public static TalonSRX FlyR_Front = new TalonSRX(50);

    //Drive Train Class
    public static DifferentialDrive m_drive;

    //Joystick
    public static Joystick m_joy = new Joystick(0);

    //Pigeon IMU
    public static WPI_PigeonIMU  pidgey = new WPI_PigeonIMU(ConveyorMotor2);

    public static double heading;
    public static double kP = 1;
    public static double testPrint1 = pidgey.getAngle();
    public static double testPrint2 = pidgey.getCompassHeading();
    public static double testPrint3 = pidgey.getYaw();

    //LimeLight 
    public static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    //public static TalonSRX C50 = new TalonSRX(50);
    //public static VictorSPX C14 = new VictorSPX(14);
    
    //public static TalonSRX C51 = new TalonSRX(51);
}
