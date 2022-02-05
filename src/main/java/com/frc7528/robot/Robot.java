//https://docs.wpilib.org/en/stable/index.html for documentation

/*
Running the program:
press ctrl + shift + p
look up "WPILib: Simulate Robot Code on Desktop"
press "halsim_gui.dll"
assign desired system joystick to virtual joystick
click "Map Gamepad"
click "Teleoperated"
*/

package com.frc7528.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.io.File;
import java.text.SimpleDateFormat;
//import java.util.List;
import com.ctre.phoenix.motorcontrol.ControlMode;

import static com.frc7528.robot.common.RobotMap.*;


//imports from wpilib and RobotMap class

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends TimedRobot {
		
		private SendableChooser<Double> fineControlSpeed = new SendableChooser<>();
		private SendableChooser<Double> deadBandOptions = new SendableChooser<>();
		private double fineControlSpeedDouble;
		private SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yy HH:mm:ss");
		//private NetworkTableEntry ledStatusEntry = Shuffleboard.getTab("DRIVETRAIN").add("LED Status", "OFF").getEntry();
		private final Drivetrain m_drivetrain = new Drivetrain();
		
		private double conveyor1;
		private double conveyor2;
		private double Tmove;
		private double RAft;
		private double LAft;
		private boolean found;
		UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);
		MjpegServer switchCam = CameraServer.getInstance().addSwitchedCamera("Camera");

		//UsbCamera cam = new UsbCamera(, path)
		//CvSink cvSink = cam.getVideo();
		
		//private final RamseteController m_ramsete = new RamseteController();
		//private final Timer m_timer = new Timer();
		//private Trajectory m_trajectory;
		//private final XboxController m_controller = new XboxController(0);
		//private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
		//private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
		//creates neccessary objects to run program

		private boolean isInverted = true;
		
		//Configure Victors, SendableChoosers, and initial debug statistics
		@Override
		public void robotInit() {
				File file = new File(Robot.class.getProtectionDomain().getCodeSource().getLocation().getPath());
				Shuffleboard.getTab("DEBUG").add("Left Aft Drivetrain Firm",m_leftAft.getFirmwareVersion());
				Shuffleboard.getTab("DEBUG").add("Left Front Drivetrain Firm",m_leftFront.getFirmwareVersion());
				Shuffleboard.getTab("DEBUG").add("Right Aft Drivetrain Firm",m_rightAft.getFirmwareVersion());
				Shuffleboard.getTab("DEBUG").add("Right Front Drivetrain Firm",m_rightFront.getFirmwareVersion());
				Shuffleboard.getTab("DEBUG").add("Last code deploy",sdf.format(file.lastModified()));
					
				//Format all motor controllers
				m_leftAft.configFactoryDefault();
				m_leftFront.configFactoryDefault();
				m_rightAft.configFactoryDefault();
				m_rightFront.configFactoryDefault();
				ConveyorMotor1.configFactoryDefault();
				ConveyorMotor2.configFactoryDefault();
				Turret.configFactoryDefault();

				FlyL_Aft.configFactoryDefault(); //change this
				FlyR_Front.configFactoryDefault();
				//Config followers
				m_leftAft.follow(m_leftFront);
				m_rightAft.follow(m_rightFront);
				
				//Config inversion
				m_leftFront.setInverted(false);
				m_rightFront.setInverted(false);

				//C14.configFactoryDefault();
				//C9.configFactoryDefault();
				
				//Instantiate DifferentialDrive and put it on Shuffleboard
				m_drive = new DifferentialDrive(m_leftFront,m_rightFront);
				Shuffleboard.getTab("DRIVETRAIN").add(m_drive);
				
				//Put Limelight LED Status to Shuffleboard
				//ledStatusEntry.setString("OFF");
				
				//Fine Control Speed chooser
				fineControlSpeed.addOption("35% Speed", 0.35);
				fineControlSpeed.addOption("40% Speed", 0.40);
				fineControlSpeed.setDefaultOption("45% Speed", 0.45);
				fineControlSpeed.addOption("50% Speed", 0.50);
				fineControlSpeed.addOption("55% Speed", 0.55);
				fineControlSpeed.addOption("60% Speed", 0.60);
				Shuffleboard.getTab("SETUP").add("Fine Control Speed", fineControlSpeed);
				
				//Deadband chooser
				deadBandOptions.setDefaultOption("5%", 0.05);
				deadBandOptions.addOption("10%", 0.10);
				deadBandOptions.addOption("15%", 0.15);
				Shuffleboard.getTab("SETUP").add("Dead Band", deadBandOptions);
				
				//Transmits video through cameras
				//CameraServer.startAutomaticCapture();

				// Flush NetworkTables every loop. This ensures that robot pose and other values
				// are sent during every iteration.
				setNetworkTablesFlushEnabled(true);
				//List<Translation2d> waypoints = new ArrayList<Translation2d>();
				//Generates trajectory for the display using the starting pose, given waypoints, and a trajectory config
				//m_trajectory = TrajectoryGenerator.generateTrajectory(
								//new Pose2d(2, 2, new Rotation2d()),
								
								//List.of(new Translation2d(3,3), new Translation2d(4,5), new Translation2d(5,4), new Translation2d(6,6)),
								//new Pose2d(4, 3, new Rotation2d()),
								//new TrajectoryConfig(2, 2));
				
		}

		//Sets fine control speed and deadband
		@Override
		public void teleopInit() {
				fineControlSpeedDouble = -fineControlSpeed.getSelected(); //Set fine control speed
				m_drive.setDeadband(deadBandOptions.getSelected()); //Set deadband
				limelightTable.getEntry("ledMode").setNumber(0);
				System.out.println("yo");
		}

		//Teleop driving (Fine control and joystick control)
		@Override
		public void teleopPeriodic(){
			Tmove = 0;
			if (m_joy.getRawButton(11)){
				limelightTable.getEntry("ledMode").setNumber(1);
				found = false;
			}
				if (m_joy.getRawButton(12)){
					System.out.print(limelightTable.getEntry("tx").getDouble(0));
					System.out.println(" - tx");
					System.out.print(limelightTable.getEntry("ta").getDouble(0));
					System.out.println("- ta");
					//System.out.print(limelightTable.getEntry("tv").getDouble(0));
					//System.out.println("- tv");
					limelightTable.getEntry("ledMode").setNumber(3);
					
					if (limelightTable.getEntry("tx").getDouble(0)>0.0){
						Tmove = -0.2;
					}
					else if (limelightTable.getEntry("tx").getDouble(0)<0.0){
						Tmove = 0.2;
					}
					//positive is right

	
					if (limelightTable.getEntry("tv").getDouble(0)==1.0){
						//m_drive.arcadeDrive(0.1, 0);
						found = true;
						System.out.println("hey");
						System.out.println();
					}
					else if(limelightTable.getEntry("tv").getDouble(0)==0)
					{
						found = false;
					}
					if (found)
					{
						FlyR_Front.set(ControlMode.PercentOutput, 0.5);
					}
				}

				
				//conveyor
				conveyor1 = 0.0;
				if (m_joy.getTrigger()){
					conveyor1 = -0.5;
				}
				conveyor2 = 0.0;
				if (m_joy.getTrigger()){
					conveyor2 = -0.5;
				}
				
				//Turret
				if (m_joy.getRawButton(6)){
					Tmove = -0.2;
				}
				if (m_joy.getRawButton(5)){
					Tmove = 0.2;
				}
				

				//Flywheel
				RAft = 0;
				LAft = 0; //Laft
				if (m_joy.getRawButton(4)){
					RAft = -0.1;
					RAft = m_joy.getThrottle()*-1;
				}
				if (m_joy.getRawButton(3)){
					LAft = -0.1; //LAFT
					LAft = m_joy.getThrottle()*-1; //Laft

				}
				
				Turret.set(ControlMode.PercentOutput, Tmove);

				ConveyorMotor2.set(ControlMode.PercentOutput, conveyor2);
				ConveyorMotor1.set(ControlMode.PercentOutput, conveyor1);

				FlyL_Aft.set(ControlMode.PercentOutput, LAft);////////FlyL_Aft not C9
				FlyR_Front.set(ControlMode.PercentOutput, RAft);
				
				
				


				//Fine control
				if (m_joy.getPOV() == 0) { //Forward
						m_drive.arcadeDrive(fineControlSpeedDouble,0);
				} else if (m_joy.getPOV() == 90) { //Right
						m_drive.arcadeDrive(0,-fineControlSpeedDouble);
				} else if (m_joy.getPOV() == 180) { //Backward
						m_drive.arcadeDrive(-fineControlSpeedDouble,0);
				} else if (m_joy.getPOV() == 270) { //Left
						m_drive.arcadeDrive(0,fineControlSpeedDouble);
				} else {

						//Curvature Drive
						if (isInverted) {
								m_drive.curvatureDrive(m_joy.getY(), m_joy.getX(), m_joy.getRawButton(2));
						} else {
								m_drive.curvatureDrive(-m_joy.getY(), m_joy.getX(), m_joy.getRawButton(2));
						}
				}
		}

	//Runs Drivetrain class periodically
	@Override
	public void robotPeriodic() {
		m_drivetrain.periodic();
	}

	//resets and starts timer, and sets initial pose of robot in the simulator
	/*
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
	}

	//gets previously made timer to make sure time is the same. 
	//returns the trajectory at time specified and 
	public void autonomousPeriodicSim() {
		double elapsed = m_timer.get();
		Trajectory.State reference = m_trajectory.sample(elapsed);
		ChassisSpeeds speeds = m_ramsete.calculate(m_drivetrain.getPose(), reference);
		m_drivetrain.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
	}
	

	//Gets data from controller to use during simulated teleop
	@SuppressWarnings("LocalVariableName")
	public void teleopPeriodicSim() {
		// Get the x speed from a controller
		double xSpeed =
				-m_speedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;

		// Get the rate of angular rotation from controller
		double rot =
				-m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;
		m_drivetrain.drive(xSpeed, rot);
	}

	//Runs Drivetrain class periodically in simulation
	@Override
	public void simulationPeriodic() {
		m_drivetrain.simulationPeriodic();
	}
	*/
}