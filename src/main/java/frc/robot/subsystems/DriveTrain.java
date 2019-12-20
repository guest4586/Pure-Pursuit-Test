
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utils.pure_pursuite.*;
import frc.robot.commands.DriveArcade;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  //the robot it self 
  public WPI_TalonSRX leftLeader;
  public WPI_TalonSRX rightLeader;
  
  
  private WPI_VictorSPX leftFollower;
  private WPI_VictorSPX rightFollower;

  private SpeedControllerGroup leftController;
  private SpeedControllerGroup rightController;
  
  public  DifferentialDrive diffDrive;

  private AHRS Gyro;

  //for pure pursuit in the code its called position or odometry
  private int lastR,lastL;
  private Odometry position;

  private NetworkTable Table;
  private NetworkTableInstance NetInst;


  private static DriveTrain Instance;
  private DriveTrain(){
    super("DriverTrain");
    // initalize everything
    this.leftLeader = new WPI_TalonSRX(RobotMap.DriveI.LEFT_LEADER);
    this.rightLeader = new WPI_TalonSRX(RobotMap.DriveI.RIGHT_LEADER);

    this.leftFollower = new WPI_VictorSPX(RobotMap.DriveI.LEFT_FOLLOWER);
    this.rightFollower = new WPI_VictorSPX(RobotMap.DriveI.RIGHT_FOLLOWER);

    this.leftFollower.follow(this.leftLeader);
    this.rightFollower.follow(this.rightLeader);

    this.leftLeader.setNeutralMode(NeutralMode.Brake);
    this.rightLeader.setNeutralMode(NeutralMode.Brake);
    this.leftFollower.setNeutralMode(NeutralMode.Brake);
    this.rightFollower.setNeutralMode(NeutralMode.Brake);

    this.leftController = new SpeedControllerGroup(this.leftLeader,this.leftFollower);
    this.rightController = new SpeedControllerGroup(this.rightLeader,this.rightFollower);

    // this.diffDrive = new DifferentialDrive(this.leftController,this.rightController);

    this.Gyro = new AHRS(SPI.Port.kMXP);
    position = Odometry.getInstance();
    Instance = null;
    this.NetInst = NetworkTableInstance.getDefault();
    this.Table = NetInst.getTable("DriveTrain");
    // this is were the real code starts.
  }

  public double getRightEncoderVelocityInM(){return this.rightLeader.getSelectedSensorVelocity()*0.0005829;}
  public double getLeftEncoderVelocityInM(){return this.leftLeader.getSelectedSensorVelocity()*0.0005829;}

  public double getRightEncoderVelocity(){return this.rightLeader.getSelectedSensorVelocity();}
  public double getLeftEncoderVelocity(){return this.leftLeader.getSelectedSensorVelocity();}


  public int getLeftEncoderPosition(){
    return this.leftLeader.getSelectedSensorPosition();
  }

  public int getRightEncoderPosition(){
    return -this.rightLeader.getSelectedSensorPosition();
  }
  public void setLeftSpeed(double speed){
    if(Math.abs(speed)<0.2)
      this.leftLeader.set(speed);
  }
  public void setRightSpeed(double speed){
    if(Math.abs(speed)<0.2)
      this.rightLeader.set(speed);
  }
  public double getAngle(){
    return this.Gyro.getAngle();
  }

  public double getLeftPO(){return this.leftLeader.get();}

  public void setLPO(double demand){
    this.leftLeader.set(ControlMode.PercentOutput, demand);
  }

  public void setRPO(double demand){
    this.rightLeader.set(ControlMode.PercentOutput, demand);
  }


  @Override
  public void periodic() {
    this.Table..putNumber("left posi", this.getLeftEncoderPosition());
    this.Table..putNumber("right posi", this.getRightEncoderPosition());
    this.Table..putNumber("X pos",this.position.x);
    this.Table..putNumber("Y pos",this.position.y);
    this.Table..putNumber("gyro",getAngle());    
  }


  public void resetOdometry(){
    this.position.Reset();
    this.leftLeader.setSelectedSensorPosition(0);
    this.rightLeader.setSelectedSensorPosition(0);
    lastL = 0;lastR = 0;
    resetGyro(); 
  }
  public void resetGyro(){
    this.Gyro.reset();
  }

  public void UpdateOdometry(){
    int deltaR,deltaL;
    deltaL = getLeftEncoderPosition() - this.lastL;
    deltaR = getRightEncoderPosition() - this.lastR;
    this.lastL = getLeftEncoderPosition();
    this.lastR = getRightEncoderPosition();
    
    this.position.UpdateRobotPosition((double)deltaL, (double)deltaR, getAngle());
    Robot.x.setDouble(this.position.x);
    Robot.y.setDouble(this.position.y);
    Robot.angle.setDouble(getAngle());
    
  }
  
  public double encoderToMeters(int in){
    return 0.152*Math.PI*in/8196;
  }

  public Vector getRobotOdometry(){
    return this.position.getRobotPosition();
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new DriveArcade());
  }

  public void ArcadeDrive(final double rotation, final double speed){
    this.diffDrive.arcadeDrive(-speed, rotation);
  }

  
  public static DriveTrain getInstance(){
    if(Instance == null){
      Instance = new DriveTrain();
    }
    return Instance;
  }
}
