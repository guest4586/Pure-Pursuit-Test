
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Utils.pure_pursuite.*;
import frc.robot.commands.DriveArcade;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  //the robot it self 
  private WPI_TalonSRX leftLeader;
  private WPI_TalonSRX rightLeader;
  
  
  private WPI_VictorSPX leftFollower;
  private WPI_VictorSPX rightFollower;

  private SpeedControllerGroup leftController;
  private SpeedControllerGroup rightController;
  
  private  DifferentialDrive diffDrive;

  private AHRS Gyro;

  //for pure pursuit in the code its called position or odometry
  private int lastR,lastL;
  private Odometry position;

  

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

    this.diffDrive = new DifferentialDrive(this.leftController,this.rightController);

    this.Gyro = new AHRS(SPI.Port.kMXP);

    Instance = null;

    this.resetOdometry();
    

    // this is were the real code starts.
  }

  public int getLeftEncoderPosition(){
    return this.leftLeader.getSelectedSensorPosition();
  }

  public int getRightEncoderPosition(){
    return -this.rightLeader.getSelectedSensorPosition();
  }

  public double getGyro(){
    return this.Gyro.getAngle();
  }




  public void resetOdometry(){
    this.position = new Odometry(0,0);
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
    SmartDashboard.putNumber("X",this.position.x);
    SmartDashboard.putNumber("Y",this.position.y);
    
  }
  
  public double encoderToMeters(int in){
    return 0.152*Math.PI*in/8196;
  }

  public Vector getRobotOdometry(){
    return this.position.getRobotPosition();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveArcade());
  }

  public double getAngle(){
    return this.Gyro.getAngle();
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
