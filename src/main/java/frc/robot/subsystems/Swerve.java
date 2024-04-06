package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveMods;
    public final Pigeon2 gyro;
    final Field2d field = new Field2d();
    private final SysIdRoutine m_sysIdRoutine;
     // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
     private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
     // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
     private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
     // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
     private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
 
    // public ADXRS450_Gyro gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();
        
        // gyro = new ADXRS450_Gyro();
        // gyro.calibrate(); 

        SmartDashboard.putData("Field", field);

        swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS),
            new SwerveModule(1, Constants.Swerve.Mod1.SWERVE_MODULE_CONSTANTS),
            new SwerveModule(2, Constants.Swerve.Mod2.SWERVE_MODULE_CONSTANTS),
            new SwerveModule(3, Constants.Swerve.Mod3.SWERVE_MODULE_CONSTANTS)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */


         
        Timer.delay(1.0);
        for (SwerveModule module : swerveMods) {
            module.resetToAbsolute();
        }

        swerveOdometry = new SwerveDriveOdometry(
                Constants.Swerve.SWERVE_KINEMATICS,
                getHeading(),
                getModulePositions()
        );

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier.
                                                                                             // MUST BE ROBOT RELATIVE
                speeds -> {
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    SwerveModuleState[] swerveModuleStates =
                            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
                    setModuleStates(swerveModuleStates);
                },
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants( // Translation PID constants
                                Constants.Auto.AUTO_DRIVE_P,
                                Constants.Auto.AUTO_DRIVE_I,
                                Constants.Auto.AUTO_DRIVE_D
                        ),
                        new PIDConstants( // Rotation PID constants
                                Constants.Auto.AUTO_ANGLE_P,
                                Constants.Auto.AUTO_ANGLE_I,
                                Constants.Auto.AUTO_ANGLE_D
                        ),
                        Constants.Swerve.MAX_SPEED -1, // Max module speed, in m/s
                        Constants.Swerve.CENTER_TO_WHEEL, // Drive base radius in meters. Distance from robot center to
                                                          // furthest module.
                        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                  },
                  this // Reference to this subsystem to set requirements
        );
        m_sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motors.
                (Measure<Voltage> volts) -> {
                    for (SwerveModule mod : swerveMods) {
                        mod.setMotor(volts.in(Volts));
                    }
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the left motors.  Since these share an encoder, we consider
                    // the entire group to be one motor.
                    log.motor("front-left?")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                swerveMods[2].getMotor(swerveMods[2]).getAppliedOutput()*swerveMods[2].getMotor(swerveMods[2]).getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(swerveMods[2].getPosition().distanceMeters, Meters))
                        .linearVelocity(
                            m_velocity.mut_replace(swerveMods[2].getMotor(swerveMods[2]).getEncoder().getVelocity(), MetersPerSecond));

                  
                    log.motor("front-right?")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                               swerveMods[3].getMotor(swerveMods[3]).getAppliedOutput()*swerveMods[3].getMotor(swerveMods[3]).getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(swerveMods[3].getPosition().distanceMeters, Meters))
                        .linearVelocity(
                            m_velocity.mut_replace(swerveMods[3].getMotor(swerveMods[3]).getEncoder().getVelocity(), MetersPerSecond));
                    
                            log.motor("back-left?")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                               swerveMods[0].getMotor(swerveMods[0]).getAppliedOutput()*swerveMods[0].getMotor(swerveMods[0]).getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(swerveMods[0].getPosition().distanceMeters, Meters))
                        .linearVelocity(
                            m_velocity.mut_replace(swerveMods[0].getMotor(swerveMods[0]).getEncoder().getVelocity(), MetersPerSecond));

                    log.motor("back-right?")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                               swerveMods[1].getMotor(swerveMods[1]).getAppliedOutput()*swerveMods[1].getMotor(swerveMods[1]).getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(swerveMods[1].getPosition().distanceMeters, Meters))
                        .linearVelocity(
                            m_velocity.mut_replace(swerveMods[1].getMotor(swerveMods[1]).getEncoder().getVelocity(), MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name ("drive")
                this));
    }
    

    @Override
    public void periodic() {
        swerveOdometry.update(getHeading(), getModulePositions());
        field.setRobotPose(swerveOdometry.getPoseMeters());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Absolute", mod.getAbsoluteAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Relative", mod.getAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Position", mod.getPosition().distanceMeters);
        }
    }

    public Field2d getField2d() {
        return field;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading()
                ) : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation
                )
        );
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    public double getYaw() {
        return (Constants.Swerve.INVERT_GYRO) ?
                Constants.MAXIMUM_ANGLE - (gyro.getAngle()) :
                gyro.getAngle();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

        public double distanceMod1(){
            SwerveModule mod1 =
        swerveMods[0];
            return
        mod1.getPosition().distanceMeters;
    }

            public double distanceMod2(){
            SwerveModule mod2 =
        swerveMods[0];
            return
        mod2.getPosition().distanceMeters;
    }

            public double distanceMod3(){
            SwerveModule mod3 =
        swerveMods[0];
            return
        mod3.getPosition().distanceMeters;
    }

            public double distanceMod4(){
            SwerveModule mod4 =
        swerveMods[0];
            return
        mod4.getPosition().distanceMeters;
    }

    // TODO: FIX SWERVE SCUFFEDNESS
    public void zeroGyro() {
        gyro.reset();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void setgyro(){
        //fill later
    }
     public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}