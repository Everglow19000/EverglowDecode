package org.firstinspires.ftc.teamcode.everglow_library;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveModuleState;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.OTOSLocalizer;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class SwerveDrive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        public double wheelbaseWidth = 10; //inches
        public double wheelbaseLength = 10; //inches
        public double wheelDiameter = 0.096;

        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;


        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(-PARAMS.wheelbaseWidth/2, PARAMS.wheelbaseLength/2),
            new Translation2d(PARAMS.wheelbaseWidth/2, PARAMS.wheelbaseLength/2),
            new Translation2d(-PARAMS.wheelbaseWidth/2, -PARAMS.wheelbaseLength/2),
            new Translation2d(PARAMS.wheelbaseWidth/2, -PARAMS.wheelbaseLength/2)
    );
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public final LazyImu lazyImu;
    public final Localizer localizer;
    public final VoltageSensor voltageSensor;

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint;
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);


    private final SwerveModule[] modules;

    public class SwerveWheelVelConstraint implements VelConstraint {
        private final double maxWheelVel;  // meters/second

        public SwerveWheelVelConstraint(double maxWheelVel) {
            this.maxWheelVel = maxWheelVel;
        }

        @Override
        public double maxRobotVel(@NonNull Pose2dDual<Arclength> robotPose, @NonNull PosePath posePath, double v) {
            Pose2d txRobotWorld = robotPose.value().inverse();
            PoseVelocity2d robotVelWorld = robotPose.velocity().value();
            PoseVelocity2d robotVelRobot = txRobotWorld.times(robotVelWorld);
            SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(robotVelRobot.linearVel.x, robotVelRobot.linearVel.y, robotVelRobot.angVel));
            double min = -1;
            for (SwerveModuleState state : moduleStates) {
                double calculated = maxWheelVel / state.speedMetersPerSecond*39.3701;
                if (min == -1) {
                    min = calculated;
                }
                else if (calculated < min) {
                    min = calculated;
                }
            }

            return min;
        }
    }


    public SwerveDrive(HardwareMap hardwareMap, Pose2d startPose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        modules = new SwerveModule[]{
                new SwerveModule(hardwareMap, "leftFrontMotor", false, "leftFrontServo", false, "leftFrontServoEncoder", PARAMS.wheelDiameter, Motor.GoBILDA.RPM_312),
                new SwerveModule(hardwareMap, "rightFrontMotor", false, "rightFrontServo", false, "rightFrontServoEncoder", PARAMS.wheelDiameter, Motor.GoBILDA.RPM_312),
                new SwerveModule(hardwareMap, "leftBackMotor", false, "leftBackServo", false, "leftBackServoEncoder", PARAMS.wheelDiameter, Motor.GoBILDA.RPM_312),
                new SwerveModule(hardwareMap, "rightBackMotor", false, "rightBackServo", false, "rightBackServoEncoder", PARAMS.wheelDiameter, Motor.GoBILDA.RPM_312),
        };

        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        localizer = new OTOSLocalizer(hardwareMap, startPose);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        new SwerveWheelVelConstraint(PARAMS.maxWheelVel),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(powers.linearVel.x, powers.linearVel.y, powers.angVel));
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                modules[0].stopMotor();
                modules[1].stopMotor();
                modules[2].stopMotor();
                modules[3].stopMotor();

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);

            SwerveModuleState[] velocityStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(command.linearVel.x.get(0), command.linearVel.y.get(0), command.angVel.get(0)));
            SwerveModuleState[] accelerationStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(command.linearVel.x.get(1), command.linearVel.y.get(1), command.angVel.get(1)));
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            double leftFrontPower = feedforward.compute(velocityStates[0].speedMetersPerSecond*39.3701, accelerationStates[0].speedMetersPerSecond*39.3701) / voltage;
            double rightFrontPower = feedforward.compute(velocityStates[1].speedMetersPerSecond*39.3701, accelerationStates[1].speedMetersPerSecond*39.3701) / voltage;
            double leftBackPower = feedforward.compute(velocityStates[2].speedMetersPerSecond*39.3701, accelerationStates[2].speedMetersPerSecond*39.3701) / voltage;
            double rightBackPower = feedforward.compute(velocityStates[3].speedMetersPerSecond*39.3701, accelerationStates[3].speedMetersPerSecond*39.3701) / voltage;

            modules[0].set(leftFrontPower, velocityStates[0].angle.getRadians());
            modules[1].set(rightFrontPower, velocityStates[1].angle.getRadians());
            modules[2].set(leftBackPower, velocityStates[2].angle.getRadians());
            modules[3].set(rightBackPower, velocityStates[3].angle.getRadians());

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }
    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                modules[0].stopMotor();
                modules[1].stopMotor();
                modules[2].stopMotor();
                modules[3].stopMotor();

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);

            SwerveModuleState[] velocityStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(command.linearVel.x.get(0), command.linearVel.y.get(0), command.angVel.get(0)));
            SwerveModuleState[] accelerationStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(command.linearVel.x.get(1), command.linearVel.y.get(1), command.angVel.get(1)));
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            double leftFrontPower = feedforward.compute(velocityStates[0].speedMetersPerSecond*39.3701, accelerationStates[0].speedMetersPerSecond*39.3701) / voltage;
            double rightFrontPower = feedforward.compute(velocityStates[1].speedMetersPerSecond*39.3701, accelerationStates[1].speedMetersPerSecond*39.3701) / voltage;
            double leftBackPower = feedforward.compute(velocityStates[2].speedMetersPerSecond*39.3701, accelerationStates[2].speedMetersPerSecond*39.3701) / voltage;
            double rightBackPower = feedforward.compute(velocityStates[3].speedMetersPerSecond*39.3701, accelerationStates[3].speedMetersPerSecond*39.3701) / voltage;

            modules[0].set(leftFrontPower, velocityStates[0].angle.getRadians());
            modules[1].set(rightFrontPower, velocityStates[1].angle.getRadians());
            modules[2].set(leftBackPower, velocityStates[2].angle.getRadians());
            modules[3].set(rightBackPower, velocityStates[3].angle.getRadians());

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }



    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}
