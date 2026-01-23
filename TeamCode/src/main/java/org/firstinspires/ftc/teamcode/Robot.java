package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.everglow_library.RobotBase;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColor;
import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Motif;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot extends RobotBase {
    private static double CAMERA_RELIABALITY_DISTANCE = 100;
    public static Vector2d goalPoseDistanceStatic = new Vector2d(-58.374, -55.641);
    public static Vector2d goalPoseOrientationStatic = new Vector2d(-75, -55);
    public static Vector2d goalEdge1Static = new Vector2d(-52, -61);
    public static Vector2d goalEdge2Static = new Vector2d(-66, -51);
    public Vector2d goalPoseDistance;
    public Vector2d goalPoseOrientation;
    public Vector2d goalEdge1;
    public Vector2d goalEdge2;


    public static Motif currentMotif;
    private static Pose2d lastActivationEndPose = null;

    Intake intake;
    public Camera camera;
    public Shooter shooter;
    public FeedingMechanism feedingMechanism;

    public boolean usedLastPose = false;
    private int iterationCount = 0;
    private boolean isBlue;
    private double distanceCache = -1;
    private int distanceCacheIteration = 0;
    public Robot(HardwareMap hardwareMap, boolean isBlue, boolean isAuto, Motif motif) {
        goalEdge1 = new Vector2d(goalEdge1Static.x, goalEdge1Static.y*(isBlue ? 1 : -1));
        goalEdge2 = new Vector2d(goalEdge2Static.x, goalEdge2Static.y*(isBlue ? 1 : -1));
        goalPoseDistance = new Vector2d(goalPoseDistanceStatic.x, goalPoseDistanceStatic.y*(isBlue ? 1 : -1));
        goalPoseOrientation = new Vector2d(goalPoseOrientationStatic.x, goalPoseOrientationStatic.y*(isBlue ? 1 : -1));

        subsystems = new Subsystem[4];

        if (lastActivationEndPose == null || isAuto) {
            this.drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            usedLastPose = false;
        }
        else {
            this.drive = new MecanumDrive(hardwareMap, lastActivationEndPose);
            usedLastPose = true;
        }
        camera = new Camera(hardwareMap, drive.localizer);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, this);
        feedingMechanism = new FeedingMechanism(hardwareMap, motif);

        this.isBlue = isBlue;

        camera.start();

        currentMotif = motif;

        subsystems[0] = intake;
        subsystems[1] = camera;
        subsystems[2] = shooter;
        subsystems[3] = feedingMechanism;
    }

    public Robot(HardwareMap hardwareMap, boolean isBlue) {
        this(hardwareMap, isBlue, true, Motif.NONE);
    }
    public void update() {
        iterationCount++;
        distanceCache = calculateDistanceFromGoal();
        distanceCacheIteration = iterationCount;
        updateSubsystems();
        if (feedingMechanism.isNowStoppedIntaking()) {
            stopIntake();
        }
    }
    public Action getOrientRobotForShootAction() {
        return drive.actionBuilder(drive.localizer.getPose())
                .turnTo(getOptimalAngleToShoot())
                .build();
    }

    public Rotation2d getOptimalAngleToShoot() {
        if (calculateDistanceFromGoal() >= CAMERA_RELIABALITY_DISTANCE) {
            return Utils.getOptimalAngleToShoot(goalEdge1, goalEdge2, drive.localizer.getPose().position);
        }
        return Utils.getOptimalAngleToShoot(goalPoseDistance, drive.localizer.getPose().position);
    }

    public void setEndPose(Pose2d pose) {
        lastActivationEndPose = pose;
    }

    // pose is formatted as following, since Pose2d class cannot be changed:
    // [x, y, heading]
    public Action getLocalizeWithApriltagAction(double[] pose) {
        return camera.getFindLocationAction(pose, 200);
    }

    public Action getLocalizeWithApriltagAction(double[] pose, boolean isMT2) {
        return camera.getFindLocationAction(pose, isMT2 ? 200 : 400, isMT2);
    }

    // the contents of motif[0] will be changed according to the Motif on the obelisk
    public Action getMotifFromObeliskAction(Motif[] motif) {
        return camera.getDetermineMotifAction(motif);
    }
    public Action getSpinUpShooterAction(double distance) {
        return new ParallelAction(
                shooter.getStartUpShooterAction(distance),
                shooter.getAimHoodAction(shooter.getServoAngleForDistanceFromGoal(distance))
        );
    }
    public Action getStopShooterAction() {
        return shooter.getStopShooterSpinAction();
    }
    public Action getLaunchAllArtifactsAction() {
        FeedingMechanism.SpindexerPosition[] shootingSequence = feedingMechanism.getShootingSequence();
        if (shootingSequence.length != 0) {
            SequentialAction action = new SequentialAction(
                    shooter.getWaitUntilShooterSpinupAction(),
                    feedingMechanism.getMoveSpindexerAction(shootingSequence[0]),
                    feedingMechanism.getFeedSingleArtifactAction(shootingSequence[0])
            );
            for (int i = 1; i < shootingSequence.length; i++) {
                action = new SequentialAction(
                        action,
                        shooter.getWaitUntilShooterSpinupAction(),
                        feedingMechanism.getMoveSpindexerAction(shootingSequence[i]),
                        feedingMechanism.getFeedSingleArtifactAction(shootingSequence[i])
                );
            }
            return action;
        }
        return new ParallelAction();
    }
    public Action getScanArtifactColorsAction() {
        return feedingMechanism.getScanArtifactColorsAction();
    }
    public Action getStartIntakeAction() {
        return intake.getStartIntakeAction();
    }
    public Action getStopIntakeAction() {
        return intake.getStopIntakeAction();
    }

    public String getFeedingMechanismContents() {
        return feedingMechanism.getStoredArtifacts();
    }
    public double calculateDistanceFromGoal() {
        if (iterationCount == distanceCacheIteration) {
            return distanceCache;
        }

        double cameraDistance = camera.getDistanceFromAprilTag(isBlue);
        Vector2d diff = goalPoseDistance.minus(drive.localizer.getPose().position);
        double positionDistance = Math.sqrt(Math.pow(diff.x, 2) + Math.pow(diff.y, 2));
        if (cameraDistance == -1) {
            return positionDistance;
        }

        if (cameraDistance >= CAMERA_RELIABALITY_DISTANCE || positionDistance >= CAMERA_RELIABALITY_DISTANCE) {
            camera.isUpdatePoseOnUpdate = false;
        }
        else if (cameraDistance < CAMERA_RELIABALITY_DISTANCE && positionDistance < CAMERA_RELIABALITY_DISTANCE) {
            camera.isUpdatePoseOnUpdate = true;
        }

        return cameraDistance <= CAMERA_RELIABALITY_DISTANCE ? cameraDistance : positionDistance;
    }

    public void setMotif(Motif motif) {
        feedingMechanism.setMotif(motif);
        currentMotif = motif;
    }

    public void startIntake(){ //start intake
        intake.startIntake();
        feedingMechanism.startIntaking();
    }
    public void stopIntake(){ //stop intake
        intake.stopIntake();
        feedingMechanism.stopIntaking();
    }

    public void setSpindexerPosition(FeedingMechanism.SpindexerPosition position) {
        feedingMechanism.setSpindexerPosition(position);
    }

    public void stopShooterMotor() {
        shooter.stopMotor();
    }
}
