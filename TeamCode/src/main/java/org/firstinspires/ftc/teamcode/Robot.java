package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.everglow_library.RobotBase;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Motif;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot extends RobotBase {
    public static Vector2d goalPoseDistance = new Vector2d(-62, -60);
    public static Vector2d goalEdge1 = new Vector2d(-48, -64);
    public static Vector2d goalEdge2 = new Vector2d(-70, -48);
    public static Motif currentMotif;

    Intake intake;
    private Camera camera;
    public Shooter shooter;
    private FeedingMechanism feedingMechanism;
    public Robot(HardwareMap hardwareMap, boolean isBlue, Motif motif) {
        goalEdge1 = new Vector2d(goalEdge1.x, goalEdge1.y*(isBlue ? 1 : -1));
        goalEdge2 = new Vector2d(goalEdge2.x, goalEdge2.y*(isBlue ? 1 : -1));
        goalPoseDistance = new Vector2d(goalPoseDistance.x, goalPoseDistance.y*(isBlue ? 1 : -1));

        subsystems = new Subsystem[4];

        this.drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        camera = new Camera(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feedingMechanism = new FeedingMechanism(hardwareMap, motif);

        camera.start();

        currentMotif = motif;

        subsystems[0] = intake;
        subsystems[1] = camera;
        subsystems[2] = shooter;
        subsystems[3] = feedingMechanism;
    }

    public Robot(HardwareMap hardwareMap, boolean isBlue) {
        this(hardwareMap, isBlue, Motif.NONE);
    }
    @Override
    public void update(int iterationCount) {
        updateSubsystems(iterationCount);
    }
    public Action getOrientRobotForShootAction() {
        return drive.actionBuilder(drive.localizer.getPose())
                .turnTo(Utils.getOptimalAngleToShoot(goalEdge1, goalEdge2, drive.localizer.getPose().position))
                .build();
    }

    // pose is formatted as following, since Pose2d class cannot be changed:
    // [x, y, heading]
    public Action getLocalizeWithApriltagAction(double[] pose) {
        return camera.getFindLocationAction(pose, 200);
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
        SequentialAction action = new SequentialAction(
                shooter.getWaitUntilShooterSpinupAction(),
                feedingMechanism.getFeedSingleArtifactAction(shootingSequence[0])
        );
        for (int i = 1; i < shootingSequence.length; i++) {
            action = new SequentialAction(
                    action,
                    shooter.getWaitUntilShooterSpinupAction(),
                    feedingMechanism.getFeedSingleArtifactAction(shootingSequence[i])
            );
        }
        return action;
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

    public double calculateDistanceFromGoal() {
        Vector2d diff = goalPoseDistance.minus(drive.localizer.getPose().position);

        return Math.sqrt(Math.pow(diff.x, 2) + Math.pow(diff.y, 2));
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

    public void prepareToLaunch() { //starts to spin the launcher based on the distance to the target and directs the robot to the target
    }

    public boolean isReadyToLaunch() { //checks if the launcher is ready to launch
        return Math.abs(drive.localizer.getPose().heading.toDouble() - Utils.getOptimalAngleToShoot(goalEdge1, goalEdge2, drive.localizer.getPose().position).toDouble()) <= 0.2 && shooter.isFlywheelFinishedSpinning();
    }

    public void launchSingle() { //launches a single ball

    }
}
