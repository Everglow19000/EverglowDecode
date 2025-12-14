package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.everglow_library.RobotBase;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

public class Robot extends RobotBase {
    public static Vector2d goalPose = new Vector2d(-62, -60);
    public static Motif currentMotif;
    Intake intake;
    public Robot(HardwareMap hardwareMap, boolean isBlue) {
        goalPose = new Vector2d(goalPose.x, goalPose.y*(isBlue ? 1 : -1));
        subsystems = new Subsystem[1];
        this.drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap);
        subsystems[0] = intake;
    }
    @Override
    public void update(int iterationCount) {
        updateSubsystems(iterationCount);
    }
    public Action getOrientRobotForShootAction() {
        Pose2d currentPose = drive.localizer.getPose();
        Vector2d currentVector = currentPose.position;
        Vector2d goalPoseDiff = goalPose.minus(currentVector);
        return drive.actionBuilder(currentPose)
                .turnTo(Math.atan2(goalPoseDiff.x, goalPoseDiff.y))
                .build();
    }
    public Action getLocalizeWithApriltagAction(Pose2d pose) {
        return null;
    }
    public Action getOrderArtifactsAction(Motif motif) {
        return null;
    }
    public Action getSpinUpShooterAction(double distance) {
        return null;
    }
    public Action getStopShooterAction() {
        return null;
    }
    public Action getLaunchSingleArtifactAction() {
        return null;
    }
    public Action getMotifFromObeliskAction() {
        return null;
    }
    public Action getStartIntakeAction() {
        return intake.getStartIntakeAction();
    }
    public Action getStopIntakeAction() {
        return intake.getStopIntakeAction();
    }

    public double calculateDistanceFromGoal() {
        Vector2d diff = goalPose.minus(drive.localizer.getPose().position);

        return Math.sqrt(Math.pow(diff.x, 2) + Math.pow(diff.y, 2));
    }

    public void startIntake(){ //start intake
        intake.startIntake();
    }
    public void stopIntake(){ //stop intake
        intake.stopIntake();
    }

    public void startBelt(){ //start belt

    }

    public void stopBelt(){ //stop belt

    }

    public void prepareToLaunch() { //starts to spin the launcher based on the distance to the target and directs the robot to the target
    }

    public boolean isReadyToLaunch() { //checks if the launcher is ready to launch
        return true;
    }

    public void launchSingle() { //launches a single ball

    }
}
