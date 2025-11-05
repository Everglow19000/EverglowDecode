package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.everglow_library.RobotBase;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

public class Robot extends RobotBase {

    public static Motif currentMotif;
    Intake intake;
    public Robot(HardwareMap hardwareMap) {
        subsystems = new Subsystem[1];
        this.drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap);
        subsystems[0] = intake;
    }
    @Override
    public void update(int iterationCount) {

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
