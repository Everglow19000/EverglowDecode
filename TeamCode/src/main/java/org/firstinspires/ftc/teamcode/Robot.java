package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.everglow_library.RobotBase;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

public class Robot extends RobotBase {

    public static Motif currentMotif;
    private Camera camera;
    public Robot(HardwareMap hardwareMap) {
        subsystems = new Subsystem[1];
        this.drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        camera = new Camera(hardwareMap);
        subsystems[0] = camera;
    }
    @Override
    public void update(int iterationCount) {

    }
    // pose is formatted as following, since Pose2d class cannot be changed:
    // [x, y, heading]
    public Action getLocalizeWithApriltagAction(double[] pose) {
        return camera.getFindLocationAction(pose, 100);
    }
    // the contents of motif[0] will be changed according to the Motif on the obelisk
    public Action getOrderArtifactsAction(Motif[] motif) {
        return camera.getDetermineMotifAction(motif);
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
        return null;
    }
    public Action getStopIntakeAction() {
        return null;
    }

    public void startIntake(){ //start intake

    }
    public void stopIntake(){ //stop intake

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
