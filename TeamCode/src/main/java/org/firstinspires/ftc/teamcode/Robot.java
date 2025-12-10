package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.everglow_library.RobotBase;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
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
    private Shooter shooter;
    public Robot(HardwareMap hardwareMap) {
        subsystems = new Subsystem[3];
        this.drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        camera = new Camera(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        subsystems[0] = intake;
        subsystems[1] = camera;
        subsystems[2] = shooter;
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
