package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class ScorePreloadsRed extends OpMode {
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private Servo spindexerServo;

    private Servo serializerServo;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private ShooterSubsystem shooterSubsystem;
    private SpindexerSubsystem spindexerSubsystem;



    private final Pose startPose = new Pose(86, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(105, 115, Math.toRadians(45));
    private final Pose endPose = new Pose(84, 55, Math.toRadians(0));
    private Path scorePreload;
    private Path leaveShootingZone;
    //private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        leaveShootingZone = new Path(new BezierLine(scorePose, endPose));
        leaveShootingZone.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooterSubsystem.revToRPM(2000);
                intakeMotor.setPower(-0.2);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    scorePreloads();

                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(leaveShootingZone);
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void scorePreloads() {
        for (int i = 0; i < 3; i ++) {
            actionTimer.resetTimer();
            spindexerSubsystem.rotateSpindexerShooter();
            while (actionTimer.getElapsedTimeSeconds() < 0.7) {

            }

            serializerServo.setPosition(0.7);
            actionTimer.resetTimer();
            while (actionTimer.getElapsedTimeSeconds() < 1) {

            }
            actionTimer.resetTimer();
            serializerServo.setPosition(0.48);
            while (actionTimer.getElapsedTimeSeconds() < 1) {

            }
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        spindexerServo = hardwareMap.get(Servo.class, "spindexer");
        serializerServo = hardwareMap.get(Servo.class, "serializer");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        shooterSubsystem = new ShooterSubsystem(shooterMotor);
        spindexerSubsystem = new SpindexerSubsystem(spindexerServo);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        intakeMotor.setPower(-0.2);
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}