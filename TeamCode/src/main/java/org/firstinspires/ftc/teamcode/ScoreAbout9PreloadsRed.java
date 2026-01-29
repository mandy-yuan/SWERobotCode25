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
public class ScoreAbout9PreloadsRed extends OpMode {
    private DcMotorEx shooterMotor1;

    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;
    private Servo spindexerServo;

    private Servo serializerServo;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private ShooterSubsystem shooterSubsystem;
    private SpindexerSubsystem spindexerSubsystem;



    private final Pose startPose = new Pose(86, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(72,82,Math.toRadians(225));
    private final Pose prepareIntakePose1 = new Pose(56,36,Math.toRadians(180));
    private final Pose intakePose1 = new Pose(22,36, Math.toRadians(180));
    private final Pose prepareIntakePose2 = new Pose(56,60, Math.toRadians(180));
    private final Pose intakePose2 = new Pose(22,60, Math.toRadians(180));

    private final Pose endPose = new Pose(84, 55, Math.toRadians(0));
    private Path scorePreload;
    private Path pickUpPosition1;
    private Path pickup1;
    private Path pickUpPosition2;
    private Path pickup2;
    private Path endAuton;
    //private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pickUpPosition1 = new Path(new BezierLine(scorePose, prepareIntakePose1));
        pickUpPosition1.setLinearHeadingInterpolation(scorePose.getHeading(), prepareIntakePose1.getHeading());

        pickup1 = new Path(new BezierLine(prepareIntakePose1, intakePose1));
        pickup1.setLinearHeadingInterpolation(prepareIntakePose1.getHeading(), intakePose1.getHeading());

        pickUpPosition2 = new Path(new BezierLine(scorePose, prepareIntakePose2));
        pickUpPosition2.setLinearHeadingInterpolation(scorePose.getHeading(), prepareIntakePose2.getHeading());

        pickup2 = new Path(new BezierLine(prepareIntakePose2, intakePose2));
        pickup2.setLinearHeadingInterpolation(prepareIntakePose2.getHeading(), intakePose2.getHeading());


        endAuton = new Path(new BezierLine(scorePose, endPose));
        endAuton.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());


        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */
    }

    public void autonomousPathUpdate() {
        shooterSubsystem.setPowerTo(0.8);
        //score cycle 0
        follower.followPath(scorePreload);
        if(!follower.isBusy()) {
            scorePreloads();
        }

        //score cycle 1
        follower.followPath(pickUpPosition1);
        if(!follower.isBusy()) {
            follower.followPath(pickup1);
        }
        follower.followPath(scorePreload);
        if(!follower.isBusy()) {
            scorePreloads();
        }

        //score cycle 2
        follower.followPath(pickUpPosition2);
        if(!follower.isBusy()) {
            follower.followPath(pickup2);
        }
        follower.followPath(scorePreload);
        if(!follower.isBusy()) {
            scorePreloads();
        }

        follower.followPath(endAuton);
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/

    public void scorePreloads() {
        switch(pathState){
            case 0:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 1;
                }
            break;
            case 1:
                shooterSubsystem.setPowerTo(0.8);
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2:
                double time = actionTimer.getElapsedTimeSeconds();
                if (time<0.1 || (0.3<time && time<0.4) || (0.6<time &&time<0.7)){
                    intakeMotor.setPower(-0.6);
                }
                if(time>0.8){
                    intakeMotor.setPower(0);
                    pathState = 0;
                }
                break;
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
        serializerServo = hardwareMap.get(Servo.class, "serializer");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        shooterSubsystem = new ShooterSubsystem(shooterMotor1, shooterMotor2);

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
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}