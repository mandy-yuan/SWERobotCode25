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
public class ScorePreloadsBlue extends OpMode {
    private DcMotorEx shooterMotor1;

    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;
    private Servo spindexerServo;

    private Servo serializerServo;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState=0;

    private ShooterSubsystem shooterSubsystem;
    private SpindexerSubsystem spindexerSubsystem;



    private final Pose startPose = new Pose(86, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(72,82,Math.toRadians(315));
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
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/

    public void scorePreloads() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2:
                shooterSubsystem.setPowerTo(0.85);
                if (actionTimer.getElapsedTimeSeconds() > 1.2) {
                    actionTimer.resetTimer();
                    pathState = 3;
                }
                break;

            case 3:
                double time = actionTimer.getElapsedTimeSeconds();
                if (time<0.1 || (0.3<time && time<0.45) || (0.85<time &&time<1.05)){
                    intakeMotor.setPower(-0.8);
                }
                else{
                    intakeMotor.setPower(0);
                }
                if(time>3){
                    intakeMotor.setPower(0);
                    shooterSubsystem.setPowerTo(0);
                    follower.followPath(leaveShootingZone);
                    pathState = 4;
                }
                break;
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        scorePreloads();

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
        opmodeTimer.resetTimer();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}