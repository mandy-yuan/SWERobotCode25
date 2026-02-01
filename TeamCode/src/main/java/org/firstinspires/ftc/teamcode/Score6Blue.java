package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Score6Blue extends OpMode {
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int actionState;
    private ShooterSubsystem shooterSubsystem;

    private final Pose startPose = new Pose(86, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(72,82,Math.toRadians(225));
    private final Pose prepareIntakePose1 = new Pose(96,60, Math.toRadians(0));
    private final Pose pickUpPose1 = new Pose(130,60, Math.toRadians(0));
    private final Pose endPose = new Pose(84, 55, Math.toRadians(0));
    private Path scorePreload;
    private Path endAuton;
    private PathChain score1;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        endAuton = new Path(new BezierLine(scorePose, endPose));
        endAuton.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());

        //cycle 1
        score1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(80.660, 60.330),
                                new Pose(104.220, 59.580)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(104.220, 59.580),
                                new Pose(123.000, 59.660)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(123.000, 59.660),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0: //preload path
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1: //score preload
                shootBalls(2);
                break;
            case 2: //pickup balls 1
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intakeMotor.setPower(-0.6);
                    follower.followPath(score1);
                    setPathState(3);
                }
                break;
            case 3: //adjust balls 1
                adjustBalls(4);
                break;
            case 4: //go to shooting pose and shoot
                shootBalls(5);
                break;
            case 5: //go to end position
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(endAuton);
                    setPathState(-1);
                }
                break;
        }
    }
    private void shootBalls(int nextState) {
        if (!follower.isBusy()) {
            scorePreloads();
            if (actionState == -1) {
                actionState = 0;
                setPathState(nextState);
            }
        }
    }
    private void adjustBalls(int nextState){
        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) {
            if (pathTimer.getElapsedTimeSeconds() < 0.2) {
                intakeMotor.setPower(0.4);
            } else {
                intakeMotor.setPower(0);
                setPathState(nextState);
            }
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/

    public void scorePreloads() {
        switch(actionState){
            case 0:
                actionTimer.resetTimer();
                actionState = 1;
            break;
            case 1:
                shooterSubsystem.setPowerTo(0.8);
                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                    actionTimer.resetTimer();
                    actionState = 2;
                }
                break;

            case 2:
                double time = actionTimer.getElapsedTimeSeconds();
                if (time<0.1 || (0.3<time && time<0.4) || (0.6<time &&time<0.7)){
                    intakeMotor.setPower(-0.8);
                }
                else{
                    intakeMotor.setPower(0);
                }
                if(time>3){
                    intakeMotor.setPower(0);
                    actionState = -1;
                }
                break;
        }

    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
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