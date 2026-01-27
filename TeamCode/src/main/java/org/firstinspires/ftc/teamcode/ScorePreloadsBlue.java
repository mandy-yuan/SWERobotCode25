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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class ScorePreloadsBlue extends OpMode {
    private DcMotorEx shooterMotor1;

    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private ShooterSubsystem shooterSubsystem;


//put intake next to wall
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(50, 90, Math.toRadians(135));
    private final Pose endPose = new Pose(50, 55, Math.toRadians(180));
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
                shooterSubsystem.revToRPM(2000);
                follower.followPath(scorePreload);
                setPathState(1);
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /*  will wait until the robot position is close (1 inch away) from the scorePose's position */
        if(!follower.isBusy()) {
            /* Score Preload */
            if(actionTimer.seconds()<2){
                intakeMotor.setPower(-0.5);
            }
            follower.followPath(leaveShootingZone);

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
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

        pathTimer = new ElapsedTime();
        opmodeTimer =  new ElapsedTime();
        actionTimer =  new ElapsedTime();
        opmodeTimer.reset();

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
        actionTimer.reset();
        while(actionTimer.seconds()<3){
            shooterSubsystem.setPowerTo(0.75);
            while(actionTimer.seconds()>1.5 && actionTimer.seconds()<1.8){intakeMotor.setPower(-0.65);}
            while(actionTimer.seconds()>2 && actionTimer.seconds()<2.2){intakeMotor.setPower(-0.65);}
            while(actionTimer.seconds()>2.4 && actionTimer.seconds()<2.7){intakeMotor.setPower(-0.65);}
        }
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}