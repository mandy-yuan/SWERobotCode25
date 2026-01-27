package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@TeleOp
public class CombinedTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private DcMotorEx intakeMotor;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private ShooterSubsystem shooterSubsystem;
    private ElapsedTime timer;

    private int shooterState = 0;
    private double shooterMotorPower;
    private double intakeMotorPower = -0.8;

    private final Pose farBlue = new Pose(70,25,120);
    private final Pose farRed = new Pose(72,25,60);
    private final Pose nearBlue = new Pose(72,82,135);
    private final Pose nearRed = new Pose(72,82,45);



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(72, 72, 0) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterSubsystem = new ShooterSubsystem(shooterMotor1, shooterMotor2);

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        //Driver controls
        if (gamepad1.leftBumperWasPressed()) {
            follower.setPose(new Pose(72, 72, 0));
        }
        if (gamepad1.left_trigger > 0) {
            follower.setPose(new Pose(72, 72, 180));
        }

        //Shooting setpoints (far/small triangle and close/large triangle)
        if (gamepad1.xWasPressed() && !automatedDrive) {
            automatedDrive = true;
            driveToNearBlue();
        }
        else if (gamepad2.yWasPressed()&&!automatedDrive) {
            automatedDrive = true;
            driveToNearRed();
        }
        else if (gamepad2.right_trigger>0&&!automatedDrive){
            automatedDrive = true;
            driveToFarBlue();
        }
        else if (gamepad2.right_bumper&&!automatedDrive){
            automatedDrive = true;
            driveToFarRed();
        }

        if (automatedDrive && !follower.isBusy()) {
            automatedDrive = false;
            follower.startTeleopDrive();
        }

        //Normal Teleop driving
        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );
//            //SlowMode on
//            else follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y * slowModeMultiplier,
//                    -gamepad1.left_stick_x * slowModeMultiplier,
//                    -gamepad1.right_stick_x * slowModeMultiplier,
//                    false // Robot Centric
//            );
        }



        //Operator controls

        //Intake/unintake
        if (gamepad2.left_bumper) {
            intakeMotor.setPower(intakeMotorPower);
        } else if(gamepad2.right_bumper){
            intakeMotor.setPower(0.4);
        }
        else {
            intakeMotor.setPower(0);
        }

//      ugly

//        if(gamepad2.aWasPressed()){
//            timer.reset();
//            while(timer.seconds()<3){
//                shooterSubsystem.setPowerTo(1);
//                while(timer.seconds()>1.5 && timer.seconds()<1.8){intakeMotor.setPower(intakeMotorPower);}
//                while(timer.seconds()>2 && timer.seconds()<2.2){intakeMotor.setPower(intakeMotorPower);}
//                while(timer.seconds()>2.4 && timer.seconds()<2.7){intakeMotor.setPower(intakeMotorPower);}
//            }
//        }
//        else if(gamepad2.yWasPressed()){
//            timer.reset();
//            while(timer.seconds()<3){
//                shooterSubsystem.setPowerTo(0.75);
//                while(timer.seconds()>1.5 && timer.seconds()<1.8){intakeMotor.setPower(intakeMotorPower);}
//                while(timer.seconds()>2 && timer.seconds()<2.2){intakeMotor.setPower(intakeMotorPower);}
//                while(timer.seconds()>2.4 && timer.seconds()<2.7){intakeMotor.setPower(intakeMotorPower);}
//            }
//        }
//        else if(gamepad2.xWasPressed()){
//            timer.reset();
//            while(timer.seconds()<3){
//                shooterSubsystem.setPowerTo(0.75);
//                while(timer.seconds()>1.5 && timer.seconds()<1.8){intakeMotor.setPower(intakeMotorPower);}
//                while(timer.seconds()>2 && timer.seconds()<2.2){intakeMotor.setPower(intakeMotorPower);}
//                while(timer.seconds()>2.4 && timer.seconds()<2.7){intakeMotor.setPower(intakeMotorPower);}
//            }
//        }
//        else if(gamepad2.right_trigger>0){
//            shooterSubsystem.setPowerTo(1);
//        }
//        else if(gamepad2.left_trigger>0){
//            shooterSubsystem.setPowerTo(1);
//        }
//        else{
//            shooterSubsystem.stop();
//        }
//
//
// shooter buttons, 25% power increment at 1.5 seconds
        // switch case inspired by brian
        switch (shooterState){
            case 0:
                intakeMotor.setPower(0);
                if(gamepad2.aWasPressed()){
                    shooterMotorPower = 0.5;
                    timer.reset();
                    shooterState = 1;
                }
                else if(gamepad2.xWasPressed()){
                    shooterMotorPower = 0.75;
                    timer.reset();
                    shooterState = 1;
                }
                else if(gamepad2.yWasPressed()){
                    shooterMotorPower = 1;
                    timer.reset();
                    shooterState = 1;
                }
                break;
            case 1:
                double time = timer.seconds();
                shooterSubsystem.setPowerTo(shooterMotorPower);
                if ((time>1.5 && time< 1.8)||(time>2.0 && time<2.2)||(time>2.4 && time<2.7)) {
                    intakeMotor.setPower(intakeMotorPower);
                } else {
                    intakeMotor.setPower(0);
                }

                if(time>3.0){
                    shooterSubsystem.stop();
                    intakeMotor.setPower(0);
                    shooterState=0;
                }
                break;
        }




//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }

//        //Slow Mode
//        if (gamepad1.dpadDownWasPressed()) {
//            slowMode = !slowMode;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);



    }
    private void driveToFarBlue(){
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), farBlue))
                        .setLinearHeadingInterpolation(follower.getHeading(), farBlue.minus(follower.getPose()).getAsVector().getTheta())
                        .build()
        );
    }

    private void driveToFarRed(){
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), farRed))
                        .setLinearHeadingInterpolation(follower.getHeading(), farRed.minus(follower.getPose()).getAsVector().getTheta())
                        .build()
        );
    }

    private void driveToNearBlue(){
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), nearBlue))
                        .setLinearHeadingInterpolation(follower.getHeading(), nearBlue.minus(follower.getPose()).getAsVector().getTheta())
                        .build()
        );
    }

    private void driveToNearRed(){
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), nearRed))
                        .setLinearHeadingInterpolation(follower.getHeading(), nearRed.minus(follower.getPose()).getAsVector().getTheta())
                        .build()
        );
    }

    public void stop() {}
    }





