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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class CombinedTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private Servo spindexerServo;
    private DcMotorEx intakeMotor;
    private SpindexerSubsystem spindexerSubsystem;
    private DcMotorEx shooterMotor;
    private Servo serializerServo;

    private static final int SPINDEXER_TICKS_PER_REVOLUTION = 1120;
    private ShooterSubsystem shooterSubsystem;

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
        spindexerServo = hardwareMap.get(Servo.class, "spindexer");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        spindexerSubsystem = new SpindexerSubsystem(spindexerServo);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterSubsystem = new ShooterSubsystem(shooterMotor);
        serializerServo = hardwareMap.get(Servo.class, "serializer");
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        spindexerServo.setPosition(0);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (gamepad1.leftBumperWasPressed()) {
            follower.setPose(new Pose(72, 72, 0));
        }
        if (gamepad1.left_trigger > 0) {
            follower.setPose(new Pose(72, 72, 180));
        }


        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.8,
                    -gamepad1.left_stick_x * 0.8,
                    -gamepad1.right_stick_x * 0.8,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

//        //Slow Mode
//        if (gamepad1.rightBumperWasPressed()) {
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


        if (gamepad2.yWasPressed()) {
            spindexerSubsystem.rotateSpindexerShooter();
        }
        if (gamepad2.xWasPressed()) {
            spindexerSubsystem.rotateSpindexerIntake();
        }
        //if (gamepad2.rightBumperWasPressed()) {
        //    spindexerSubsystem.rotateSpindexerIntake();
        //}

        if (gamepad2.right_trigger > 0) {
            //shooterMotor.setPower(1);
            shooterSubsystem.revToRPM(6000);
        } else {
            shooterSubsystem.stop();
        }

        if (gamepad2.a) {
            serializerServo.setPosition(0.7);
        } else {
            serializerServo.setPosition(0.48);
        }

        if (gamepad2.b) {
            intakeMotor.setPower(0.5);
        } else if(gamepad2.left_bumper){
            intakeMotor.setPower(-0.5);
        }
        else {
            intakeMotor.setPower(0);
        }



    }
    public void stop() {}
    }




