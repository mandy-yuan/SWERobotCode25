package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
public class PIDTuner extends OpMode {
    public DcMotorEx shooter;

    public double highVelocity = 5000;
    public double lowVelocity = 3000;

    double currentTarget = highVelocity;

    double F;
    double P;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init(){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init completed");
    }

    @Override
    public void loop(){
        //switch from far to near
        if (gamepad1.yWasPressed()) {
            if (currentTarget == highVelocity) {
                currentTarget = lowVelocity;
            } else {
                currentTarget = highVelocity;
            }
        }

        //adjust step size
        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex+1) % stepSizes.length;
        }

        //increase F value
        if (gamepad1.dpadLeftWasPressed()){
            F+=stepSizes[stepIndex];
        }
        //decrease F value
        if (gamepad1.dpadRightWasPressed()){
            F-=stepSizes[stepIndex];
        }

        //increase P value
        if (gamepad1.dpadUpWasPressed()){
            P+=stepSizes[stepIndex];
        }
        //decrease P value
        if (gamepad1.dpadDownWasPressed()){
            P-=stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter.setVelocity(currentTarget);

        double currentVelocity = shooter.getVelocity();
        double error = currentTarget-currentVelocity;

        telemetry.addData("Target velocity", currentTarget);
        telemetry.addData("Current velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("Tuning F", F);
        telemetry.addData("Tuning P", P);
        telemetry.addData("Step size", stepSizes[stepIndex]);
        telemetry.update();
    }
}
