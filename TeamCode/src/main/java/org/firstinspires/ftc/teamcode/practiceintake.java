package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class practiceintake extends OpMode{ //public class means that it can be viewed anywhere and everywhere in the project
    private DcMotor intakeMotor; //private class means that its only there in the public class its in
    @Override
    public void init(){
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakePractice"); //?gets? the motor and allows the code to connect to it

    }
    @Override
    public void start(){}

    @Override //replaces parent classes method - method is basically a function
    public void loop(){  //void means that nothing is returned
        intakeMotor.setPower(0.3); //the power will be 1 and the intake will run
    }
    @Override
    public void stop(){}

}
