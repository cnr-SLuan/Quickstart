package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.workBench1;
import org.firstinspires.ftc.teamcode.mechanisms.workBench;

@TeleOp
public class DCmotorPractice extends OpMode {
    workBench bench = new workBench();

    @Override
    public void init(){
        bench.init(hardwareMap);
    }

    @Override
    public void loop(){
        double motorSpeed = gamepad1.right_stick_y;
        bench.setMotorSpeed(motorSpeed);

        if (gamepad1.a){
            bench.setMotorZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if (gamepad1.b){
            bench.setMotorZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (bench.isTouchSensorPressed()){
            bench.setMotorSpeed(0.5);
        }
        else{
            bench.setMotorSpeed(0.0);//stops the motor
        }
        telemetry.addData("Motor Revs", bench.getMotorRevs());
    }
}
