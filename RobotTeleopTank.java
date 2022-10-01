/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import java.util.Queue;

/**
 * This particular OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop Tank", group="Robot")

public class RobotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  elivator1   = null;
    public DcMotor  elivator2   = null;
    public DcMotor  looper      = null;
    public DcMotorEx shooter2   = null;
    public DcMotorEx  shooter3  = null;
    public DcMotor  collector   = null;
    public Servo    machine1    = null;
    public Servo    machine2    = null;
    public Servo    machine3    = null;
    public Servo    machine4    = null;
    public Servo    lift1       = null;
    public Servo    lift2       = null;

    double clawOffset = 0;
    boolean shooterFlag ;
    boolean xFirstTime;
    private ElapsedTime xTime = new ElapsedTime();
    double elapsed;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;
    double kp = 10.0;  //0.000425
    double ki = 1.9;   //0.00017
    double kd = 0.0;  //0.0000155  try it ...photo
    double kf = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "walk1");
        rightDrive = hardwareMap.get(DcMotor.class, "walk2");
        elivator1  = hardwareMap.get(DcMotor.class, "elevator");
        elivator2  = hardwareMap.get(DcMotor.class, "elevator 2");
        looper   = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2   = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter3   = hardwareMap.get(DcMotorEx.class, "shooter3");
        collector  = hardwareMap.get(DcMotor.class, "collecter");
        machine1   = hardwareMap.get(Servo.class, "turbina1");
        machine2   = hardwareMap.get(Servo.class, "turbina2");
        machine3   = hardwareMap.get(Servo.class, "turbina3");
        machine4   = hardwareMap.get(Servo.class, "turbina4");
        lift1      = hardwareMap.get(Servo.class,"elevatordown");
        lift2      = hardwareMap.get(Servo.class, "elevatorup");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter3.setDirection(DcMotorSimple.Direction.REVERSE);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        shooterFlag = false;
        xFirstTime = false;
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooter3.setVelocityPIDFCoefficients(80.0, 0.0, 10.0, 10.0);


        //shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new PIDFCoefficients(0.5, 0.1, 0.01, 0.2));
        elapsed = 0;
        xTime.reset();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)


        leftDrive.setPower(-gamepad1.left_stick_y-gamepad1.right_stick_x);
        rightDrive.setPower(-gamepad1.left_stick_y+gamepad1.right_stick_x);

        if(gamepad2.a){
            double collectorspeed = -1;
            collector.setPower(gamepad2.left_trigger>0.1 ? collectorspeed : -collectorspeed);
        } else {
            collector.setPower(0);
        }





        if(gamepad2.y){
            double loopSpeed = 0.58;///////////////0.60
            looper.setPower(gamepad2.left_trigger>0.1 ? loopSpeed : -loopSpeed);
        } else {
            looper.setPower(0);
        }
//////////////////////////////////////////////////////////////////////////////////////////////////
       // if (Math.abs(gamepad2.left_stick_y-0.1)<0.1 && Math.abs(gamepad2.left_stick_x-0.1)<0.1) {  //if lift servos aren't running.
            elivator1.setPower(-gamepad2.right_stick_y);
            elivator2.setPower(-gamepad2.right_stick_y);
      //  }

        if(gamepad2.left_stick_y>0.3)
        {
            lift1.setPosition(1);
            if(gamepad2.left_trigger<0.1){
            elivator1.setPower(0.5);
            elivator2.setPower(0.5);
            }
        }
        else if(gamepad2.left_stick_y<-0.3)
        {
            lift1.setPosition(0);
            if(gamepad2.left_trigger<0.1){
            elivator1.setPower(-0.5);
            elivator2.setPower(-0.5);
            }
        }
        else {
            lift1.setPosition(0.5);
            if(gamepad2.right_stick_y == 0)
            {
                elivator1.setPower(0);
                elivator2.setPower(0);
            }
        }

        if(gamepad2.left_stick_x>0.3){
            lift2.setPosition(1);
            if(gamepad2.left_trigger<0.1){
            elivator1.setPower(1);
            elivator2.setPower(1);
            }
        }
        else if(gamepad2.left_stick_x<-0.3){
            lift2.setPosition(0);
            if(gamepad2.left_trigger<0.1){
            elivator1.setPower(-1);
            elivator2.setPower(-1);
            }
        }
        else{
            lift2.setPosition(0.5);
            if(gamepad2.right_stick_y == 0)
            {
                elivator1.setPower(0);
                elivator2.setPower(0);
            }
        }

        if(gamepad2.x){
            if(!xFirstTime)
            {
                elapsed = xTime.seconds();
                xFirstTime = true;
            }
            if(xTime.seconds() - elapsed >0.5)
            {
                shooterFlag = !shooterFlag;
                xFirstTime = false;
            }
        }
        if(shooterFlag)
        {
            int speed =1250;///////////1250
            shooter3.setVelocity(speed);
            shooter2.setVelocity(speed);
        }
        else {
            shooter2.setPower(0);
            shooter3.setPower(0);
            timer.reset();
            lastError=0;
            integralSum=0;
        }

//////////////////////////////////////////////////////////////////////////////////////////////
        // Send telemetry message to signify robot running;
        telemetry.addData("speed3",  "Offset = %.2f", shooter3.getVelocity());
        telemetry.addData("power3",  "Offset = %.2f", shooter3.getPower());
        telemetry.addData("speed2",  "Offset = %.2f", shooter2.getVelocity());
        telemetry.addData("power2",  "Offset = %.2f", shooter2.getPower());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public  double PIDControl(double reference, double state){
        double error = reference - state;
        if(Math.abs(error)<20)
            error=0;
        integralSum += error * timer.seconds();
        double derevative = (error - lastError) / timer.milliseconds();
        lastError = error;
        telemetry.addData("integralSum",  " %.2f", integralSum);
        telemetry.addData("error",  " %.2f", error);
        telemetry.addData("reference*kf",  " %.2f", reference*kf);
        telemetry.addData("lasterror",  " %.2f", lastError);
        telemetry.addData("derivative",  " %.2f", derevative);
        timer.reset();
        double output = (error * kp ) + (derevative * kd) + (integralSum *ki)+(reference*kf);
        telemetry.addData("output",  " %.2f", output);
        //if(output>-0.5)
        /*if(output>0.98)
            output=0.98;
        if(output<-0.98)
            output=-0.98;*/
        return output;
    }
}
