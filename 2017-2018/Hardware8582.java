package org.firstinspires.ftc.teamcode.FTC8582;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by myradada on 10/9/17.
 */

/**
 * This is NOT an opmode.
 * <p/>
 * This class contains all hardware for Team 8582's robot. It can be imported at the start of each class in place of adding motors, etc.
 */

//Uncomment hardware components as they are added to the robot.
//Make sure robot controller is using the correct configuration.

public class Hardware8582 {

    /* Public OpMode members. */
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorBR = null;
    public DcMotor motorBL = null;
    public DcMotor motorLift = null;

    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo jewels = null;

    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor rangeLeft;
    ModernRoboticsI2cRangeSensor rangeRight;
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware8582() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        //Save reference to Hardware map
        hwMap = ahwMap;

        //Define and Initialize Motors
        motorFR = hwMap.get(DcMotor.class, "motorFR");
        motorFL = hwMap.get(DcMotor.class, "motorFL");
        motorBR = hwMap.get(DcMotor.class, "motorBR");
        motorBL = hwMap.get(DcMotor.class, "motorBL");
        motorLift = hwMap.get(DcMotor.class, "lift");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);

        colorSensor = hwMap.colorSensor.get("color");
        rangeLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeLeft");
        rangeLeft.setI2cAddress(I2cAddr.create8bit(0x26));//added
        rangeRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeRight");
        rangeRight.setI2cAddress(I2cAddr.create8bit(0x28));//added and changed address from 0x28
        sensorGyro = hwMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;

        //encoders:
        /* motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setMaxSpeed(2800);
        motorFL.setMaxSpeed(2800);
        motorBR.setMaxSpeed(2800);
        motorBL.setMaxSpeed(2800);
        motorLift.setMaxSpeed(2800);

        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorLift.setPower(0); */

        // Define and initialize ALL installed servos.
        leftClaw = hwMap.servo.get("left");
        rightClaw = hwMap.servo.get("right");
        jewels = hwMap.servo.get("jewels");

        leftClaw.setPosition(0.0);
        rightClaw.setPosition(1.0);
        jewels.setPosition(0.2);
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}


