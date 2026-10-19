package org.firstinspires.ftc.teamcode.ArcPlanner.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Clase que controla el chasis del robot utilizando motores en configuración Mecanum/Standard
 * y soporta compensación de voltaje y modos de conducción (Tank, Field-Centric y Robot-Centric).
 */
public class ArcChasis
{

    // Declaración de los motores del chasis (Front-Left, Front-Right, Back-Left, Back-Right)
    private DcMotorEx FL, FR, BL, BR;

    // Sensor para leer el voltaje actual de la batería
    private VoltageSensor batteryVoltage;

    // Voltaje nominal de referencia para la compensación de potencia (12 Voltios)
    private static double NOMINAL_VOLTAGE = 12.0;

    /**
     * Constructor para inicializar los componentes de hardware del chasis.
     */
    public ArcChasis(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, VoltageSensor voltageSensor)
    {
        this.FL = frontLeft;
        this.FR = frontRight;
        this.BL = backLeft;
        this.BR = backRight;
        this.batteryVoltage = voltageSensor;

        configureMotors();
    }

    /**
     * Configura las direcciones iniciales, el comportamiento de frenado (Brake)
     * y el modo de operación (sin encoders) para cada motor.
     */
    private void configureMotors()
    {
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motors = {FL, FR, BL, BR};
        for (DcMotorEx motor : motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Ajusta la potencia objetivo en función del voltaje actual de la batería
     * para mantener un comportamiento consistente a medida que la batería se descarga.
     */
    private double getVoltageCompesatePower(double targetPower)
    {
        double currentVoltage = batteryVoltage.getVoltage();

        if (currentVoltage < 1.0) return targetPower;

        return targetPower * (NOMINAL_VOLTAGE / currentVoltage);
    }

    /**
     * Método de conducción tipo Tank / Arcade básico basado en traslación lineal y giro.
     */
    public void TankDrive(double y, double rx)
    {
        if (y != 0)
        {
            FL.setPower(y);
            FR.setPower(y);
            BL.setPower(y);
            BR.setPower(y);
        }
        else if (rx != 0)
        {
            FL.setPower(rx);
            FR.setPower(-rx);
            BL.setPower(rx);
            BR.setPower(-rx);
        }
        else
        {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }

    /**
     * Realiza la transformación matemática para la conducción orientada al campo (Field-Centric),
     * ajustando los vectores de movimiento según la orientación actual del robot (heading).
     */
    public void FielCentricDrive(double x, double y, double rx, double heading)
    {
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        RobotCentric(rotX, rotY, rx);
    }

    /**
     * Calcula y aplica las potencias individuales a cada rueda para la conducción Mecanum orientada al robot (Robot-Centric),
     * aplicando normalización de denominador y compensación de voltaje.
     */
    public void RobotCentric(double x, double y, double rx)
    {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double driveY = getVoltageCompesatePower(y);
        double driveX = getVoltageCompesatePower(x);
        double driveRx = getVoltageCompesatePower(rx);

        FL.setPower(((driveY + driveX + driveRx) / denominator) * 0.5);
        BL.setPower(((driveY - driveX + driveRx) / denominator) * 0.5);
        FR.setPower(((driveY - driveX - driveRx) / denominator) * 0.5);
        BR.setPower(((driveY + driveX - driveRx) / denominator) * 0.5);
    }
}
