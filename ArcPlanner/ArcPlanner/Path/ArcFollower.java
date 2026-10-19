package org.firstinspires.ftc.teamcode.ArcPlanner.Path;

import org.firstinspires.ftc.teamcode.ArcPlanner.Core.ArcPose;
import org.firstinspires.ftc.teamcode.ArcPlanner.Hardware.ArcChasis;
import org.firstinspires.ftc.teamcode.ArcPlanner.Core.ArcPID;

/**
 * Clase encargada de seguir trayectorias y puntos objetivo utilizando controladores PID
 * para posición y orientación, interactuando directamente con el chasis del robot.
 */
public class ArcFollower
{
    private ArcPID xController, yController, headingController;
    private ArcChasis chasis;
    private double posTolerance = 10.0;
    private double headTolerance = Math.toRadians(2);

    /**
     * Constructor que inicializa el seguidor con los controladores PID para los ejes X, Y y el heading,
     * configurando valores iniciales de tolerancias y compensación estática.
     */
    public ArcFollower(ArcChasis chasis)
    {
        this.chasis = chasis;

        this.xController = new ArcPID(0.05, 0, 0.001);
        this.yController = new ArcPID(0.05, 0, 0.001);
        this.headingController = new ArcPID(1.2, 0, 0.05);

        setAllTolerances(5.0, Math.toRadians(2));
        setAllStaticComp(0.05, 0.08);
    }

    /**
     * Verifica si el robot aún se encuentra navegando hacia el objetivo,
     * evaluando si la distancia o el error angular superan las tolerancias permitidas.
     */
    public boolean isBusy(ArcPose current, ArcPose target) {
        if (target == null) return false;

        double distance = Math.hypot(target.x - current.x, target.y - current.y);

        double angleError = target.heading - current.heading;
        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;

        // El seguidor está ocupado si la distancia es mayor a la tolerancia
        // O si el ángulo aún no está en rango.
        return (distance > posTolerance || Math.abs(angleError) > headTolerance);
    }

    /**
     * Configura la tolerancia para considerar que se ha llegado al objetivo en posición y orientación.
     */
    public void setAllTolerances(double posTolerance, double headingTolerance) {
        xController.setTolerance(posTolerance);
        yController.setTolerance(posTolerance);
        headingController.setTolerance(headingTolerance);
    }

    /**
     * Configura el kS (Feedforward) para vencer la fricción estática en los motores.
     */
    public void setAllStaticComp(double driveKS, double headingKS) {
        xController.setFeedforward(driveKS);
        yController.setFeedforward(driveKS);
        headingController.setFeedforward(headingKS);
    }

    /**
     * Configura los límites de la suma integral para evitar el windup en los controladores.
     */
    public void setAllIntegralLimits(double limit) {
        xController.setIntegralLimit(limit);
        yController.setIntegralLimit(limit);
        headingController.setIntegralLimit(limit);
    }

    /**
     * Actualiza el movimiento del chasis en cada ciclo de control calculando los vectores de velocidad
     * mediante los controladores PID hacia el punto objetivo (target) basándose en la pose actual (current).
     */
    public void update(ArcPose current, ArcPose target)
    {
        if (target == null)
        {
            chasis.RobotCentric(0, 0, 0);
            return;
        }

        double dx = target.x - current.x;
        double dy = target.y - current.y;

        double distance = Math.hypot(dx, dy);

        double dirX = 0;
        double dirY = 0;
        if (distance > 1e-6) {
            dirX = dx / distance;
            dirY = dy / distance;
        }

        double speed = xController.calculate(distance, 0);

        double outputX = dirX * speed;
        double outputY = dirY * speed;

        double angleError = target.heading - current.heading;
        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;

        double outputRx = headingController.calculate(angleError, 0);

        if (Math.abs(angleError) > Math.toRadians(20)) {
            outputX *= 0.5;
            outputY *= 0.5;
        }

        if (distance < 50) {
            outputX *= 0.5;
            outputY *= 0.5;
        }

        double magnitude = Math.hypot(outputX, outputY);
        if (magnitude > 1.0) {
            outputX /= magnitude;
            outputY /= magnitude;
        }

        chasis.FielCentricDrive(outputX, outputY, outputRx, current.heading);
    }
}