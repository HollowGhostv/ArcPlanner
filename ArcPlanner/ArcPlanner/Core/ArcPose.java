package org.firstinspires.ftc.teamcode.ArcPlanner.Core;

import java.util.Locale;

/**
 * Clase que representa la posición y orientación (pose) de un robot en el plano cartesiano
 * mediante coordenadas (x, y) y un ángulo de orientación (heading).
 */
public class ArcPose
{
    // Coordenadas cartesianas y ángulo de orientación del robot
    public double x;
    public double y;
    public double heading;

    /**
     * Constructor para inicializar la pose con valores específicos de x, y y heading.
     */
    public ArcPose(double x, double y, double heading)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Constructor por defecto que inicializa la pose en el origen (0, 0, 0).
     */
    public ArcPose()
    {
        this(0, 0, 0);
    }

    /**
     * Calcula la distancia euclidiana entre la pose actual y otra pose dada.
     */
    public double distanceTo(ArcPose other)
    {
        return Math.hypot(other.x - this.x, other.y - this.y);
    }

    /**
     * Retorna el valor del heading convertido de radianes a grados.
     */
    public double getHeadingDegrees()
    {
        return Math.toDegrees(heading);
    }

    /**
     * Copia los valores de x, y y heading de otra pose a la pose actual.
     */
    public void copy(ArcPose other)
    {
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
    }

    /**
     * Devuelve una representación en cadena formateada de la pose (X, Y y Heading en grados).
     */
    @Override
    public String toString()
    {
        return String.format(Locale.ENGLISH, "X: %.2f, Y: %.2f, H: %.2f°", x, y, getHeadingDegrees());
    }
}