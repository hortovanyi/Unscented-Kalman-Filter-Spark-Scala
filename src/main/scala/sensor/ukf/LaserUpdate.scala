package sensor.ukf

import breeze.linalg.{DenseMatrix, DenseVector}
import org.apache.log4j.Logger
import sensor.models.MeasurementPackage
import sensor.ukf.UKFTypes.{CoVarianceMatrix, SigmaPoints, StateVector}

/**
  * Created by nick on 27/3/17.
  */
trait LaserUpdate extends UKFUpdate {
  val n_z_laser: Int = 2 // laser measurement dimensions px,py

  private val logger = Logger.getLogger(this.getClass.getName.split("\\$").last)

  // Laser measurement noise standard deviation position1 in m
  val std_laspx: Double = 0.15

  // Laser measurement noise standard deviation position2 in m
  val std_laspy: Double = 0.15

  def laserUpdate(measurementPackage: MeasurementPackage, XsigPred: SigmaPoints,
                  x: StateVector, P: CoVarianceMatrix): (StateVector, CoVarianceMatrix) = {
    // Measurement
    val z = DenseVector(measurementPackage.raw_measurements.toArray)
    val n_z = n_z_laser
    assert(z.length == n_z, "Laser measurement dimensions aren't valid")

    // pattern variables must start with lowercase letter
    val (zPred: StateVector, mS: CoVarianceMatrix) = predictLaserMeasurement(n_x, n_aug, n_z, XsigPred)

    val Zsig: SigmaPoints = sigmaPointsLaserMeasurementSpace(n_z, n_aug, XsigPred)

    logger.debug("n_z: " + n_z + " zPred: " + zPred + "z: " + z)
    super.UpdateState(n_x, n_aug, n_z, XsigPred, x, P, Zsig, zPred, mS, z)
  }

  private def predictLaserMeasurement(n_x: Int, n_aug: Int, n_z: Int, XsigPred: SigmaPoints): (StateVector, CoVarianceMatrix) = {
    val Zsig: SigmaPoints = sigmaPointsLaserMeasurementSpace(n_z, n_aug, XsigPred)

    val (zPred, mS) = super.predictMeasurement(n_x, n_aug, n_z, Zsig, XsigPred)

    // add measurement noise
    mS :+= laserNoiseCovarianceMatrix

    (zPred, mS)
  }

  private def laserNoiseCovarianceMatrix: CoVarianceMatrix = {
    DenseMatrix(
      (std_laspx * std_laspx, 0.0),
      (0.0, std_laspy * std_laspy)
    )
  }

  private def sigmaPointsLaserMeasurementSpace(n_z: Int, n_aug: Int, XsigPred: SigmaPoints): SigmaPoints = {
    var Zsig: SigmaPoints = DenseMatrix.zeros[Double](n_z, 2 * n_aug + 1)

    for (i <- 0 until 2 * n_aug + 1) {
      val p_x = XsigPred(0, i)
      val p_y = XsigPred(1, i)

      // measurement model
      Zsig(0, i) = p_x
      Zsig(1, i) = p_y
    }
    Zsig
  }
}
