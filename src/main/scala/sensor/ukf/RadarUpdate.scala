package sensor.ukf

import breeze.linalg.{DenseMatrix, DenseVector}
import breeze.numerics.{atan2, cos, sin, sqrt}
import org.apache.log4j.Logger
import sensor.models.MeasurementPackage
import sensor.ukf.UKFTypes.{CoVarianceMatrix, SigmaPoints, StateVector}

/**
  * Created by nick on 27/3/17.
  */
trait RadarUpdate extends UKFUpdate {
  val n_z_radar: Int = 3 // laser measurement dimensions r, phi, r_dot

  private val logger = Logger.getLogger(this.getClass.getName.split("\\$").last)

  // Radar measurement noise standard deviation radius in m
  val std_radr = 0.3

  // Radar measurement noise standard deviation angle in rad
  val std_radphi = 0.03

  // Radar measurement noise standard deviation radius change in m/s
  val std_radrd = 0.3

  def radarUpdate(measurementPackage: MeasurementPackage, XsigPred: SigmaPoints,
                  x: StateVector, P: CoVarianceMatrix): (StateVector, CoVarianceMatrix) = {
    // Measurement
    val z = DenseVector(measurementPackage.raw_measurements.toArray)
    val n_z = n_z_radar
    assert(z.length == n_z, "Radar measurement dimensions aren't valid")

    val (zPred: StateVector, mS: CoVarianceMatrix) = predictRadarMeasurement(n_x, n_aug, n_z, XsigPred)

    val Zsig: SigmaPoints = sigmaPointsRadarMeasurementSpace(n_z, n_aug, XsigPred)

    logger.debug("n_z: " + n_z + " zPred: " + zPred + "z: " + z)
    super.UpdateState(n_x, n_aug, n_z, XsigPred, x, P, Zsig, zPred, mS, z)

  }

  private def predictRadarMeasurement(n_x: Int, n_aug: Int, n_z: Int, XsigPred: SigmaPoints): (StateVector, CoVarianceMatrix) = {
    val Zsig: SigmaPoints = sigmaPointsRadarMeasurementSpace(n_z, n_aug, XsigPred)

    val (zPred, mS) = super.predictMeasurement(n_x, n_aug, n_z, Zsig, XsigPred)

    // add measurement noise
    mS :+= radarNoiseCovarianceMatrix

    (zPred, mS)
  }

  private def radarNoiseCovarianceMatrix: CoVarianceMatrix = {
    DenseMatrix(
      (std_radr * std_radr, 0.0, 0.0),
      (0.0, std_radphi * std_radphi, 0.0),
      (0.0, 0.0, std_radrd * std_radrd)
    )
  }

  private def sigmaPointsRadarMeasurementSpace(n_z: Int, n_aug: Int, XsigPred: SigmaPoints): SigmaPoints = {
    val Zsig: SigmaPoints = DenseMatrix.zeros[Double](n_z, 2 * n_aug + 1)

    for (i <- 0 until 2 * n_aug + 1) {
      // extract values
      val p_x = XsigPred(0, i)
      val p_y = XsigPred(1, i)
      val v = XsigPred(2, i)
      val yaw = XsigPred(3, i)
      val v1 = v * cos(yaw)
      val v2 = v * sin(yaw)

      // measurement model
      Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y)
      Zsig(1, i) = atan2(p_y, p_x)
      if (Zsig(0, i) != 0.0) {
        Zsig(2, i) = (p_x * v1 + p_y * v2) / Zsig(0, i) // r_dot
      } else {
        Zsig(2, i) = 0.0
      }
    }
    Zsig
  }

}
