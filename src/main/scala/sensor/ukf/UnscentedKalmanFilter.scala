package sensor.ukf

import breeze.linalg.{DenseMatrix, DenseVector}
import breeze.numerics.{cos, log, sin}
import org.apache.log4j.Logger
import sensor.models.MeasurementPackage
import sensor.ukf.UKFTypes.{CoVarianceMatrix, SigmaPoints, StateVector}


/**
  * Created by nick on 25/3/17.
  */
class UnscentedKalmanFilter extends UKFPredict with LaserUpdate with RadarUpdate  {
  val logger = Logger.getLogger(this.getClass.getName.split("\\$").last)

  val LASER = "L"
  val RADAR = "R"

  var previous_timestamp: Long = 0

  // initial covariance matrix
  var P:CoVarianceMatrix = DenseMatrix(
    (1.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, 1.0))

  var x:StateVector = DenseVector.zeros[Double](n_x)

  def initStateLaserMeasurement(measurementPackage: MeasurementPackage): StateVector = {
    val x:StateVector = DenseVector.zeros[Double](n_x)

    x(0) = measurementPackage.raw_measurements(0) // px
    x(1) = measurementPackage.raw_measurements(1) // py
    x
  }

  def initStateRadarMeasurement(measurementPackage: MeasurementPackage): StateVector = {
    val x:StateVector = DenseVector.zeros[Double](n_x)

    val rho = measurementPackage.raw_measurements(0) // Range - radial distance from origin
    val phi = measurementPackage.raw_measurements(1) // Bearing - angle between rho and x
    val rhod = measurementPackage.raw_measurements(2) // Radial Velocity - change of p (range rate)

    val px = rho * cos(phi) // metres
    val py = rho * sin(phi)
    val v = rhod // metres/sec
    val yaw = phi // radians
    val yawd = 0.0 // radians/sec

    x(0) = px
    x(1) = py
    x(2) = v
    x(3) = yaw
    x(4) = yawd

    x
  }

  def processMeasurement(measurementPackage: MeasurementPackage): StateVector = {

    // initialisation
    if (!isInitialised) {

      logger.info("UKF Initialise")
      x = measurementPackage.sensor_type match {
        case LASER => initStateLaserMeasurement(measurementPackage)
        case RADAR => initStateRadarMeasurement(measurementPackage)
      }

      previous_timestamp = measurementPackage.timestamp

      isInitialised=true

      logger.debug("init x: " + x)
      logger.debug("init P: " + P)
      return x
    }

    // work out delta time in secs and store the previous
    val deltaTime:Double = (measurementPackage.timestamp - previous_timestamp) / 1000000.0
    previous_timestamp = measurementPackage.timestamp


    logger.debug("predict x: " + x + " deltaTime: " + deltaTime)
    logger.debug("predict P: " + P)
    // prediction - scala matching needs lowercase first letter
    val (pXsigPred, px, pP) = prediction(x, P, deltaTime)

    // update
    val (ux, uP) = measurementPackage.sensor_type match {
      case LASER => laserUpdate(measurementPackage, pXsigPred, px, pP)
      case RADAR => radarUpdate(measurementPackage, pXsigPred, px, pP)
    }

    // store updated state vector and covariance matrix
    x = ux
    P = uP

    logger.debug("x: " + x)
    logger.debug("P: " + P)

    x
  }
}
