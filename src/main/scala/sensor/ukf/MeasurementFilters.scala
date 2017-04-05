package sensor.ukf

import sensor.ukf.UKFTypes._
import breeze.linalg._
import breeze.numerics._
import org.apache.commons.math3.util.MathUtils
import sensor.models.MeasurementPackage

/**
  * Created by nick on 25/3/17.
  */

trait UKFVariables {
  val n_x: Int = 5 // state dimensions
  val n_aug: Int = 7 // augmented state dimensions

  val std_a: Double = 0.45 // Process noise standard deviation longitudinal acceleration in m/s^2
  val std_yawdd: Double = 0.55 // Process noise standard deviation yaw acceleration in rad/s^2

  val lambda: Double = 3 - n_aug //define spreading parameter

  def sigmaPointsWeights: DenseVector[Double] = {
    val weight_0 = lambda / (lambda + n_aug)
    val weight = 0.5 / (lambda + n_aug)

    val weights = DenseVector.fill(2 * n_aug + 1){weight}
    weights(0) = weight_0
    weights
  }

}

trait UKFPredict extends UKFVariables{

  var isInitialised = false

  // predict
  def prediction(x:StateVector, P:CoVarianceMatrix, deltaT: Double): (SigmaPoints, StateVector, CoVarianceMatrix) = {
    val XSigAug: SigmaPoints = augmentedSigmaPoints(n_x, n_aug, std_a, std_yawdd, x, P)
    val XSigPred: SigmaPoints = predictSigmaPoints(n_x, n_aug, deltaT, XSigAug)
    predictMeanAndCovariance(n_x, n_aug, XSigPred)
  }

  protected def augmentedSigmaPoints(n_x: Int, n_aug: Int, std_a: Double, std_yawdd: Double, x: StateVector, P: CoVarianceMatrix): SigmaPoints = {
    //create augmented mean state
    val xAug:StateVector = DenseVector(x.toArray ++ Array(0.0,0.0))

    //create augmented covariance matrix
    val PAug:CoVarianceMatrix = DenseMatrix.zeros[Double](n_aug,n_aug)
    PAug(0 to 4, 0 to 4) := P
    PAug(5, 5) = std_a * std_a
    PAug(6, 6) = std_yawdd * std_yawdd

    // create square root matrix
    val L = cholesky(PAug.toDenseMatrix)

    // temp breeze matrix for sigma points
    val XsigAug:SigmaPoints = DenseMatrix.zeros[Double](n_aug, 2 * n_aug + 1)

    // create augmented sigma points
    XsigAug(::,0) :=  xAug
    for (i <- 0 until n_aug) {
      XsigAug(::,i+1) := xAug + math.sqrt(lambda + n_aug) * L(::,i)
      XsigAug(::,i+1+n_aug) := xAug - math.sqrt(lambda + n_aug) * L(::,i)
    }

    XsigAug
  }

  protected def predictSigmaPoints(n_x: Int, n_aug: Int, deltaT: Double, XsigAug: SigmaPoints): SigmaPoints = {
    val XsigPred:SigmaPoints = DenseMatrix.zeros[Double](n_aug, 2 * n_aug + 1)

    for (i <- 0 until 2 * n_aug +1) {
      //extract values for better readability//extract values for better readability
      val p_x = XsigAug(0, i)
      val p_y = XsigAug(1, i)
      val v = XsigAug(2, i)
      val yaw = XsigAug(3, i)
      val yawd = XsigAug(4, i)
      val nu_a = XsigAug(5, i)
      val nu_yawdd = XsigAug(6, i)

      //predicted state values//predicted state values
      var px_p = 0.0
      var py_p = 0.0

      //avoid division by zero
      if (abs(yawd) > 0.001) {
        px_p = p_x + v / yawd * (sin(yaw + yawd * deltaT) - sin(yaw))
        py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * deltaT))
      }
      else {
        px_p = p_x + v * deltaT * cos(yaw)
        py_p = p_y + v * deltaT * sin(yaw)
      }

      var v_p = v
      var yaw_p = yaw + yawd * deltaT
      var yawd_p = yawd

      //add noise
      px_p = px_p + 0.5 * nu_a * deltaT * deltaT * cos(yaw)
      py_p = py_p + 0.5 * nu_a * deltaT * deltaT * sin(yaw)
      v_p = v_p + nu_a * deltaT

      yaw_p = yaw_p + 0.5 * nu_yawdd * deltaT * deltaT
      yawd_p = yawd_p + nu_yawdd * deltaT

      //write predicted sigma point into right column
      XsigPred(0, i) = px_p
      XsigPred(1, i) = py_p
      XsigPred(2, i) = v_p
      XsigPred(3, i) = yaw_p
      XsigPred(4, i) = yawd_p
    }

    XsigPred
  }

  protected def predictMeanAndCovariance(n_x: Int, n_aug: Int, XsigPred: SigmaPoints):
  (SigmaPoints, StateVector, CoVarianceMatrix) = {
    //create vector for predicted state
    var x: StateVector = DenseVector.zeros[Double](n_x)

    //create covariance matrix for prediction
    var P: CoVarianceMatrix = DenseMatrix.zeros[Double](n_x, n_x)

    //create vector for weights
    val weights = sigmaPointsWeights

    //predicted state mean
    for (i <- 0 until 2 * n_aug + 1) { //iterate over sigma points
      x :+= weights(i) * XsigPred(0 to 4, i)
    }

    //predicted state covariance matrix
    for (i <- 1 until 2 * n_aug + 1) { //iterate over sigma points

      // state difference
//      val x_diff = XsigPred(0 to 4, i) - x
      val x_diff = XsigPred(0 to 4, i) - XsigPred(0 to 4, 0)

      //angle normalization
      x_diff(3) = MathUtils.normalizeAngle(x_diff(3), 0.0)
      P :+= weights(i) * x_diff * x_diff.t
    }
    (XsigPred, x, P)
  }
}

trait UKFUpdate extends UKFVariables{

  // will be overriden by the measurement device
//  def update(measurementPackage: MeasurementPackage, XsigPred:SigmaPoints, x:StateVector, P:CoVarianceMatrix)
//  : (StateVector, CoVarianceMatrix)
//
//  def noiseCovarianceMatrix: CoVarianceMatrix

  /*
   * Predict Measurement
   */
  protected def predictMeasurement(n_x:Int, n_aug:Int, n_z:Int,
                                   Zsig:SigmaPoints, XsigPred:SigmaPoints): (StateVector, CoVarianceMatrix) = {

    val weights = sigmaPointsWeights

    val zPred = meanPredictedMeasurement(n_z, n_aug, weights, Zsig)

    val S = measurementCovarianceMatrixS(n_z, n_aug, Zsig, zPred, weights)

    (zPred,S)
  }

  protected def meanPredictedMeasurement(n_z:Int, n_aug:Int, weights:WeightsVector, Zsig:SigmaPoints): StateVector = {
    val zPred = DenseVector.zeros[Double](n_z)

    for (i <- 0 until 2 * n_aug + 1) {
      zPred :+= weights(i) * Zsig(::,i)
    }

    zPred
  }

  protected def measurementCovarianceMatrixS(n_z:Int, n_aug:Int, Zsig:SigmaPoints,
                                             zPred:StateVector, weights:WeightsVector): CoVarianceMatrix = {
    val S = DenseMatrix.zeros[Double](n_z, n_z)

    for (i <- 1 until 2 * n_aug + 1) {
//      val zDiff =  Zsig(::,i) - zPred
      val zDiff =  Zsig(::,i) - Zsig(::,0)
      zDiff(1) = MathUtils.normalizeAngle(zDiff(1), 0.0)
      S :+= weights(i) * zDiff * zDiff.t
    }
    S
  }



  /*
   * Update State
   */
  protected def UpdateState(n_x:Int, n_aug:Int, n_z:Int, XsigPred:SigmaPoints, x:StateVector, P:CoVarianceMatrix,
                            Zsig:SigmaPoints, zPred:StateVector, S:CoVarianceMatrix,
                            z:StateVector): (StateVector, CoVarianceMatrix) = {

    val weights = sigmaPointsWeights

    val Tc = crossCorrelationMatrix(n_x, n_aug, n_z, Zsig, zPred, XsigPred, x, weights)

    // Kalman gain K
    val K = Tc * inv(S)
    
    // residual
    val zDiff = z - zPred

    val xUpdate:StateVector = x + K * zDiff
    val PUpdate:CoVarianceMatrix = P - K * S * K.t

    (xUpdate, PUpdate)
  }

  protected def crossCorrelationMatrix(n_x:Int, n_aug:Int, n_z:Int, Zsig:SigmaPoints,
                                       zPred:StateVector, XsigPred:SigmaPoints,
                                       x:StateVector, weights:WeightsVector): CrossCorrelationMatrix = {
    val Tc = DenseMatrix.zeros[Double](n_x, n_z)

    for (i <- 1 until 2 * n_aug + 1) {
//      val zDiff =  Zsig(::,i) - zPred
      val zDiff =  Zsig(::,i) - Zsig(::,0)
      zDiff(1) = MathUtils.normalizeAngle(zDiff(1), 0.0)
//      val x_diff = XsigPred(0 to 4, i) - x
      val x_diff = XsigPred(0 to 4, i) - XsigPred(0 to 4, 0)
      x_diff(3) = MathUtils.normalizeAngle(x_diff(3), 0.0)
      Tc :+= weights(i) * x_diff * zDiff.t
    }
    Tc
  }

}





