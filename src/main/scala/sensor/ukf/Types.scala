package sensor.ukf

import breeze.linalg._

/**
  * Created by nick on 25/3/17.
  */

object UKFTypes {

  type StateVector = DenseVector[Double]
  type CoVarianceMatrix = DenseMatrix[Double]
  type SigmaPoints = DenseMatrix[Double]
  type WeightsVector = DenseVector[Double]
  type CrossCorrelationMatrix = DenseMatrix[Double]
}
