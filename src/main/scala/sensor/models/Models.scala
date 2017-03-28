package sensor.models

import org.apache.spark.ml.linalg.Vector

/**
  * Created by nick on 23/3/17.
  */

case class LidarMeasurement
(
  px: Double,
  py: Double,
  timestamp: Long // in microseconds
)

case class RadarMeasurement
(
  rho: Double,
  theta: Double,
  rho_dot: Double,
  timestamp: Long// in microseconds
)

case class GroundTruth
(
  px: Double,
  py: Double,
  vx: Double,
  vy: Double
)

case class MeasurementPackage
(
  raw_measurements: Vector,
  timestamp: Long,
  sensor_type: String
)

case class Estimates
(
  x: Vector
)
