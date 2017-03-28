package sensor.ukf

import org.apache.log4j.Logger
import org.apache.spark.sql.functions._
import org.apache.spark.sql.{DataFrame, SparkSession}
import sensor.models._
import org.apache.spark.ml.linalg.{Vector, Vectors}
import org.apache.spark.sql.catalyst.expressions.GenericRowWithSchema

/**
  * Created by nick on 21/3/17.
  */
object UnscentedKalmanFilterStreamProcessor {
  val logger = Logger.getLogger(UnscentedKalmanFilterStreamProcessor.getClass.getName.split("\\$").last)

  val spark = SparkSession
    .builder
    .master("local")
    .appName("UnscentedKarmanFilterStreamProcessor")
    .getOrCreate()

  val ukf = new UnscentedKalmanFilter

  import spark.implicits._

  def main(args: Array[String]) {
    logger.debug("start")

    // define UDFs for extract

    val device = (a: Array[Byte]) => a(0).toString
    val deviceUDF = udf(device)

    val lidarUDF = udf { s: String =>

      // If lidar return new Lidar Measurement else null
      s(0) match {
        case 'L' => {
          val l_vals = s.split("\t").slice(1, 4)
          val (px: Double, py: Double, timestamp: Long) = (l_vals(0).toDouble, l_vals(1).toDouble, l_vals(2).toLong)
          LidarMeasurement(px, py, timestamp)
        }
        case 'R' => null
      }
    }

    val radarUDF = udf { s: String =>
      // if radar return new radar measurement else null
      s(0) match {
        case 'R' => {
          val r_vals = s.split("\t").slice(1, 5)
          val (rho: Double, theta: Double, rho_dot: Double, timestamp: Long) =
            (r_vals(0).toDouble, r_vals(1).toDouble, r_vals(2).toDouble, r_vals(3).toLong)
          RadarMeasurement(rho, theta, rho_dot, timestamp)
        }
        case 'L' => null
      }
    }

    val groundTruthUDF = udf { s: String => {
      val vals = s(0) match {
        case 'R' => s.split("\t").slice(5, 9)
        case 'L' => s.split("\t").slice(4, 8)
      }
      val (px: Double, py: Double, vx: Double, vy: Double) =
        (vals(0).toDouble, vals(1).toDouble, vals(2).toDouble, vals(3).toDouble)
      GroundTruth(px, py, vx, vy)
    }
    }

    val timestampUDF = udf { s: String =>
      s(0) match {
        case 'R' => s.split("\t").slice(4, 5)(0).toLong
        case 'L' => s.split("\t").slice(3, 4)(0).toLong
      }
    }

    val measurementUDF = udf { s: String => {
      val sensor_type = s(0).toString
      val timestamp = s(0) match {
        case 'R' => s.split("\t").slice(4, 5)(0).toLong
        case 'L' => s.split("\t").slice(3, 4)(0).toLong
      }
      val raw_measurements = Vectors.dense(s(0) match {
        case 'L' => s.split("\t").slice(1, 3).map(_.toDouble)
        case 'R' => s.split("\t").slice(1, 4).map(_.toDouble)
      }
      )
      MeasurementPackage(raw_measurements, timestamp, sensor_type)
    }
    }

    val ukfUDF = udf { mp_row: GenericRowWithSchema => {
      val raw_measurements = mp_row.getAs[Vector]("raw_measurements")
      val timestamp = mp_row.getAs[Long]("timestamp")
      val sensor_type = mp_row.getAs[String]("sensor_type")

      val mp = MeasurementPackage(raw_measurements, timestamp, sensor_type)
      val x = ukf.processMeasurement(mp)
      logger.debug("ukfUDF raw_measurements: " + raw_measurements + " x:" + x)
      Estimates(Vectors.dense(x.toArray))
    }
    }

    val kafka_df: DataFrame = spark
      .readStream
      .format("kafka")
      .option("kafka.bootstrap.servers", "localhost:9092")
      .option("subscribe", "sensor-data")
      .option("failOnDataLoss", "false")
            .option("startingOffsets", "earliest") // read data from the start of the stream
//      .option("startingOffsets", "latest") // read data from the end of the stream
      .load()


    val sensor_df = kafka_df.selectExpr("CAST(value as string)", "CAST(topic as string)")
      .as[(String, String)]
      //      .withColumn("device", deviceUDF('value))
      .withColumn("timestamp", timestampUDF('value))
      //      .withColumn("lidar", lidarUDF('value))
      //      .withColumn("radar", radarUDF('value))
      .withColumn("ground_truth", groundTruthUDF('value))
      .withColumn("measurement", measurementUDF('value))

    //    sensor_df.printSchema()

    val ukf_df = sensor_df.select($"measurement", $"ground_truth")
      .withColumn("estimates", ukfUDF('measurement))

    ukf_df.printSchema()

    // csv extract cant handle nested structures in columns
    val xValUDF = udf { (x: Vector, index: Int) => x(index) }

    val measurementValUDF = udf { (mp_row: GenericRowWithSchema, index: Int) => {
      val raw_measurements = mp_row.getAs[Vector]("raw_measurements")
      val sensor_type = mp_row.getAs[String]("sensor_type")

      val ps = sensor_type match {
        // Radar convert to px, py
        case "R" => {
          val rho = raw_measurements(0)
          val phi = raw_measurements(1)
          Array(rho * Math.cos(phi), rho * Math.sin(phi))
        }
        // Laser use px, py
        case "L" => Array(raw_measurements(0), raw_measurements(1))
      }
      ps(index)
    }
    }

    val extract_df = ukf_df.selectExpr("estimates.x as x", "measurement",
      "ground_truth.px as px_gt", "ground_truth.py as py_gt")
      .as[(Vector, MeasurementPackage, Double, Double)]
      .withColumn("px", xValUDF('x, lit(0)))
      .withColumn("py", xValUDF('x, lit(1)))
      .withColumn("vel", xValUDF('x, lit(2)))
      .withColumn("yaw", xValUDF('x, lit(3)))
      .withColumn("yaw_rate", xValUDF('x, lit(4)))
      .withColumn("p1", measurementValUDF('measurement, lit(0)))
      .withColumn("p2", measurementValUDF('measurement, lit(1)))


    val output_df = extract_df.select($"px", $"py", $"vel", $"yaw", $"yaw_rate", $"p1", $"p2", $"px_gt", $"py_gt")

    //    val query = sensor_df.writeStream
    //      .outputMode("append")
    //      .format("console")
    ////      .start()
    //
    //    query.awaitTermination()

    //    val ukf_query = ukf_df.writeStream
    //    .outputMode("append")
    //    .format("console")
    //    .start()


//    val console_output =  output_df.writeStream
//      .format("console")
//      .start()

    val output_query = output_df.writeStream
      .outputMode("append")
            .format("csv")
            .option("checkpointLocation", "/tmp")
            .start("/tmp/output")

    //    ukf_query.awaitTermination()
    output_query.awaitTermination()
//    console_output.awaitTermination()

    logger.debug("finish")
  }

}
