name := "Unscented-Kalman-Filter-Spark-Scala"

version := "0.1"

scalaVersion := "2.11.8"

lazy val versions = new {
  val spark = "2.1.0"
}
resolvers ++= Seq(
  DefaultMavenRepository,
  Resolver.bintrayRepo("scalaz", "releases"),
  Resolver.typesafeRepo("releases"),
  Resolver.typesafeIvyRepo("releases"),
  Resolver.sonatypeRepo("public")
)

libraryDependencies ++= Seq(
  "org.apache.spark" %% "spark-core" % versions.spark,
  "org.apache.spark" %% "spark-sql" % versions.spark,
  "org.apache.spark" %% "spark-sql-kafka-0-10" % versions.spark,
  "org.apache.spark" %% "spark-mllib" % versions.spark
)
