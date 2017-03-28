#!/usr/bin/env bash

while read measurement; do
  $KAFKA_INSTALL/bin/kafka-console-producer.sh --broker-list localhost:9092 --topic sensor-data "$measurement"
done  < ../data/sample-laser-radar-measurement-data-2.txt
