#!/bin/bash
# --------------------------------------------------------------------------
# This script is provided as an example to run the ORAN simulation for
# ML training data collection. It runs the simulation for each of the 4
# configurations (0,1,2,3) and then extracts the dataset (e.g., distances,
# packet loss, etc.) from the repository database.
#
# The simulation executable is assumed to be "oran-lte2lte-rsrp-ml-handover-example"
# and the database, traffic, position, and handover trace files are saved with
# configuration-specific names.
#
# NIST-developed software is provided "AS IS" without warranty.
# --------------------------------------------------------------------------

# Define the base names for trace files and the database
DB_BASE="oran-repository-config"
TRAFFIC_BASE="traffic-trace-config"
POSITION_BASE="position-trace-config"
HANDOVER_BASE="handover-trace-config"
SIM_TIME=10

# The SQL statement that extracts the desired training dataset.
# (This example uses the "nodeapploss" and "nodelocation" tables as in the original script.)
SQL_STATEMENT="SELECT
  ue1loss.simulationtime,
  ue1loss.nodeid, SQRT(POWER(ue1loc.x - 0, 2) + POWER(ue1loc.y - 0, 2)) AS dist1,
  SQRT(POWER(ue1loc.x - 265, 2) + POWER(ue1loc.y - 0, 2)) AS dist2, ue1loss.loss,
  ue2loss.nodeid, SQRT(POWER(ue2loc.x - 0, 2) + POWER(ue2loc.y - 0, 2)) AS dist1,
  SQRT(POWER(ue2loc.x - 265, 2) + POWER(ue2loc.y - 0, 2)) AS dist2, ue2loss.loss,
  ue3loss.nodeid, SQRT(POWER(ue3loc.x - 0, 2) + POWER(ue3loc.y - 0, 2)) AS dist1,
  SQRT(POWER(ue3loc.x - 265, 2) + POWER(ue3loc.y - 0, 2)) AS dist2, ue3loss.loss,
  ue4loss.nodeid, SQRT(POWER(ue4loc.x - 0, 2) + POWER(ue4loc.y - 0, 2)) AS dist1,
  SQRT(POWER(ue4loc.x - 265, 2) + POWER(ue4loc.y - 0, 2)) AS dist2, ue4loss.loss,
  (ue1loss.loss + ue2loss.loss + ue3loss.loss + ue4loss.loss) / 4.0 AS totloss
FROM
    nodeapploss AS ue1loss
      INNER JOIN nodelocation AS ue1loc ON ue1loc.nodeid = ue1loss.nodeid AND ue1loss.simulationtime = ue1loc.simulationtime
    INNER JOIN nodeapploss AS ue2loss ON ue2loss.nodeid = 2 AND ue2loss.simulationtime = ue1loss.simulationtime
      INNER JOIN nodelocation AS ue2loc ON ue2loc.nodeid = 2 AND ue2loc.simulationtime = ue1loc.simulationtime
    INNER JOIN nodeapploss AS ue3loss ON ue3loss.nodeid = 3 AND ue3loss.simulationtime = ue1loss.simulationtime
      INNER JOIN nodelocation AS ue3loc ON ue3loc.nodeid = 3 AND ue3loc.simulationtime = ue1loc.simulationtime
    INNER JOIN nodeapploss AS ue4loss ON ue4loss.nodeid = 4 AND ue4loss.simulationtime = ue1loss.simulationtime
      INNER JOIN nodelocation AS ue4loc ON ue4loc.nodeid = 4 AND ue4loc.simulationtime = ue1loc.simulationtime
WHERE
  ue1loss.nodeid = 1
ORDER BY
  ue1loss.simulationtime ASC;"

# Path to the sqlite3 executable (if not in PATH, adjust accordingly)
SQLITE=sqlite3

# Run simulations for configurations 0, 1, 2, and 3
for config in {0..3}; do
    echo "Running simulation for configuration ${config}..."
    ./ns3 run "oran-lte-2-lte-ml-handover-example \
      --use-oran=true \
      --use-distance-lm=false \
      --use-onnx-lm=false \
      --use-torch-lm=false \
      --start-config=${config} \
      --db-file=${DB_BASE}${config}.db \
      --traffic-trace-file=${TRAFFIC_BASE}${config}.tr \
      --position-trace-file=${POSITION_BASE}${config}.tr \
      --handover-trace-file=${HANDOVER_BASE}${config}.tr \
      --sim-time=${SIM_TIME}"
done

# Extract data from each configuration's database and write to CSV files
for config in {0..3}; do
    output_file="config${config}.csv"
    echo "Extracting data from ${DB_BASE}${config}.db to ${output_file}..."
    echo "${SQL_STATEMENT}" | ${SQLITE} ${DB_BASE}${config}.db | sed 's/|/ /g' | awk -v config=${config} '{print $0, config}' > ${output_file}
done

# Optionally, combine all CSV files into one master training file:
cat config0.csv config1.csv config2.csv config3.csv > training_data.csv
echo "Training data saved in training_data.csv"

# End of script

