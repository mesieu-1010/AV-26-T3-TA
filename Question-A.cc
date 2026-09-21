// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

std::int16_t readSigned16LittleEdian(
    const std::array<std::uint8_t, 8> &data,
    int startByte
) {
    // We first extract the first byte at start byte and place on the 8 rightmost positions
    // We shift to the left 8 position, extracting the next byte and use OR operator.
    const std::uint16_t raw = 
        static_cast<std::uint16_t>(data[startByte]) |
        static_cast<std::uint16_t>(data[startByte + 1] << 8);

    return static_cast<std::int16_t> (raw);
}

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    // Since 512 in decimal is 0x200 in hexadecimal
    constexpr unsigned STEER_ACTUATOR_LOG_ID = 0x200;

    std::vector<Row> rows;
    std::ifstream file(path);

    // Cannot read file or file does not exist
    if (!file) {
        return rows;
    }

    bool haveFirstKeptFrame = false;
    double firstKeptTimestamp = 0.0;

    std::string line;
    while(std::getline(file, line)) {
        double timestamp;
        char interfaceName[20];
        char frameToken[100];

        //E.g: Extract (1705638751.000000) vcan0 200#00001400852FB01D
        if (std::sscanf(line.c_str(), "(%lf) %19s %99s", &timestamp,
            interfaceName, frameToken) != 3) {
            continue;
        }

        const std::string frame(frameToken);
        const std::size_t hashPosition = frame.find('#');

        if (hashPosition == std::string::npos) {
            continue;
        }

        // Extract the canId into hexadecimal number
        const unsigned canId = std::stoul(frame.substr(0, hashPosition), 
            nullptr, 16);
        
        // Keep only STEER_ActuatorLog frames
        if (canId != STEER_ACTUATOR_LOG_ID) {
            continue;
        }

        // Extract the hexa metadata
        const std::string hexData = frame.substr(hashPosition + 1);

        // 8 data bytes = 16 hexadecimal characters
        if (hexData.size() != 16) {
            continue;
        }

        std::array<std::uint8_t, 8> data{};
        for (int i{0}; i < 8; ++i) {
            data[i] = static_cast<std::uint8_t>(
                std::stoul(hexData.substr(i * 2, 2), nullptr, 16)
            );
        }

        // MeasuredAngle: 0|16@1- (0.1, 0)
        const std::int16_t measuredRaw = readSigned16LittleEdian(data, 0);
        
        // CmdAngularRate: 16|16@1- (0.1, 0)
        const std::int16_t cmdRaw = readSigned16LittleEdian(data, 2);

        // Physical value = raw value * scale + offset
        // In this problem, scale = 0.1 and offset = 0;
        const double y_measured = measuredRaw * 0.1;
        const double u_commanded = cmdRaw * 0.1;

        if (!haveFirstKeptFrame) {
            firstKeptTimestamp = timestamp;
            haveFirstKeptFrame = true;
        }

        const double t = timestamp - firstKeptTimestamp;
        rows.push_back(Row{t, u_commanded, y_measured});

    }
    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
