#include "lib/util.hpp"

#include <iostream>
#include <filesystem>
#include <fstream>

namespace util {
    float degrees(float radians) {
        return radians*(180/M_PI);
    }

    float radians(float degrees) {
        return degrees*(M_PI/180);
    }

    float clamp(float val, float min, float max) {
        return std::max(std::min(val, max), min);
    }

    float calculate_shortest_angle(float target, float current) {
        // Determine the larger and smaller angles
        float b = std::max(target, current);
        float s = std::min(target, current);

        // Calculate the difference
        float diff = b - s;

        // Calculate the shortest angular difference
        float shortest_diff = (diff <= 180) ? diff : (360 - b) + s;

        // Directional multiplier function (determines whether to turn clockwise or counterclockwise)
        auto dir_to_spin = [](float target, float current) {
            // If the angular difference is less than 180 degrees, turn clockwise, otherwise counterclockwise
            return (fmod((target - current + 360), 360) < 180) ? 1 : -1;
        };

        // Apply the directional multiplier
        return shortest_diff * dir_to_spin(target, current);
    }  

    float get_angle_to_target(float robotX, float robotY, float targetX, float targetY) {
        float deltaX = targetX - robotX;
        float deltaY = targetY - robotY;

        return fmod(degrees(atan2(deltaY, deltaX)), 360);
    }

    float no_big_angles_pls(float theta) {
        return theta - 2*M_PI*floor((theta+M_PI)/(2*M_PI));
    }

    int sign(double value) {
        return (value > 0) - (value < 0);
    }

    float cheap_norm_pdf(const float x) {
        // Approximation of the standard normal PDF
        // Coefficients for the rational approximation
        const float a = 0.3989422804014337;
        const float e = 0.59422804014337;

        // Compute the approximate normal PDF using a rational polynomial
        const float pdfApprox = a / (1.0 + e * x * x * x * x);

        return pdfApprox;
    }

    float avg(std::vector<float> values) {
        float sum = 0;
        for (float value : values) { sum += value; }
        return sum / values.size();
    }

    // Function to trim whitespace from a string
    std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t\n\r");
        size_t last = str.find_last_not_of(" \t\n\r");
        return (first == std::string::npos || last == std::string::npos) ? "" : str.substr(first, last - first + 1);
    }

    // Function to extract a value from a key-value pair in JSON-like format
    std::string extractValue(const std::string& line, const std::string& key) {
        size_t keyPos = line.find(key);
        if (keyPos != std::string::npos) {
            size_t colonPos = line.find(":", keyPos);
            size_t commaPos = line.find(",", colonPos);
            size_t endPos = (commaPos == std::string::npos) ? line.find("}", colonPos) : commaPos;
            std::string value = line.substr(colonPos + 1, endPos - colonPos - 1);
            return trim(value);
        }
        return "";
    }

    // Function to parse control points from the JSON file
    std::vector<bezier::Point> parseBezierControlPoints(const std::string& filePath) {
        std::ifstream file(filePath);
        if (!file) {
            throw std::runtime_error("Error: Could not open the JSON file!");
        }

        std::vector<bezier::Point> controlPoints;
        std::string line;
        bool inControlsSection = false;

        while (std::getline(file, line)) {
            line = trim(line);

            // Detect start of controls section
            if (line.find("\"controls\"") != std::string::npos) {
                inControlsSection = true;
                continue;
            }

            // Detect end of controls section
            if (inControlsSection && line.find("]") != std::string::npos) {
                inControlsSection = false;
                continue;
            }

            // Parse control points within the controls section
            if (inControlsSection && line.find("{") != std::string::npos) {
                bezier::Point cp;

                // Parse properties
                std::getline(file, line); cp.x = std::stod(extractValue(line, "\"x\""));
                std::getline(file, line); cp.y = std::stod(extractValue(line, "\"y\""));

                controlPoints.push_back(cp);
            }
        }

        return controlPoints;
    }

    std::string extractValueFromString(const std::string& line, const std::string& key) {
        size_t keyPos = line.find(key);
        if (keyPos != std::string::npos) {
            size_t colonPos = line.find(":", keyPos);
            size_t commaPos = line.find(",", colonPos);
            size_t endPos = (commaPos == std::string::npos) ? line.find("}", colonPos) : commaPos;
            std::string value = line.substr(colonPos + 1, endPos - colonPos - 1);
            value.erase(remove(value.begin(), value.end(), '\"'), value.end()); // Remove quotes
            return value;
        }
        return "";
    }

    // Function to parse control points from a JSON-like string
    std::vector<bezier::Point> parseControlPointsFromString(const std::string& jsonString) {
        std::vector<bezier::Point> controlPoints;
        std::istringstream stream(jsonString);
        std::string line;
        bool inControlsSection = false;

        while (std::getline(stream, line)) {
            // Detect start of controls section
            if (line.find("\"controls\"") != std::string::npos) {
                inControlsSection = true;
                continue;
            }

            // Detect end of controls section
            if (inControlsSection && line.find("]") != std::string::npos) {
                inControlsSection = false;
                continue;
            }

            // Parse control points within the controls section
            if (inControlsSection && line.find("{") != std::string::npos) {
                bezier::Point cp;

                // Parse properties
                std::getline(stream, line); cp.x = std::stod(extractValueFromString(line, "\"x\""));
                std::getline(stream, line); cp.y = std::stod(extractValueFromString(line, "\"y\""));

                controlPoints.push_back(cp);
            }
        }

        return controlPoints;
    }
}