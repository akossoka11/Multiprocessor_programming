#include "../include/lodepng.h"
#include "../include/image_functions.h"
#include <functional>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>

// Function to calculate ZNCC for a given window
float calculateZNCC(const std::vector<std::vector<int>>& leftImage, const std::vector<std::vector<int>>& rightImage,
    int x, int y, int d, int winSize) {
    int B = winSize;
    float sumLeft = 0, sumRight = 0, sumLeftSquared = 0, sumRightSquared = 0, sumCross = 0;

    for (int i = 0; i < B; ++i) {
        for (int j = 0; j < B; ++j) {
            int leftPixel = leftImage[y + i][x + j];
            int rightPixel = rightImage[y + i][x + j - d];

            sumLeft += leftPixel;
            sumRight += rightPixel;
            sumLeftSquared += leftPixel * leftPixel;
            sumRightSquared += rightPixel * rightPixel;
            sumCross += leftPixel * rightPixel;
        }
    }

    float meanLeft = sumLeft / (B * B);
    float meanRight = sumRight / (B * B);

    float numerator = sumCross - (sumLeft * sumRight) / (B * B);
    float denominatorLeft = std::sqrt(sumLeftSquared - (sumLeft * sumLeft) / (B * B));
    float denominatorRight = std::sqrt(sumRightSquared - (sumRight * sumRight) / (B * B));

    float znccValue = numerator / (denominatorLeft * denominatorRight);

    return znccValue;
}

// Function for depth estimation using ZNCC
void depthEstimation(const std::vector<std::vector<int>>& leftImage, const std::vector<std::vector<int>>& rightImage,
    int maxDisp, int winSize, std::vector<std::vector<int>>& disparityMap) {
    int height = leftImage.size();
    int width = leftImage[0].size();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float currentMaxSum = -std::numeric_limits<float>::infinity();
            int bestDisparity = 0;

            for (int d = 0; d < maxDisp; ++d) {
                float currentSum = 0;

                for (int winY = 0; winY < winSize; ++winY) {
                    for (int winX = 0; winX < winSize; ++winX) {
                        currentSum += calculateZNCC(leftImage, rightImage, x + winX, y + winY, d, winSize);
                    }
                }

                if (currentSum > currentMaxSum) {
                    currentMaxSum = currentSum;
                    bestDisparity = d;
                }
            }

            disparityMap[y][x] = bestDisparity;
        }
    }
}

std::string extractValue(const std::string& line, const std::string& key) {
    size_t pos = line.find(key);
    if (pos != std::string::npos) {
        pos = line.find("="); // Find the position of '='
        if (pos != std::string::npos) {
            return line.substr(pos + 1); // Extract substring after '='
        }
    }
    return ""; // Return empty string if key not found or '=' not found
}

int searchFileForKeyValuePair(const std::string& keyToFind) {
    std::ifstream calibFile("calib.txt");
    std::string line;
    int foundValue = -1; // Default value if key is not found

    if (calibFile.is_open()) {
        while (std::getline(calibFile, line)) {
            // std::cout << "Debug: Reading line: " << line << std::endl;
            std::string value = extractValue(line, keyToFind);
            if (!value.empty()) {
                foundValue = std::stoi(value);
                break;
            }
        }
        calibFile.close();
    } else {
        std::cout << "Unable to open calib.txt." << std::endl;
        exit(1);
    }

    return foundValue;
}

int findValueForKey(const std::string& keyToFind) {
    int foundValue = searchFileForKeyValuePair(keyToFind);

    if (foundValue == -1) {
        std::cout << "Key '" << keyToFind << "' not found in calib.txt." << std::endl;
        exit(1);
    }

    return foundValue;
}

std::vector<std::vector<int>> createMatrix(int rows, int cols) {
    return std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0));
}

std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> createImages(int winSize) {
    std::vector<std::vector<int>> leftImage = createMatrix(winSize, winSize);
    std::vector<std::vector<int>> rightImage = createMatrix(winSize, winSize);
    return std::make_pair(leftImage, rightImage);
}

void printMatrix(const std::vector<std::vector<int>>& matrix) {
    // For debugging purposes
    for (const auto& row : matrix) {
        for (int value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
}

// Function to profile the execution time of a given function and display the result
template <typename Func, typename... Args>
auto profileFunction(const std::string& functionName, Func&& func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();

    // Call the function
    auto result = std::forward<Func>(func)(std::forward<Args>(args)...);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Execution time of " << functionName << ": " << duration.count() << " ms\n";

    return result;
}

template <typename Func, typename... Args>
void profileWriteImage(const std::string& functionName, Func&& func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();

    // Call the WriteImage function
    std::forward<Func>(func)(std::forward<Args>(args)...);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Execution time of " << functionName << ": " << duration.count() << " ms\n";
}

int main() {
    // Example usage:
    // Assume leftImage, rightImage, and disparityMap are 2D vectors representing grayscale images
    // and the output disparity map respectively.
    // maxDisp is the maximum disparity value (ndisp in calib.txt)
    // winSize is the window size, start with 9x9

    std::string workspaceFolder = getCurrentDirectory();
    // std::cout << "DEBUG Workspace folder: " << workspaceFolder << std::endl;

    // Construct file paths using the workspace folder
    std::string filename = workspaceFolder + "/images/grayscale_image.png";
    std::cout << "Trying to find image from " << filename << std::endl;

    // Load the PNG image using ReadImage function
    //std::vector<unsigned char> image_0; // This will store the raw pixel data
    unsigned width, height;  // Define width and height
    auto gray_image = profileFunction("ReadImage", ReadImage, filename.c_str(), std::ref(width), std::ref(height));

    // Check if the image was successfully loaded
    if (gray_image.empty()) {
        // Handle the error as needed
        std::cout << "Failed to load image." << std::endl;
        return 1;
    }

    std::string maxDispKey = "ndisp";
    int maxDisp = findValueForKey(maxDispKey);
    // std::cout << "DEBUG maxDisp: " << maxDisp << std::endl;

    std::string winSizeKey = "winSize";
    int winSize = findValueForKey(winSizeKey);
    // std::cout << "DEBUG winSize: " << winSize << std::endl;

    std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> images = createImages(winSize);
    std::vector<std::vector<int>> leftImage = images.first;
    std::vector<std::vector<int>> rightImage = images.second;
    std::cout << "DEBUG leftImage:\n" << std::endl;
    printMatrix(leftImage);

    // Initialize disparityMap with zeros
    std::vector<std::vector<int>> disparityMap(leftImage.size(), std::vector<int>(leftImage[0].size(), 0));

    // Call the depth estimation function
    depthEstimation(leftImage, rightImage, maxDisp, winSize, disparityMap);

    // Now disparityMap contains the estimated disparities for each pixel
    return 0;
}