#include "../include/lodepng.h"
#include "../include/image_functions.h"
#include <algorithm>
#include <limits>
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
    float B = static_cast<float>(winSize);
    float sumLeft = 0, sumRight = 0, sumLeftSquared = 0, sumRightSquared = 0, sumCross = 0;

    for (int i = 0; i < winSize; ++i) {
        for (int j = 0; j < winSize; ++j) {
            float leftPixel = static_cast<float>(leftImage[static_cast<size_t>(y + i)][static_cast<size_t>(x + j)]);
            float rightPixel = static_cast<float>(rightImage[static_cast<size_t>(y + i)][static_cast<size_t>(x + j - d)]);

            sumLeft += leftPixel;
            sumRight += rightPixel;
            sumLeftSquared += leftPixel * leftPixel;
            sumRightSquared += rightPixel * rightPixel;
            sumCross += leftPixel * rightPixel;
        }
    }

    // float meanLeft = sumLeft / (B * B);
    // float meanRight = sumRight / (B * B);

    float numerator = sumCross - (sumLeft * sumRight) / (B * B);
    float denominatorLeft = std::sqrt(sumLeftSquared - (sumLeft * sumLeft) / (B * B));
    float denominatorRight = std::sqrt(sumRightSquared - (sumRight * sumRight) / (B * B));

    float znccValue = numerator / (denominatorLeft * denominatorRight);

    return znccValue;
}

std::vector<std::vector<int>> depthEstimation(
    const std::vector<std::vector<int>>& image0, 
    const std::vector<std::vector<int>>& image1,
    size_t width, size_t height, int maxDisp, int winSize) {

    std::cout << "Depth estimation in progress." << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<int>> disparityMap(height, std::vector<int>(width, 0));
    std::cout << "DEBUG disparityMap height: " << disparityMap.size() << " and width: " << disparityMap[0].size() << std::endl;

    int h = static_cast<int>(disparityMap.size());
    int w = static_cast<int>(disparityMap[0].size());

    std::cout << "DEBUG h x w: " << h << " x " << w << std::endl;

    for (int y = 0; y < (h - winSize); ++y) {
        for (int x = 0; x < (w - winSize); ++x) {
            float currentMaxSum = -std::numeric_limits<float>::infinity();
            int bestDisparity = 0;

            for (int d = 0; d < maxDisp; ++d) {
                float currentSum = 0;

                /*
                for (int winY = 0; winY < winSize; ++winY) {
                    for (int winX = 0; winX < winSize; ++winX) {
                        currentSum += calculateZNCC(image0, image1, x + winX, y + winY, d, winSize);
                    }
                }
                */

               currentSum += calculateZNCC(image0, image1, x, y, d, winSize);

                if (currentSum > currentMaxSum) {
                    currentMaxSum = currentSum;
                    bestDisparity = d;
                }
            }

            // size_t y_index = static_cast<size_t>(y);
            // size_t x_index = static_cast<size_t>(x);

            disparityMap[static_cast<size_t>(y)][static_cast<size_t>(x)] = bestDisparity;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Execution time of depthEstimation: " << duration.count() << " ms" << std::endl;

    return disparityMap;
}

/*
void crossCheck(
    const std::vector<std::vector<int>>& leftDisparityMap, const std::vector<std::vector<int>>& rightDisparityMap, 
    std::vector<std::vector<int>>& consolidatedDisparityMap, int threshold) {

    int height = leftDisparityMap.size();
    int width = leftDisparityMap[0].size();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int leftDisparity = leftDisparityMap[y][x];
            int rightDisparity = rightDisparityMap[y][x];

            // Check if the disparities are consistent within the threshold
            if (std::abs(leftDisparity - rightDisparity) <= threshold) {
                // Use the average of the disparities as the consolidated value
                consolidatedDisparityMap[y][x] = (leftDisparity + rightDisparity) / 2;
            } else {
                // If the disparities are not consistent, set the consolidated disparity to zero
                consolidatedDisparityMap[y][x] = 0;
            }
        }
    }
}

// Save the output image after applying CrossCheck post processing and submit it along with the report

void occlusionFill(const std::vector<std::vector<int>>& inputDisparityMap, std::vector<std::vector<int>>& outputDisparityMap) {
    int height = inputDisparityMap.size();
    int width = inputDisparityMap[0].size();

    // Initialize the output disparity map with the input disparity map
    outputDisparityMap = inputDisparityMap;

    // Iterate over each pixel in the disparity map
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // If the pixel value is zero (indicating occlusion), find the nearest non-zero value
            if (inputDisparityMap[y][x] == 0) {
                int nearestNonZero = std::numeric_limits<int>::max(); // Initialize with a large value
                // Iterate over a window around the current pixel to find the nearest non-zero value
                for (int offsetY = -1; offsetY <= 1; ++offsetY) {
                    for (int offsetX = -1; offsetX <= 1; ++offsetX) {
                        int neighborY = y + offsetY;
                        int neighborX = x + offsetX;
                        // Check if the neighbor pixel is within bounds and non-zero
                        if (neighborY >= 0 && neighborY < height && neighborX >= 0 && neighborX < width &&
                            inputDisparityMap[neighborY][neighborX] != 0) {
                            nearestNonZero = std::min(nearestNonZero, inputDisparityMap[neighborY][neighborX]);
                        }
                    }
                }
                // Update the current pixel with the nearest non-zero value
                outputDisparityMap[y][x] = nearestNonZero;
            }
        }
    }
}

// Save the final output image after applying OcclusionFill post processing and submit it along with the report
*/

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

void printMatrix(const std::vector<std::vector<int>>& matrix) {
    // For debugging purposes
    for (const auto& row : matrix) {
        for (int value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
}

void printVector(const std::vector<unsigned char>& vec) {
    for (unsigned char value : vec) {
        std::cout << static_cast<int>(value) << " " << std::endl;
    }
    // std::cout << std::endl;
}

// Function to profile the execution time of a given function and display the result
template <typename Func, typename... Args>
auto profileFunction(const std::string& functionName, Func&& func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();

    // Call the function
    auto result = std::forward<Func>(func)(std::forward<Args>(args)...);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Execution time of " << functionName << ": " << duration.count() << " ms" << std::endl;

    return result;
}

template <typename Func, typename... Args>
void profileWriteImage(const std::string& functionName, Func&& func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();

    // Call the WriteImage function
    std::forward<Func>(func)(std::forward<Args>(args)...);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Execution time of " << functionName << ": " << duration.count() << " ms" << std::endl;
}

std::vector<std::vector<int>> vectorToMatrix(const std::vector<unsigned char>& image, size_t width, size_t height) {
    std::vector<std::vector<int>> matrix(height, std::vector<int>(width, 0));

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            // Calculate the index in the 1D vector corresponding to the current pixel
            size_t index = y * width + x;
            // Assign the pixel value to the corresponding position in the matrix
            matrix[y][x] = static_cast<int>(image[index]);
        }
    }

    return matrix;
}

std::vector<unsigned char> matrixToVector(const std::vector<std::vector<int>>& matrixImage) {
    std::vector<unsigned char> vectorImage;
    for (const auto& row : matrixImage) {
        for (int value : row) {
            vectorImage.push_back(static_cast<unsigned char>(value));
        }
    }
    return vectorImage;
}

int main() {
    // Example usage:
    // Assume leftImage, rightImage, and disparityMap are 2D vectors representing grayscale images
    // and the output disparity map respectively.
    // maxDisp is the maximum disparity value (ndisp in calib.txt).
    // winSize is the window size, start with 9x9 (winSize in calib.txt).

    std::string workspaceFolder = getCurrentDirectory();

    // Construct file paths using the workspace folder
    // Used phase2task2 converter to construct left and right grayscale images from im0 and im1 respectively, 
    // here we just fetch them from their location.
    std::string leftFilename = workspaceFolder + "/images/left_grayscale.png";
    std::string rightFilename = workspaceFolder + "/images/right_grayscale.png";

    // Load the PNG image using ReadImage function
    unsigned width, height;  // Define width and height
    auto leftGrayImage = profileFunction("ReadImage", ReadImage, leftFilename.c_str(), std::ref(width), std::ref(height));
    auto rightGrayImage = profileFunction("ReadImage", ReadImage, rightFilename.c_str(), std::ref(width), std::ref(height));

    // Check if the image was successfully loaded
    if (leftGrayImage.empty() || rightGrayImage.empty()) {
        std::cout << "Failed to load image(s)." << std::endl;
        return 1;
    }

    std::string maxDispKey = "ndisp";
    int origMaxDisp = findValueForKey(maxDispKey);
    int maxDisp = origMaxDisp / 4;  // image was downscaled with factor 4

    // winSize can be edited by changing the value from calib.txt
    std::string winSizeKey = "winSize";
    int winSize = findValueForKey(winSizeKey);

    std::string leftImagePath = workspaceFolder + "/images/left_grayscale.png";
    auto leftImageMatrix = vectorToMatrix(leftGrayImage, std::ref(width), std::ref(height));
    std::string rightImagePath = workspaceFolder + "/images/right_grayscale.png";
    auto rightImageMatrix = vectorToMatrix(rightGrayImage, std::ref(width), std::ref(height));

    // Call the depth estimation function
    std::vector<std::vector<int>> leftToRightDisparityMap = depthEstimation(
        leftImageMatrix, rightImageMatrix, std::ref(width), std::ref(height), maxDisp, winSize);
    std::cout << "leftToRightDisparityMap has been formed." << std::endl;

    std::cout << "Normalize disparityMap pixel values (range 0-255)." << std::endl;
    int maxDisparityValue = maxDisp * 4;  // Multiply by 4 to compensate for downsampling
    for (auto& row : leftToRightDisparityMap) {
        for (auto& pixel : row) {
            pixel = static_cast<int>(static_cast<float>(pixel) / static_cast<float>(maxDisparityValue) * 255);
        }
    }

    std::vector<unsigned char> disparityMapVector = matrixToVector(leftToRightDisparityMap);

    std::cout << "Writing the disparityMap to a file." << std::endl;
    std::cout << "DEBUG width x height: " << height << " x " << width << std::endl;
    std::cout << "DEBUG vector size: " << disparityMapVector.size() << std::endl;
    std::string depthMapFilename = workspaceFolder + "/outputs/task3_outputs/depthmap.png";
    profileWriteImage("WriteImage (disparityMap)", WriteImage, depthMapFilename.c_str(), disparityMapVector, 
                      std::ref(width), std::ref(height));

    return 0;
}