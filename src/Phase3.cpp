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

/*
// Function to calculate ZNCC for a given window
float calculateZNCC(const std::vector<unsigned char>& image, const std::vector<unsigned char>& otherImage,
                    unsigned x, unsigned y, unsigned d, unsigned winSize) {
    unsigned sumLeft = 0, sumRight = 0, numerator = 0;
    unsigned leftMeanDeviationSquared = 0, rightMeanDeviationSquared = 0;

    // Precompute mean red value in each pixel
    for (unsigned i = 0; i < winSize; ++i) {
        for (unsigned j = 0; j < winSize; ++j) {
            sumLeft += image[(y + i) * x + (x + j)];
            sumRight += otherImage[(y + i) * x + (x + j - d)];
        }
    }

    unsigned meanLeft = sumLeft / (winSize * winSize);
    unsigned meanRight = sumRight / (winSize * winSize);

    // Calculate ZNCC
    for (unsigned i = 0; i < winSize; ++i) {
        for (unsigned j = 0; j < winSize; ++j) {
            unsigned leftDiff = image[(y + i) * x + (x + j)] - meanLeft;
            unsigned rightDiff = otherImage[(y + i) * x + (x + j - d)] - meanRight;

            numerator += leftDiff * rightDiff;
            leftMeanDeviationSquared += leftDiff * leftDiff;
            rightMeanDeviationSquared += rightDiff * rightDiff;
        }
    }

    float znccValue = (float)(numerator / (std::sqrt(leftMeanDeviationSquared) * std::sqrt(rightMeanDeviationSquared)));

    return znccValue;
}
*/

const std::vector<unsigned char> depthEstimation(
    const std::vector<unsigned char>& image, 
    const std::vector<unsigned char>& otherImage,
    unsigned width, unsigned height, unsigned maxDisp, unsigned winSize) {

    std::cout << "Depth estimation in progress..." << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    std::vector<unsigned char> disparityMap(height * (width * 4), 0);

    for (unsigned y = 0; y < (height - winSize); ++y) {
        for (unsigned x = 0; x < (width - winSize); ++x) {
            float currentMaxSum = 0.0f;
            unsigned char bestDisparity = 0;

            for (unsigned d = 0; d < maxDisp; ++d) {
                if (x >= d) {
                    // float currentSum = 0;
                    // Calculate ZNCC value

                    unsigned sumLeft = 0, sumRight = 0, numerator = 0;
                    unsigned leftMeanDeviationSquared = 0, rightMeanDeviationSquared = 0;

                    // Precompute mean red value in each pixel
                    for (unsigned i = 0; i < winSize; ++i) {
                        for (unsigned j = 0; j < winSize; ++j) {
                            sumLeft += image[4 * ((y + i) * width + (x + j))];
                            sumRight += otherImage[4 * ((y + i) * width + (x + j - d))];
                        }
                    }

                    unsigned meanLeft = sumLeft / (winSize * winSize);
                    unsigned meanRight = sumRight / (winSize * winSize);

                    // Calculate ZNCC
                    for (unsigned i = 0; i < winSize; ++i) {
                        for (unsigned j = 0; j < winSize; ++j) {
                            unsigned leftDiff = image[4 * ((y + i) * width + (x + j))] - meanLeft;
                            unsigned rightDiff = otherImage[4 * ((y + i) * width + (x + j - d))] - meanRight;

                            numerator += leftDiff * rightDiff;
                            leftMeanDeviationSquared += leftDiff * leftDiff;
                            rightMeanDeviationSquared += rightDiff * rightDiff;
                        }
                    }

                    float znccValue = (float)(numerator / (std::sqrt(leftMeanDeviationSquared) * std::sqrt(rightMeanDeviationSquared)));

                    if (znccValue > currentMaxSum) {
                        currentMaxSum = znccValue;
                        bestDisparity = d;
                    }
                }
            }

            // Add values to pixel
            disparityMap[4 * ((y * width) + x)] = bestDisparity;    // R
            disparityMap[4 * ((y * width) + x) + 1] = bestDisparity;  // B
            disparityMap[4 * ((y * width) + x) + 2] = bestDisparity;  // G
            disparityMap[4 * ((y * width) + x) + 3] = 255;  // A
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Depth estimation done." << std::endl;
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
    unsigned foundValue = searchFileForKeyValuePair(keyToFind);

    if (foundValue == -1) {
        std::cout << "Key '" << keyToFind << "' not found in calib.txt." << std::endl;
        exit(1);
    }

    return foundValue;
}

// Function to profile the execution time of a given function and display the result
template <typename Func, typename... Args>
auto profileFunction(const std::string& functionName, Func&& func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();

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
/*
std::vector<std::vector<int>> vectorToMatrix(const std::vector<unsigned char>& image, unsigned width, unsigned height) {
    std::vector<std::vector<int>> matrix(height, std::vector<int>(width * 4, 0));

    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width * 4; ++x) {
            // Calculate the index in the 1D vector corresponding to the current pixel
            unsigned index = y * width * 4 + x;
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

void printMatrix(const std::vector<std::vector<int>>& matrix) {
    // For debugging purposes
    for (const auto& row : matrix) {
        for (int value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
}

void printVector(const std::vector<unsigned char>& vec, size_t width, size_t height) {
    // For debugging purposes
    for (size_t i = 0; i < height; ++i) {
        for (size_t j = 0; j < width; ++j) {
            size_t index = i * width + j;
            std::cout << static_cast<int>(vec[index]) << " ";
        }
        std::cout << std::endl;
    }
}*/

int main() {
    // Example usage:
    // Assume leftImage, rightImage, and disparityMap are 2D vectors representing grayscale images
    // and the output disparity map respectively.
    // maxDisp is the maximum disparity value (ndisp in calib.txt).
    // winSize is the window size, start with 9x9 (winSize in calib.txt).

    std::string workspaceFolder = getCurrentDirectory();

    // Construct file paths using the workspace folder
    std::string leftImagePath = workspaceFolder + "/images/im0.png";
    std::string rightImagePath = workspaceFolder + "/images/im1.png";

    unsigned origWidth, origHeight;  // Define original width and height

    auto leftImage = profileFunction(
        "reading original left image", ReadImage, leftImagePath.c_str(), std::ref(origWidth), std::ref(origHeight));
    auto rightImage = profileFunction(
        "reading original right image", ReadImage, rightImagePath.c_str(), std::ref(origWidth), std::ref(origHeight));

    // Check if the image was successfully loaded
    if (leftImage.empty() || rightImage.empty()) {
        // Handle the error as needed
        std::cout << "Failed to load image(s)." << std::endl;
        return 1;
    }

    // Resize images
    const int downscaleFactor = 4;
    auto resizedLeftImage = profileFunction(
        "resizing left image", ResizeImage, leftImage, std::ref(origWidth), std::ref(origHeight), downscaleFactor);
    auto resizedRightImage = profileFunction(
        "resizing right image", ResizeImage, rightImage, std::ref(origWidth), std::ref(origHeight), downscaleFactor);
    unsigned width = origWidth / downscaleFactor;
    unsigned height = origHeight / downscaleFactor;

    // Convert to grayscale
    auto leftGrayImage = profileFunction(
        "converting left image to grayscale", ConvertToGreyscale, resizedLeftImage, std::ref(width), std::ref(height));
    auto rightGrayImage = profileFunction(
        "converting right image to grayscale", ConvertToGreyscale, resizedRightImage, std::ref(width), std::ref(height));
    std::string leftGrayFilename = workspaceFolder + "/outputs/task3_outputs/left_grayscale.png";
    std::string rightGrayFilename = workspaceFolder + "/outputs/task3_outputs/right_grayscale.png";
    profileWriteImage("writing left grayscale image", WriteImage, leftGrayFilename.c_str(), leftGrayImage, width, height);
    profileWriteImage("writing right grayscale image", WriteImage, rightGrayFilename.c_str(), rightGrayImage, width, height);

    std::string maxDispKey = "ndisp";
    int origMaxDisp = findValueForKey(maxDispKey);
    unsigned maxDisp = origMaxDisp / downscaleFactor;

    // winSize can be edited by changing the value from calib.txt
    std::string winSizeKey = "winSize";
    unsigned winSize = findValueForKey(winSizeKey);

    /*
    auto leftImageMatrix = vectorToMatrix(leftGrayImage, std::ref(width), std::ref(height));
    auto rightImageMatrix = vectorToMatrix(rightGrayImage, std::ref(width), std::ref(height));
    */
    // Call the depth estimation function
    std::vector<unsigned char> leftToRightDisparityMap = depthEstimation(
        leftGrayImage, rightGrayImage, width, height, maxDisp, winSize);
    // std::vector<std::vector<int>> rightToLeftDisparityMap = depthEstimation(
    //     rightImageMatrix, leftImageMatrix, std::ref(width), std::ref(height), maxDisp, winSize);
    /*
    std::cout << "Normalize disparityMap pixel values (range 0-255)." << std::endl;
    for (auto& row : leftToRightDisparityMap) {
        for (auto& pixel : row) {
            float normalizedPixel = static_cast<float>(pixel) / static_cast<float>(maxDisp) * static_cast<float>(origMaxDisp);
            pixel = static_cast<int>(normalizedPixel);
        }
    }
    for (auto& row : rightToLeftDisparityMap) {
        for (auto& pixel : row) {
            float normalizedPixel = pixel % maxDisp * origMaxDisp;
            pixel = static_cast<int>(normalizedPixel);
        }
    }*/

    // std::vector<unsigned char> disparityMapVectorLeft = matrixToVector(leftToRightDisparityMap);
    // std::vector<unsigned char> disparityMapVectorRight = matrixToVector(rightToLeftDisparityMap);

    std::cout << "Writing the disparityMaps to files." << std::endl;
    std::string depthMapFilename1 = workspaceFolder + "/outputs/task3_outputs/depthmap1.png";
    // std::string depthMapFilename2 = workspaceFolder + "/outputs/task3_outputs/depthmap2.png";
    profileWriteImage("writing disparityMap1 to image", WriteImage, depthMapFilename1.c_str(), 
        leftToRightDisparityMap, std::ref(width), std::ref(height));
    // profileWriteImage("writing disparityMap2 to image", WriteImage, depthMapFilename2.c_str(), 
    //     disparityMapVectorRight, std::ref(width), std::ref(height));
    return 0;
}