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

std::pair<const std::vector<unsigned char>, const std::vector<unsigned char>> depthEstimation(
    const std::vector<unsigned char>& leftImage, 
    const std::vector<unsigned char>& rightImage,
    unsigned width, unsigned height, unsigned maxDisp, unsigned winSize) {

    std::cout << "Depth estimation in progress..." << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    std::vector<unsigned char> leftDisparityMap(height * (width * 4), 0);

    for (unsigned y = 0; y < (height - winSize); ++y) {
        for (unsigned x = 0; x < (width - winSize); ++x) {
            float currentMaxSum = 0.0f;
            unsigned char bestDisparity = 0;

            for (unsigned d = 0; d < maxDisp; ++d) {
                if (x >= d) {
                    // Calculate ZNCC value

                    unsigned sumLeft = 0, sumRight = 0, leftMeanDeviationSquared = 0, rightMeanDeviationSquared = 0;
                    int numerator = 0;

                    // Precompute mean red value in each pixel
                    for (unsigned i = 0; i < winSize; ++i) {
                        for (unsigned j = 0; j < winSize; ++j) {
                            sumLeft += leftImage[4 * ((y + i) * width + (x + j))];
                            sumRight += rightImage[4 * ((y + i) * width + (x + j - d))];
                        }
                    }

                    unsigned meanLeft = sumLeft / (winSize * winSize);
                    unsigned meanRight = sumRight / (winSize * winSize);

                    // Calculate ZNCC
                    for (unsigned i = 0; i < winSize; ++i) {
                        for (unsigned j = 0; j < winSize; ++j) {
                            char leftDiff = leftImage[4 * ((y + i) * width + (x + j))] - meanLeft;
                            char rightDiff = rightImage[4 * ((y + i) * width + (x + j - d))] - meanRight;

                            numerator += leftDiff * rightDiff;
                            leftMeanDeviationSquared += (unsigned)(leftDiff * leftDiff);
                            rightMeanDeviationSquared += (unsigned)(rightDiff * rightDiff);
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
            leftDisparityMap[4 * ((y * width) + x)] = bestDisparity;    // R
            leftDisparityMap[4 * ((y * width) + x) + 1] = bestDisparity;  // B
            leftDisparityMap[4 * ((y * width) + x) + 2] = bestDisparity;  // G
            leftDisparityMap[4 * ((y * width) + x) + 3] = 255 / 4;  // A
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Depth estimation left-to-right done." << std::endl;
    std::cout << "Execution time: " << duration.count() << " ms" << std::endl;

    auto start2 = std::chrono::high_resolution_clock::now();

    std::vector<unsigned char> rightDisparityMap(height * (width * 4), 0);

    for (unsigned y = 0; y < (height - winSize); ++y) {
        for (unsigned x = 0; x < (width - winSize); ++x) {
            float currentMaxSum = 0.0f;
            unsigned char bestDisparity = 0;

            for (unsigned d = maxDisp; d > 0; --d) {
                if ((x + d + winSize) < (width * 4)) {
                    // Calculate ZNCC value

                    unsigned sumLeft = 0, sumRight = 0, leftMeanDeviationSquared = 0, rightMeanDeviationSquared = 0;
                    int numerator = 0;

                    // Precompute mean red value in each pixel
                    for (unsigned i = 0; i < winSize; ++i) {
                        for (unsigned j = 0; j < winSize; ++j) {
                            sumLeft += leftImage[4 * ((y + i) * width + (x + j + d))];
                            sumRight += rightImage[4 * ((y + i) * width + (x + j))];
                        }
                    }

                    unsigned meanLeft = sumLeft / (winSize * winSize);
                    unsigned meanRight = sumRight / (winSize * winSize);

                    // Calculate ZNCC
                    for (unsigned i = 0; i < winSize; ++i) {
                        for (unsigned j = 0; j < winSize; ++j) {
                            char leftDiff = leftImage[4 * ((y + i) * width + (x + j + d))] - meanLeft;
                            char rightDiff = rightImage[4 * ((y + i) * width + (x + j))] - meanRight;

                            numerator += leftDiff * rightDiff;
                            leftMeanDeviationSquared += (unsigned)(leftDiff * leftDiff);
                            rightMeanDeviationSquared += (unsigned)(rightDiff * rightDiff);
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
            rightDisparityMap[4 * ((y * width) + x)] = bestDisparity;    // R
            rightDisparityMap[4 * ((y * width) + x) + 1] = bestDisparity;  // B
            rightDisparityMap[4 * ((y * width) + x) + 2] = bestDisparity;  // G
            rightDisparityMap[4 * ((y * width) + x) + 3] = 255 / 4;  // A
        }
    }

    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
    std::cout << "Depth estimation right-to-left done." << std::endl;
    std::cout << "Execution time: " << duration2.count() << " ms" << std::endl;

    return {leftDisparityMap, rightDisparityMap};
}

std::vector<unsigned char> crossCheck(
    const std::vector<unsigned char>& leftDisparityMap, const std::vector<unsigned char>& rightDisparityMap, 
    int threshold, unsigned width, unsigned height) {

    std::cout << "Performing a cross-check on the disparity maps..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<unsigned char> crossCheckedDisparityMap(height * (width * 4), 0);

    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            unsigned char leftDisparity = leftDisparityMap[4 * (y * width + x)];
            unsigned char rightDisparity = rightDisparityMap[4 * (y * width + x)];

            unsigned char disparity = 0; // Remains zero if the disparities are not consistent

            // Check if the disparities are consistent within the threshold
            if (std::abs(leftDisparity - rightDisparity) <= threshold) {
                disparity = leftDisparity;
            }

            // Add values to pixel
            crossCheckedDisparityMap[4 * ((y * width) + x)] = disparity;    // R
            crossCheckedDisparityMap[4 * ((y * width) + x) + 1] = disparity;  // B
            crossCheckedDisparityMap[4 * ((y * width) + x) + 2] = disparity;  // G
            crossCheckedDisparityMap[4 * ((y * width) + x) + 3] = 255 / 4;  // A
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Cross-check done." << std::endl;
    std::cout << "Execution time: " << duration.count() << " ms" << std::endl;

    return crossCheckedDisparityMap;
}

std::vector<unsigned char> occlusionFill(const std::vector<unsigned char>& inputDisparityMap, unsigned width, unsigned height) {

    std::cout << "Performing an occlusion fill on the disparity map..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize the output disparity map with the input disparity map
    std::vector<unsigned char> outputDisparityMap = inputDisparityMap;

    // Iterate over each pixel in the disparity map
    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            // If the pixel value is zero (indicating occlusion), find the nearest non-zero value
            if (inputDisparityMap[4 * ((y * width) + x)] == 0) {
                unsigned char nearestNonZero = std::numeric_limits<unsigned char>::max(); // Initialize with a large value
                // Iterate over a window around the current pixel to find the nearest non-zero value
                for (int offsetY = 0; offsetY <= (int)height; ++offsetY) {
                    for (int offsetX = 0; offsetX <= (int)width * 4; ++offsetX) {
                        int neighborY = (int)y + offsetY;
                        int neighborX = (int)x + offsetX;
                        int negNeighborY = (int)y - offsetY;
                        int negNeighborX = (int)x - offsetX;
                        // Check if the neighbor pixel is within bounds and non-zero
                        if (neighborY >= 0 && neighborY < (int)height && neighborX >= 0 && neighborX < (int)width &&
                            inputDisparityMap[(unsigned)(4 * ((neighborY * (int)width) + neighborX))] != 0) {
                                nearestNonZero = std::min(nearestNonZero, inputDisparityMap[(unsigned)(4 * ((neighborY * (int)width) + neighborX))]);
                        }
                        if (negNeighborY >= 0 && negNeighborY < (int)height && negNeighborX >= 0 && negNeighborX < (int)width &&
                            inputDisparityMap[(unsigned)(4 * ((negNeighborY * (int)width) + negNeighborX))] != 0) {
                            nearestNonZero = std::min(nearestNonZero, inputDisparityMap[(unsigned)(4 * ((negNeighborY * (int)width) + negNeighborX))]);
                        }
                        if (negNeighborY >= 0 && negNeighborY < (int)height && neighborX >= 0 && neighborX < (int)width &&
                            inputDisparityMap[(unsigned)(4 * ((negNeighborY * (int)width) + neighborX))] != 0) {
                            nearestNonZero = std::min(nearestNonZero, inputDisparityMap[(unsigned)(4 * ((negNeighborY * (int)width) + neighborX))]);
                        }
                        if (neighborY >= 0 && neighborY < (int)height && negNeighborX >= 0 && negNeighborX < (int)width &&
                            inputDisparityMap[(unsigned)(4 * ((neighborY * (int)width) + negNeighborX))] != 0) {
                            nearestNonZero = std::min(nearestNonZero, inputDisparityMap[(unsigned)(4 * ((neighborY * (int)width) + negNeighborX))]);
                        }
                        if (nearestNonZero != std::numeric_limits<unsigned char>::max()) {
                        break;
                        }  // Stop search once a non-zero pixel has been found
                    }
                    if (nearestNonZero != std::numeric_limits<unsigned char>::max()) {
                        break;
                    }  // Stop search once a non-zero pixel has been found
                }
                // Update the current pixel with the nearest non-zero value
                outputDisparityMap[4 * ((y * width ) + x)] = nearestNonZero;
                outputDisparityMap[4 * ((y * width ) + x) + 1] = nearestNonZero;
                outputDisparityMap[4 * ((y * width ) + x) + 2] = nearestNonZero;
                outputDisparityMap[4 * ((y * width ) + x) + 3] = 255 / 4;
            }
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Occlusion fill done." << std::endl;
    std::cout << "Execution time: " << duration.count() << " ms" << std::endl;

    return outputDisparityMap;
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

unsigned searchFileForKeyValuePair(const std::string& keyToFind) {
    std::ifstream calibFile("calib.txt");
    std::string line;
    unsigned foundValue = 0; // Default value if key is not found

    if (calibFile.is_open()) {
        while (std::getline(calibFile, line)) {
            std::string value = extractValue(line, keyToFind);
            if (!value.empty()) {
                foundValue = (unsigned)std::stoi(value);
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

unsigned findValueForKey(const std::string& keyToFind) {
    unsigned foundValue = searchFileForKeyValuePair(keyToFind);

    if (foundValue == 0) {
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
    unsigned origMaxDisp = findValueForKey(maxDispKey);
    unsigned maxDisp = origMaxDisp / downscaleFactor;

    // winSize can be edited by changing the value from calib.txt
    std::string winSizeKey = "winSize";
    unsigned winSize = findValueForKey(winSizeKey);

    // Call the depth estimation function
    std::pair<std::vector<unsigned char>, std::vector<unsigned char>> disparityMaps = depthEstimation(
        leftGrayImage, rightGrayImage, width, height, maxDisp, winSize);
    std::vector<unsigned char>& leftDisparityMap = disparityMaps.first;
    std::vector<unsigned char>& rightDisparityMap = disparityMaps.second;
    
    /*
    std::cout << "Normalize disparityMap pixel values (range 0-255)." << std::endl;
    for (auto& pixel : leftDisparityMap) {
            pixel = pixel * origMaxDisp / maxDisp;
    }
    for (auto& pixel : rightDisparityMap) {
            pixel = pixel * origMaxDisp / maxDisp;
    }

    std::cout << "Writing the disparityMaps to files." << std::endl;
    std::string depthMapFilename1 = workspaceFolder + "/outputs/task3_outputs/depthmap1.png";
    std::string depthMapFilename2 = workspaceFolder + "/outputs/task3_outputs/depthmap2.png";
    profileWriteImage("writing disparityMap1 to image", WriteImage, depthMapFilename1.c_str(), 
        leftDisparityMap, std::ref(width), std::ref(height));
    profileWriteImage("writing disparityMap2 to image", WriteImage, depthMapFilename2.c_str(), 
        rightDisparityMap, std::ref(width), std::ref(height));
    */

    int threshold = 8;

    std::vector<unsigned char> crossCheckedDisparityMap = crossCheck(
        leftDisparityMap, rightDisparityMap, threshold, width, height);
    
    /*
    std::cout << "Normalize disparityMap pixel values (range 0-255)." << std::endl;
    for (auto& pixel : crossCheckedDisparityMap) {
            pixel = pixel * origMaxDisp / maxDisp;
    }
    
    std::string crossCheckFileName = workspaceFolder + "/outputs/task3_outputs/crosscheck.png";
    profileWriteImage("writing cross checked image", WriteImage, crossCheckFileName.c_str(), 
        crossCheckedDisparityMap, std::ref(width), std::ref(height));
    */

    std::vector<unsigned char> finalDisparityMap = occlusionFill(
        crossCheckedDisparityMap, width, height);
    
    std::cout << "Normalize disparityMap pixel values (range 0-255)." << std::endl;
    for (auto& pixel : finalDisparityMap) {
            pixel = pixel * origMaxDisp / maxDisp;
    }
    
    std::string finalMapFileName = workspaceFolder + "/outputs/task3_outputs/finalDisparityMap.png";
    profileWriteImage("writing occlusion filled image", WriteImage, finalMapFileName.c_str(), 
        finalDisparityMap, std::ref(width), std::ref(height));

    return 0;
}