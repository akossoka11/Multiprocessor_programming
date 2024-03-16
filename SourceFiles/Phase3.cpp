/*
#include <iostream>
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
*/
/*
int main() {
    // Example usage:
    // Assume leftImage, rightImage, and disparityMap are 2D vectors representing grayscale images
    // and the output disparity map respectively.
    // maxDisp is the maximum disparity value (ndisp in calib.txt)
    // winSize is the window size, start with 9x9

    std::vector<std::vector<int>> leftImage = { // };  // Populate with your grayscale left image data
    std::vector<std::vector<int>> rightImage = { // }; // Populate with your grayscale right image data
    int maxDisp = 260;  // Example value, replace with your actual max disparity
    int winSize = 9;    // Example value, replace with your actual window size
    //std::string filename = "C:\\Users\\akoss\\MPP\\image_0.png";
    // Initialize disparityMap with zeros
    std::vector<std::vector<int>> disparityMap(leftImage.size(), std::vector<int>(leftImage[0].size(), 0));

    // Call the depth estimation function
    depthEstimation(leftImage, rightImage, maxDisp, winSize, disparityMap);

    // Now disparityMap contains the estimated disparities for each pixel
    return 0;
}*/