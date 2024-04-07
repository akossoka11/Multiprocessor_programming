#include "../include/lodepng.h"
#include "../include/image_functions.h"
#include <functional>
#include <iostream>
#include <vector>
#include <filesystem>
#include <chrono>
#include <unistd.h> // For getcwd() function

std::string getCurrentDirectory() {
    char buffer[FILENAME_MAX];
    if (getcwd(buffer, FILENAME_MAX) != NULL) {
        return std::string(buffer);
    } else {
        std::cerr << "Error getting current directory." << std::endl;
        exit(1);
    }
}

std::vector<unsigned char> ConvertToGreyscale(const std::vector<unsigned char>& image, unsigned width, unsigned height) {
    std::vector<unsigned char> greyscaleImage;
    greyscaleImage.reserve(width * height);

    for (size_t i = 0; i < image.size(); i += 4) {
        // Extract RGB values
        unsigned char r = image[i];
        unsigned char g = image[i + 1];
        unsigned char b = image[i + 2];

        // Compute greyscale value using the given formula
        unsigned char grey = static_cast<unsigned char>(0.2126 * r + 0.7152 * g + 0.0722 * b);

        // Set RGB values of the pixel to the computed greyscale value
        greyscaleImage.push_back(grey);
        greyscaleImage.push_back(grey);
        greyscaleImage.push_back(grey);
        greyscaleImage.push_back(255);  // Alpha value
    }

    return greyscaleImage;
}

std::vector<unsigned char> ApplyFilter(const std::vector<unsigned char>& inputImage, unsigned width, unsigned height) {
    std::vector<unsigned char> filteredImage(width * height * 4, 255); // Initialize to white

    // Example: Identity filter (no change)
    float filter[5][5] = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };

    // Apply the filter to each pixel
    for (unsigned y = 2; y < height - 2; ++y) {
        for (unsigned x = 2; x < width - 2; ++x) {
            float sum[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

            for (int i = -2; i <= 2; ++i) {
                for (int j = -2; j <= 2; ++j) {
                    size_t index = ((y + i) * width + (x + j)) * 4;

                    for (int c = 0; c < 4; ++c) {
                        sum[c] += static_cast<float>(inputImage[index + c]) * filter[i + 2][j + 2];
                    }
                }
            }

            size_t filteredIndex = (y * width + x) * 4;

            for (int c = 0; c < 4; ++c) {
                filteredImage[filteredIndex + c] = static_cast<unsigned char>(sum[c]);
            }
        }
    }

    return filteredImage;
}

std::vector<unsigned char> ReadImage(const std::string& filename, unsigned& width, unsigned& height) {
    std::vector<unsigned char> image;

    unsigned error = lodepng::decode(image, width, height, filename.c_str());

    if (error) {
        std::cout << "Error while decoding PNG: " << lodepng_error_text(error) << std::endl;
        // Handle the error as needed, e.g., throw an exception or return an empty vector
        // For simplicity, this example just prints the error and returns an empty vector
        return std::vector<unsigned char>();
    }

    return image;
}

std::vector<unsigned char> ResizeImage(const std::vector<unsigned char>& inputImage, unsigned originalWidth, 
                                       unsigned originalHeight, unsigned downscaleFactor) {
    // Calculate new dimensions for the resized image (1/4 of the original size)
    unsigned newWidth = originalWidth / downscaleFactor;
    unsigned newHeight = originalHeight / downscaleFactor;

    // Create a vector to store the resized image
    std::vector<unsigned char> resizedImage(newWidth * newHeight * downscaleFactor, 255); // Initialize to white

    // Resize the image by taking pixels from every fourth row and column
    for (unsigned y = 0; y < newHeight; ++y) {
        for (unsigned x = 0; x < newWidth; ++x) {
            size_t originalIndex = ((y * 4) * originalWidth + (x * 4)) * downscaleFactor;
            size_t resizedIndex = (y * newWidth + x) * downscaleFactor;

            // Copy the RGBA values
            for (int i = 0; i < 4; ++i) {
                resizedImage[resizedIndex + i] = inputImage[originalIndex + i];
            }
        }
    }

    return resizedImage;
}

// Function to write the output image
void WriteImage(const char* filename, const std::vector<unsigned char>& outputImage, unsigned width, unsigned height) {
    unsigned error = lodepng::encode(filename, outputImage, width, height);

    if (error) {
        std::cout << "Error while encoding PNG: " << lodepng_error_text(error) << std::endl;
        exit(1);
    }
}
