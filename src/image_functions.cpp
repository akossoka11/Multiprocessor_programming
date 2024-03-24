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

std::vector<unsigned char> ReadImage(const std::string& filename, unsigned& width, unsigned& height) {
    std::vector<unsigned char> image;

    unsigned error = lodepng::decode(image, width, height, filename.c_str());

    if (error) {
        std::cerr << "Error while decoding PNG: " << lodepng_error_text(error) << std::endl;
        // Handle the error as needed, e.g., throw an exception or return an empty vector
        // For simplicity, this example just prints the error and returns an empty vector
        return std::vector<unsigned char>();
    }

    return image;
}

std::vector<unsigned char> ResizeImage(const std::vector<unsigned char>& inputImage, unsigned originalWidth, unsigned originalHeight) {
    // Calculate new dimensions for the resized image (1/4 of the original size)
    unsigned newWidth = originalWidth / 4;
    unsigned newHeight = originalHeight / 4;

    // Create a vector to store the resized image
    std::vector<unsigned char> resizedImage(newWidth * newHeight * 4, 255); // Initialize to white

    // Resize the image by taking pixels from every fourth row and column
    for (unsigned y = 0; y < newHeight; ++y) {
        for (unsigned x = 0; x < newWidth; ++x) {
            size_t originalIndex = ((y * 4) * originalWidth + (x * 4)) * 4;
            size_t resizedIndex = (y * newWidth + x) * 4;

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
        std::cerr << "Error while encoding PNG: " << lodepng_error_text(error) << std::endl;
        exit(1);
    }
}
