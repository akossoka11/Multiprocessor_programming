#include "..\include\lodepng.h"
#include <functional>
#include <iostream>
#include <vector>
#include <filesystem>
#include <chrono>

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

/*
std::vector<unsigned char> GrayScaleImage(const std::vector<unsigned char>& inputImage, unsigned width, unsigned height) {
    std::vector<unsigned char> grayScaleImage(width * height, 255); // Initialize to white

    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            size_t originalIndex = (y * width + x) * 4;
            size_t grayScaleIndex = y * width + x;

                // Convert to grayscale using the specified formula
                unsigned char red = inputImage[originalIndex];
                unsigned char green = inputImage[originalIndex + 1];
                unsigned char blue = inputImage[originalIndex + 2];

                unsigned char grayValue = static_cast<unsigned char>(0.2126 * red + 0.7152 * green + 0.0722 * blue);

                grayScaleImage[grayScaleIndex] = grayValue;
        }
    }

    return grayScaleImage;
}*/

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

// Function to write the output image
void WriteImage(const char* filename, const std::vector<unsigned char>& outputImage, unsigned width, unsigned height) {
    unsigned error = lodepng::encode(filename, outputImage, width, height);

    if (error) {
        std::cerr << "Error while encoding PNG: " << lodepng_error_text(error) << std::endl;
        exit(1);
    }
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

/*
std::vector<unsigned char> ApplyFilter(const std::vector<unsigned char>& inputImage, unsigned width, unsigned height) {
    std::vector<unsigned char> filteredImage(width * height * 4, 255); // Initialize to white

    // Define the Gaussian kernel
    float kernel[5][5] = {
        {0.003f, 0.013f, 0.022f, 0.013f, 0.003f},
        {0.013f, 0.059f, 0.097f, 0.059f, 0.013f},
        {0.022f, 0.097f, 0.159f, 0.097f, 0.022f},
        {0.013f, 0.059f, 0.097f, 0.059f, 0.013f},
        {0.003f, 0.013f, 0.022f, 0.013f, 0.003f}
    };

    // Apply the filter to each pixel
    for (unsigned y = 2; y < height - 2; ++y) {
        for (unsigned x = 2; x < width - 2; ++x) {
            float sum[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

            for (int i = -2; i <= 2; ++i) {
                for (int j = -2; j <= 2; ++j) {
                    size_t index = ((y + i) * width + (x + j)) * 4;

                    for (int c = 0; c < 4; ++c) {
                        sum[c] += static_cast<float>(inputImage[index + c]) * kernel[i + 2][j + 2];
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
}*/



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
    // Construct the full path to the image file
    std::string filename = "C:\\Users\\akoss\\MPP\\image_1.png";

    // Load the PNG image using ReadImage function
    //std::vector<unsigned char> image_0; // This will store the raw pixel data
    unsigned width, height;  // Define width and height

    //image_0 = ReadImage(filename, width, height);
    // Profile ReadImage function
    auto image_0 = profileFunction("ReadImage", ReadImage, filename, std::ref(width), std::ref(height));

    // Check if the image was successfully loaded
    if (image_0.empty()) {
        // Handle the error as needed
        return 1;
    }

    // Display some information about the original image
    std::cout << "Original Image dimensions: " << width << "x" << height << std::endl;
    std::cout << "Total pixels in original image: " << width * height << std::endl;

    // Resize the image
    //std::vector<unsigned char> resizedImage = ResizeImage(image_0, width, height);
    auto resizedImage = profileFunction("ResizeImage", ResizeImage, image_0, std::ref(width), std::ref(height));
    unsigned newWidth = width / 4;
    unsigned newHeight = height / 4;

    // Display some information about the resized image
    std::cout << "Resized Image dimensions: " << newWidth << "x" << newHeight << std::endl;
    std::cout << "Total pixels in resized image: " << newWidth * newHeight << std::endl;

    profileWriteImage("WriteImage (Resize)", WriteImage, "C:\\Users\\akoss\\MPP\\resize_image.png", resizedImage, newWidth, newHeight);

    auto resizedImageRead = profileFunction("ReadImage", ReadImage, "C:\\Users\\akoss\\MPP\\resize_image.png", std::ref(newWidth), std::ref(newHeight));


    // Convert the image to grayscale
    //std::vector<unsigned char> grayScaleImage = GrayScaleImage(image_0, width, height);
    auto grayScaleImage = profileFunction("ConvertToGreyscale", ConvertToGreyscale, resizedImageRead, std::ref(newWidth), std::ref(newHeight));
    //convertToGreyscale(resizedImage, newWidth, newHeight);


    // Save the grayscale image
    //WriteImage("C:\\Users\\akoss\\MPP\\grayscale_image.png", grayScaleImage, newWidth, newHeight);
    profileWriteImage("WriteImage (Grayscale)", WriteImage, "C:\\Users\\akoss\\MPP\\grayscale_image1.png", grayScaleImage, newWidth, newHeight);
    
    
    // Apply a 5x5 filter to the grayscale image
    //std::vector<unsigned char> filteredImage = ApplyFilter(grayScaleImage, newWidth, newHeight);
    auto filteredImage = profileFunction("ApplyFilter", ApplyFilter, grayScaleImage, std::ref(newWidth), std::ref(newHeight));
    
    // Save the filtered image
    //WriteImage("C:\\Users\\akoss\\MPP\\filtered_image.png", filteredImage, newWidth, newHeight);
    profileWriteImage("WriteImage (Grayscale)", WriteImage, "C:\\Users\\akoss\\MPP\\filtered_image1.png", filteredImage, newWidth, newHeight);
    
    return 0;
}