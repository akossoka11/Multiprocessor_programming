#include "../include/lodepng.h"
#include "../include/image_functions.h"
#include <functional>
#include <iostream>
#include <vector>
#include <filesystem>
#include <chrono>
#include <unistd.h> // For getcwd() function

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
    std::string workspaceFolder = getCurrentDirectory();
    std::cout << "Workspace folder: " << workspaceFolder << std::endl;

    // Construct file paths using the workspace folder
    std::string filename = workspaceFolder + "/images/im0.png";
    std::cout << "Trying to find image from " << filename << std::endl;

    // Load the PNG image using ReadImage function
    //std::vector<unsigned char> image_0; // This will store the raw pixel data
    unsigned width, height;  // Define width and height

    //image_0 = ReadImage(filename, width, height);
    // Profile ReadImage function
    auto image_0 = profileFunction("ReadImage", ReadImage, filename.c_str(), std::ref(width), std::ref(height));

    // Check if the image was successfully loaded
    if (image_0.empty()) {
        // Handle the error as needed
        std::cout << "Failed to load image." << std::endl;
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

    std::string resizeFilename = workspaceFolder + "/outputs/task2_outputs/resize_image.png";
    profileWriteImage("WriteImage (Resize)", WriteImage, resizeFilename.c_str(), resizedImage, newWidth, newHeight);

    auto resizedImageRead = profileFunction("ReadImage", ReadImage, resizeFilename.c_str(), std::ref(newWidth), std::ref(newHeight));


    // Convert the image to grayscale
    //std::vector<unsigned char> grayScaleImage = GrayScaleImage(image_0, width, height);
    auto grayScaleImage = profileFunction("ConvertToGreyscale", ConvertToGreyscale, resizedImageRead, std::ref(newWidth), std::ref(newHeight));
    //convertToGreyscale(resizedImage, newWidth, newHeight);


    // Save the grayscale image
    std::string grayFilename = workspaceFolder + "/outputs/task2_outputs/grayscale_image.png";
    //WriteImage(grayFilename, grayScaleImage, newWidth, newHeight);
    profileWriteImage("WriteImage (Grayscale)", WriteImage, grayFilename.c_str(), grayScaleImage, newWidth, newHeight);
    
    
    // Apply a 5x5 filter to the grayscale image
    //std::vector<unsigned char> filteredImage = ApplyFilter(grayScaleImage, newWidth, newHeight);
    auto filteredImage = profileFunction("ApplyFilter", ApplyFilter, grayScaleImage, std::ref(newWidth), std::ref(newHeight));
    
    // Save the filtered image
    std::string filteredFilename = workspaceFolder + "/outputs/task2_outputs/filtered_image.png";
    //WriteImage(filteredFilename, filteredImage, newWidth, newHeight);
    profileWriteImage("WriteImage (Grayscale)", WriteImage, filteredFilename.c_str(), filteredImage, newWidth, newHeight);
    
    return 0;
}
