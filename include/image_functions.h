#ifndef IMAGE_FUNCTIONS_H
#define IMAGE_FUNCTIONS_H

#include <iostream>
#include <string>
#include <chrono>

std::string getCurrentDirectory();

std::vector<unsigned char> ReadImage(const std::string& filename, unsigned& width, unsigned& height);

std::vector<unsigned char> ResizeImage(const std::vector<unsigned char>& inputImage, unsigned originalWidth, unsigned originalHeight);

void WriteImage(const char* filename, const std::vector<unsigned char>& outputImage, unsigned width, unsigned height);

#endif