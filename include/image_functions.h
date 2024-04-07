#ifndef IMAGE_FUNCTIONS_H
#define IMAGE_FUNCTIONS_H

#include <iostream>
#include <string>
#include <chrono>

std::string getCurrentDirectory();

std::vector<unsigned char> ConvertToGreyscale(const std::vector<unsigned char>& image, unsigned width, unsigned height);

std::vector<unsigned char> ApplyFilter(const std::vector<unsigned char>& inputImage, unsigned width, unsigned height);

std::vector<unsigned char> ReadImage(const std::string& filename, unsigned& width, unsigned& height);

std::vector<unsigned char> ResizeImage(const std::vector<unsigned char>& inputImage, unsigned originalWidth, unsigned originalHeight, unsigned downscaleFactor);

void WriteImage(const char* filename, const std::vector<unsigned char>& outputImage, unsigned width, unsigned height);

#endif