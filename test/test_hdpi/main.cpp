#include <exception>
#include <iostream>
#include <string>

#include <SuperImpose/SICAD.h>

int main()
{
    std::string log_ID = "[TEST-HDPI]";
    std::cout << log_ID << "This is a FAKE TEST to check whether the present machine sports HDPI monitor or not." << std::endl;
    std::cout << log_ID << "A runtime error will be caught, but the test will always pass." << std::endl;
    std::cout << log_ID << "Read the last comment and ENABLE/DISABLE HDPI option in CMake config for better performance." << std::endl  << std::endl;

    try
    {
        SICAD si_cad(SuperImpose::ObjFileMap(), 320, 240, "");
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << log_ID << "Caught error:" << std::endl << e.what();
    }

    std::cout << log_ID << "If the rendered image size is greater than the given image size, then you have an HDPI monitor!" << std::endl;
    std::cout << log_ID << "ENABLE/DISABLE HDPI option in CMake and run the test again!" << std::endl;

    return EXIT_SUCCESS;
}
