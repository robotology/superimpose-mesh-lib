#include <exception>
#include <iostream>
#include <string>

#include <SuperimposeMesh/SICAD.h>

int main()
{
    std::string log_ID = "[Test - HDPI]";
    std::cout << log_ID << "This is a FAKE TEST to check whether the present machine sports HiDPI monitor or not." << std::endl;
    std::cout << log_ID << "A runtime error will be caught, but the test will always pass." << std::endl;
    std::cout << log_ID << "HiDPI should be handled automatically by the library." << std::endl  << std::endl;

    const unsigned int cam_width_ = 320;
    const unsigned int cam_height_ = 240;
    const float        cam_fx_ = 257.34;
    const float        cam_cx_ = 160;
    const float        cam_fy_ = 257.34;
    const float        cam_cy_ = 120;

    try
    {
        SICAD si_cad(SICAD::ModelPathContainer(),
                     cam_width_, cam_height_,
                     cam_fx_, cam_fy_, cam_cx_, cam_cy_);
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << log_ID << "Caught error:" << std::endl << e.what();
    }

    std::cout << log_ID << "If the rendered image size is greater than the given image size, then the library is not able to handle it properly!" << std::endl;
    std::cout << log_ID << "Should this error occur, please open an issue on the GitHub library repository!" << std::endl;

    return EXIT_SUCCESS;
}
