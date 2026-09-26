#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "app/App.h"
#include "app/Options.h"

#include <iostream>

int main(int argc, char** argv)
{
    try {
        app::App app(app::parseOptions(argc, argv));
        return app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
