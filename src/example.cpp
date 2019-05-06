#include <cmath>
#include <iostream>

int
main(int argc, char **argv)
{
    std::cout << "Graph Title" << std::endl;
    std::cout << "time,sin(x),cos(x),tan(x)" << std::endl;

    for (double x = 0; x < 1; x += 0.01)
        std::cout << x << ',' << sin(x) << ',' << cos(x) << ',' << tan(x) << std::endl;

    return EXIT_SUCCESS;
}
