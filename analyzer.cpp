#include "analyzer.h"

#include <sstream>

std::string Analyzer::to_string(double x)
{
    std::ostringstream stream;
    if (!(stream << x))
        return "0.0";
    return stream.str();
}
