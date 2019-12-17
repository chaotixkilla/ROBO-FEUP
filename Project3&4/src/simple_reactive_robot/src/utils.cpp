#include "utils.hpp"

using namespace std;

float radToDeg(float radians) {
    return radians * (180.0 / PI);
}

float degToRad(float degrees) {
    return degrees * (PI / 180.0);
}