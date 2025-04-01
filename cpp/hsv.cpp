#include <array>

std::array<float, 3> GetHSVColor(int index, int max_index){
    float hue = (float)index / (float)max_index;
    float saturation = 1.0;
    float value = 1.0;
    float c = value * saturation;
    float x = c * (1 - abs(fmod(hue * 6, 2) - 1));
    float m = value - c;
    float r, g, b;
    if (0 <= hue && hue < 1.0/6.0){
        r = c;
        g = x;
        b = 0;
    }
    else if (1.0/6.0 <= hue && hue < 2.0/6.0){
        r = x;
        g = c;
        b = 0;
    }
    else if (2.0/6.0 <= hue && hue < 3.0/6.0){
        r = 0;
        g = c;
        b = x;
    }
    else if (3.0/6.0 <= hue && hue < 4.0/6.0){
        r = 0;
        g = x;
        b = c;
    }
    else if (4.0/6.0 <= hue && hue < 5.0/6.0){
        r = x;
        g = 0;
        b = c;
    }
    else if (5.0/6.0 <= hue && hue <= 1){
        r = c;
        g = 0;
        b = x;
    }
    else{
        r = 0;
        g = 0;
        b = 0;
    }
    r += m;
    g += m;
    b += m;
    return {r, g, b};
}