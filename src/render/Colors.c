#include <render/Colors.h>

unsigned int getColorCode(Color c) {
    switch (c) {
        case COLOR_BLACK:         return 0xFF000000;
        case COLOR_WHITE:         return 0xFFFFFFFF;
        case COLOR_GRAY:          return 0xFF808080;
        case COLOR_LIGHT_GRAY:    return 0xFFD3D3D3;
        case COLOR_DARK_GRAY:     return 0xFF404040;

        case COLOR_RED:           return 0xFFFF0000;
        case COLOR_LIGHT_RED:     return 0xFFFF6666;
        case COLOR_DARK_RED:      return 0xFF8B0000;

        case COLOR_GREEN:         return 0xFF00FF00;
        case COLOR_LIGHT_GREEN:   return 0xFF90EE90;
        case COLOR_DARK_GREEN:    return 0xFF006400;

        case COLOR_BLUE:          return 0xFF0000FF;
        case COLOR_LIGHT_BLUE:    return 0xFFADD8E6;
        case COLOR_DARK_BLUE:     return 0xFF00008B;

        case COLOR_YELLOW:        return 0xFFFFFF00;
        case COLOR_LIGHT_YELLOW:  return 0xFFFFFFE0;
        case COLOR_DARK_YELLOW:   return 0xFFCCCC00;

        case COLOR_CYAN:          return 0xFF00FFFF;
        case COLOR_LIGHT_CYAN:    return 0xFFE0FFFF;
        case COLOR_DARK_CYAN:     return 0xFF008B8B;

        case COLOR_MAGENTA:       return 0xFFFF00FF;
        case COLOR_LIGHT_MAGENTA: return 0xFFFFB3FF;
        case COLOR_DARK_MAGENTA:  return 0xFF8B008B;

        case COLOR_BROWN:         return 0xFFA52A2A;
        case COLOR_ORANGE:        return 0xFFFFA500;
        case COLOR_PINK:          return 0xFFFFC0CB;
        case COLOR_BEIGE:         return 0xFFF5F5DC;
        case COLOR_TEAL:          return 0xFF008080;
        case COLOR_NAVY:          return 0xFF000080;
        case COLOR_OLIVE:         return 0xFF808000;
        case COLOR_MAROON:        return 0xFF800000;
        case COLOR_GOLD:          return 0xFFFFD700;
        case COLOR_SILVER:        return 0xFFC0C0C0;

        default: return 0xFF000000; // Fallback: black
    }
}
