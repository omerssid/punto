// The stb_image_write implementation, isolated into its own translation
// unit so the project's warning flags can be relaxed for it alone.
//
// Vendored third-party code is not ours to fix, and compiling it under
// -Werror makes the build hostage to whichever warnings the next compiler
// release decides to add.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
