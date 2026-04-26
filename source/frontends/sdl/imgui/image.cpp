#include "frontends/sdl/imgui/image.h"

#ifdef GL_UNPACK_ROW_LENGTH
// GLES3: defined in gl3.h
#define UGL_UNPACK_LENGTH GL_UNPACK_ROW_LENGTH
#else
// GLES2: defined in gl2ext.h as an extension GL_EXT_unpack_subimage
#define UGL_UNPACK_LENGTH GL_UNPACK_ROW_LENGTH_EXT
#endif

namespace sa2
{

    /*
      2 extensions are used here

      ES2: GL_UNPACK_ROW_LENGTH_EXT and GL_BGRA_EXT
      ES3: GL_BGRA_EXT

      They both work on Pi3/4 and my Intel Haswell card.

      What does SDL do about it?

      GL_UNPACK_ROW_LENGTH_EXT: it uses instead texture coordinates
      GL_BGRA_EXT: it uses a shader to swap the color bytes

      For simplicity (and because it just works) here we are using these 2 extensions.
    */

    void allocateTexture(GLuint texture, size_t width, size_t height)
    {
        glBindTexture(GL_TEXTURE_2D, texture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

#if defined(__EMSCRIPTEN__) || defined(GL_ES_VERSION_3_0)
        glTexStorage2D(GL_TEXTURE_2D, 1, SA2_IMAGE_FORMAT_INTERNAL, width, height);
#else
        glTexImage2D(
            GL_TEXTURE_2D, 0, SA2_IMAGE_FORMAT_INTERNAL, width, height, 0, SA2_IMAGE_FORMAT, GL_UNSIGNED_BYTE, nullptr);
#endif

        // Setup filtering parameters for display
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    void loadTextureFromData(GLuint texture, const uint8_t *data, size_t width, size_t height, size_t pitchPixels)
    {
        glBindTexture(GL_TEXTURE_2D, texture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glPixelStorei(UGL_UNPACK_LENGTH, pitchPixels); // in pixels
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, SA2_IMAGE_FORMAT, GL_UNSIGNED_BYTE, data);
        // reset to default state
        glPixelStorei(UGL_UNPACK_LENGTH, 0);
    }

} // namespace sa2
