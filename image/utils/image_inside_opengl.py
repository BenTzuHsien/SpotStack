from OpenGL.GL import *
from contextlib import contextmanager

class ImageInsideOpenGL():
    """Create OpenGL Texture"""

    def __init__(self, numpy_array):
        glEnable(GL_TEXTURE_2D)
        self.pointer = glGenTextures(1)
        with self.manage_bind():
            if numpy_array.ndim == 3 and numpy_array.shape[2] == 3:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, numpy_array.shape[1], numpy_array.shape[0], 0, \
                    GL_RGB, GL_UNSIGNED_BYTE, numpy_array)
            else:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, numpy_array.shape[1], numpy_array.shape[0], 0, \
                    GL_LUMINANCE, GL_UNSIGNED_BYTE, numpy_array)
            glTexParameter(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameter(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    @contextmanager
    def manage_bind(self):
        glBindTexture(GL_TEXTURE_2D, self.pointer)  # bind image
        try:
            yield
        finally:
            glBindTexture(GL_TEXTURE_2D, 0)  # unbind image

    def update(self, numpy_array):
        """Update texture"""
        with self.manage_bind():
            if numpy_array.ndim == 3 and numpy_array.shape[2] == 3:
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, numpy_array.shape[1], numpy_array.shape[0], \
                    GL_RGB, GL_UNSIGNED_BYTE, numpy_array)
            else:
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, numpy_array.shape[1], numpy_array.shape[0], \
                    GL_LUMINANCE, GL_UNSIGNED_BYTE, numpy_array)