from OpenGL.GL import shaders, GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, glGetUniformLocation
from SpotStack.image.utils.image_inside_opengl import ImageInsideOpenGL

class CompiledShader():
    """OpenGL shader compile"""

    def __init__(self, vert_shader, frag_shader):
        self.program = shaders.compileProgram( \
            shaders.compileShader(vert_shader, GL_VERTEX_SHADER), \
            shaders.compileShader(frag_shader, GL_FRAGMENT_SHADER) \
        )
        self.camera1_MVP = glGetUniformLocation(self.program, 'camera1_MVP')
        self.camera2_MVP = glGetUniformLocation(self.program, 'camera2_MVP')

        self.image1_texture = glGetUniformLocation(self.program, 'image1')
        self.image2_texture = glGetUniformLocation(self.program, 'image2')

        self.initialized = False
        self.image1 = None
        self.image2 = None
        self.matrix1 = None
        self.matrix2 = None

    def update_images(self, front_right, front_left):
        self.initialized = True
        self.matrix1 = front_right.MVP
        self.matrix2 = front_left.MVP
        if self.image1 is None:
            self.image1 = ImageInsideOpenGL(front_right.image)
        else:
            self.image1.update(front_right.image)
        if self.image2 is None:
            self.image2 = ImageInsideOpenGL(front_left.image)
        else:
            self.image2.update(front_left.image)