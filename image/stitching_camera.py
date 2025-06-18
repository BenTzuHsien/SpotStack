import numpy, os, pygame
from PIL import Image
from OpenGL.GL import *
from OpenGL.GLU import gluPerspective, gluLookAt
from SpotStack.image.utils.compiled_shader import CompiledShader

def proto_vec_T_numpy(vec):
    return numpy.array([vec.x, vec.y, vec.z])


def mat4mul3(mat, vec, vec4=1):
    ret = numpy.matmul(mat, numpy.append(vec, vec4))
    return ret[:-1]


def normalize(vec):
    norm = numpy.linalg.norm(vec)
    if norm == 0:
        raise ValueError("norm function returned 0.")
    return vec / norm

class StitchingCamera:
    """
    Virtual OpenGL-based camera for stitching Spot's front fisheye images.

    This class sets up an off-screen OpenGL context using `pygame` and 
    `PyOpenGL`, compiles and uses a custom shader to blend two front camera 
    images (left and right), and renders them from a virtual stitched viewpoint.

    It assumes the images are preprocessed and formatted into `ImagePreppedForOpenGL`.

    Attributes
    ----------
    FRONT_IMAGE_WIDTH : int
        Width of the output stitched image.
    FRONT_IMAGE_HEIGHT : int
        Height of the output stitched image.
    SHADER_ASSET_PATH : str
        Path to the directory containing GLSL shader files.
    RECT_STITCHING_DISTANCE_METERS : float
        (Need ti be confirmed) Distance from the virtual camera to the rendering plane.
    """
    FRONT_IMAGE_WIDTH = 1080
    FRONT_IMAGE_HEIGHT = 720
    SHADER_ASSET_PATH = os.path.join(os.path.dirname(__file__), 'assets')
    RECT_STITCHING_DISTANCE_METERS = 2.0

    def __init__(self):
        """
        Initialize the StitchingCamera.

        This sets up a hidden Pygame window with an OpenGL context and loads vertex and fragment shaders from the local asset directory.
        """
        display = (self.FRONT_IMAGE_WIDTH, self.FRONT_IMAGE_HEIGHT)
        pygame.init()
        pygame.display.set_mode(display, pygame.DOUBLEBUF | pygame.OPENGL | pygame.HIDDEN)

        with open(os.path.join(self.SHADER_ASSET_PATH, 'shader_vert.glsl'), 'r') as file:
            vert_shader = file.read()
        with open(os.path.join(self.SHADER_ASSET_PATH, 'shader_frag.glsl'), 'r') as file:
            frag_shader = file.read()

        self.stitching_program = CompiledShader(vert_shader, frag_shader)

    @staticmethod
    def _draw_geometry(plane_wrt_vo, plane_norm_wrt_vo, sz_meters):
        """Draw as GL_TRIANGLES."""
        plane_left_wrt_vo = normalize(numpy.cross(numpy.array([0, 0, 1]), plane_norm_wrt_vo))
        if plane_left_wrt_vo is None:
            return
        plane_up_wrt_vo = normalize(numpy.cross(plane_norm_wrt_vo, plane_left_wrt_vo))
        if plane_up_wrt_vo is None:
            return

        plane_up_wrt_vo = plane_up_wrt_vo * sz_meters
        plane_left_wrt_vo = plane_left_wrt_vo * sz_meters

        vertices = (
            plane_wrt_vo + plane_left_wrt_vo - plane_up_wrt_vo,
            plane_wrt_vo + plane_left_wrt_vo + plane_up_wrt_vo,
            plane_wrt_vo - plane_left_wrt_vo + plane_up_wrt_vo,
            plane_wrt_vo - plane_left_wrt_vo - plane_up_wrt_vo,
        )

        indices = (0, 1, 2, 0, 2, 3)

        glBegin(GL_TRIANGLES)
        for index in indices:
            glVertex3fv(vertices[index])
        glEnd()

    def stitch_images(self, front_right, front_left):
        """
        Stitch front-left and front-right fisheye images into a single wide-angle image.

        The method computes a virtual camera located between the two input images,
        sets up projection and modelview matrices, and renders the stitched output 
        using OpenGL and custom shaders.

        Parameters
        ----------
        front_right : ImagePreppedForOpenGL
            The right fisheye image wrapped for OpenGL usage.
        front_left : ImagePreppedForOpenGL
            The left fisheye image wrapped for OpenGL usage.

        Returns
        -------
        PIL.Image
            The final stitched image as a PIL Image object.
        """
        self.stitching_program.update_images(front_right, front_left)

        # Calculate Camera Parameter
        vo_T_body = front_right.vision_T_body.to_matrix()
        
        eye_wrt_body = proto_vec_T_numpy(front_right.body_T_image_sensor.position) \
                     + proto_vec_T_numpy(front_left.body_T_image_sensor.position)
        
        # Add the two real camera norms together to get the fake camera norm.
        eye_norm_wrt_body = numpy.array(front_right.body_T_image_sensor.rot.transform_point(0, 0, 1)) \
                          + numpy.array(front_left.body_T_image_sensor.rot.transform_point(0, 0, 1))
        
        # Make the virtual camera centered.
        eye_wrt_body[1] = 0
        eye_norm_wrt_body[1] = 0

        # Make sure our normal has length 1
        eye_norm_wrt_body = normalize(eye_norm_wrt_body)

        plane_wrt_body = eye_wrt_body + eye_norm_wrt_body * self.RECT_STITCHING_DISTANCE_METERS

        plane_wrt_vo = mat4mul3(vo_T_body, plane_wrt_body)
        plane_norm_wrt_vo = mat4mul3(vo_T_body, eye_norm_wrt_body, 0)

        eye_wrt_vo = mat4mul3(vo_T_body, eye_wrt_body)
        up_wrt_vo = mat4mul3(vo_T_body, numpy.array([0, 0, 1]), 0)


        # Rendering
        glClearColor(0, 0, 255, 0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # draw_routine
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(110, (self.FRONT_IMAGE_WIDTH / self.FRONT_IMAGE_HEIGHT), 0.1, 50.0)
        if not self.stitching_program.initialized:
            print("Gl is not ready yet.")
            return
        
        glUseProgram(self.stitching_program.program)
        glActiveTexture(GL_TEXTURE0 + 0)

        with self.stitching_program.image1.manage_bind():
            glUniform1i(self.stitching_program.image1_texture, 0)
            glActiveTexture(GL_TEXTURE0 + 1)

            with self.stitching_program.image2.manage_bind():

                glUniform1i(self.stitching_program.image2_texture, 1)

                glUniformMatrix4fv(self.stitching_program.camera1_MVP, 1, GL_TRUE, self.stitching_program.matrix1)
                glUniformMatrix4fv(self.stitching_program.camera2_MVP, 1, GL_TRUE, self.stitching_program.matrix2)

                glMatrixMode(GL_MODELVIEW)
                glLoadIdentity()
                gluLookAt(eye_wrt_vo[0], eye_wrt_vo[1], eye_wrt_vo[2], \
                        plane_wrt_vo[0], plane_wrt_vo[1], plane_wrt_vo[2], \
                        up_wrt_vo[0], up_wrt_vo[1], up_wrt_vo[2])
                
                rect_sz_meters = 7
                self._draw_geometry(plane_wrt_vo, plane_norm_wrt_vo, rect_sz_meters)

        # Get Image from OpenGL
        
        # Read pixels from OpenGL framebuffer
        glPixelStorei(GL_PACK_ALIGNMENT, 1)
        pixels = glReadPixels(0, 0, self.FRONT_IMAGE_WIDTH, self.FRONT_IMAGE_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE)
        
        # Convert raw pixel data to a NumPy array
        image_array = numpy.frombuffer(pixels, dtype=numpy.uint8).reshape(self.FRONT_IMAGE_HEIGHT, self.FRONT_IMAGE_WIDTH, 3)
        
        # OpenGL gives images with inverted Y-axis; flip it
        image_array = numpy.flipud(image_array)
        
        # Convert NumPy array to a PIL Image and save it
        image = Image.fromarray(image_array)

        return image
    
# Example Usage
if __name__ == '__main__':

    import pickle

    pair_path = ''
    save_path = os.path.join(os.path.dirname(__file__), 'stitched_image.jpg')

    with open(pair_path, "rb") as file:
        image_pairs = pickle.load(file)

    # Create a instance for stitching image
    stitching_camera = StitchingCamera()

    # Stitch image pair
    stitched_image = stitching_camera.stitch_images(image_pairs[0], image_pairs[1])

    stitched_image.save(save_path)