import io, torch
from PIL import Image
from SpotStack.image.utils.image_prepped_for_opengl import ImagePreppedForOpenGL
from SpotStack.image.stitching_camera import StitchingCamera

from bosdyn.client.image import ImageClient, build_image_request

class ImageFetcher:
    """
    Fetch and optionally stitch fisheye images from a Boston Dynamics Spot robot.

    This class provides a unified interface for retrieving fisheye images from the robot. 
    Users should always call `get_images()` rather than calling the lower-level methods directly. 
    
    Depending on the `use_front_stitching` flag, 
    it dynamically binds `self.get_images()` to either `get_five_images()` (raw 5-camera capture) or `get_four_images()` (stitched front and 3 other cameras).

    Attributes
    ----------
    sources : list of str
        List of fisheye camera sources on the Spot robot.
    ROTATION_ANGLE : dict
        Mapping of image source names to the corresponding rotation in degrees.
    SOURCE_NUMBER : dict
        Mapping of image source names to their index positions in the final image array.
    """
    sources = ['frontleft_fisheye_image', 'frontright_fisheye_image', 'back_fisheye_image', 'left_fisheye_image', 'right_fisheye_image']
    ROTATION_ANGLE = {
        'back_fisheye_image': 0,
        'frontleft_fisheye_image': -78,
        'frontright_fisheye_image': -102,
        'left_fisheye_image': 0,
        'right_fisheye_image': 180
    }
    SOURCE_NUMBER = {
        'back_fisheye_image': 3,
        'frontleft_fisheye_image': 0,
        'frontright_fisheye_image': 1,
        'left_fisheye_image': 2,
        'right_fisheye_image': 4
    }

    def __init__(self, robot, pixel_format='PIXEL_FORMAT_RGB_U8', use_front_stitching=False):
        """
        Initialize the ImageFetcher.

        Parameters
        ----------
        robot : bosdyn.client.robot.Robot
            The Spot robot instance.
        pixel_format : str, optional
            The pixel format used when requesting images (default is 'PIXEL_FORMAT_RGB_U8').
        use_front_stitching : bool, optional
            Whether to stitch the front-left and front-right images into a single image (default is False).
        """
        self._robot = robot

        # Force trigger timesync
        self._robot.time_sync.wait_for_sync()

        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._image_request = [
            build_image_request(source, quality_percent=100, pixel_format=pixel_format)
            for source in self.sources
        ]

        if use_front_stitching is True:
            self.stitching_camera = StitchingCamera()
            self.get_images = self.get_four_images
        else:
            self.get_images = self.get_five_images

    @classmethod
    def _decrypt_image(cls, image_response, auto_rotate=True, data_transform=None):
        """
        Decode and optionally rotate and transform an image from Spot.

        Parameters
        ----------
        image_response : bosdyn.api.image_pb2.ImageResponse
            Image response object received from Spot's image service.
        auto_rotate : bool, optional
            Whether to auto-rotate the image based on its source (default is True).
        data_transform : Callable, optional
            Optional transform (e.g., torchvision transform) applied to the PIL image.

        Returns
        -------
        int
            Index corresponding to the image's source camera.
        PIL.Image or transformed object
            Decoded and optionally rotated and transformed image.
        """
        img = Image.open(io.BytesIO(image_response.shot.image.data))

        if auto_rotate:
            img = img.rotate(cls.ROTATION_ANGLE[image_response.source.name])
        if data_transform:
            img = data_transform(img)

        img_nubmber = cls.SOURCE_NUMBER.get(image_response.source.name, -1)

        return img_nubmber, img
    
    def _stitch_front_images(self, front_right, front_left, data_transform=None):
        """
        Stitch front-left and front-right OpenGL-prepped images into one combined image.

        Parameters
        ----------
        front_right : ImagePreppedForOpenGL
            Front-right image formatted for stitching.
        front_left : ImagePreppedForOpenGL
            Front-left image formatted for stitching.
        data_transform : Callable, optional
            Optional transform (e.g., torchvision transform) applied to the PIL image.

        Returns
        -------
        PIL.Image
            The stitched front image.
        """
        if  self.stitching_camera is None:
            self.stitching_camera = StitchingCamera()

        stitched_image = self.stitching_camera.stitch_images(front_right, front_left)

        if data_transform:
            stitched_image = data_transform(stitched_image)

        return stitched_image

    # Implementation of getting images from robot
    def get_images(self, data_transform=None):
        """
        Virtual image retrieval method — replaced at runtime based on stitching configuration.

        Call this method to retrieve images from the Spot robot. This will point to either
        `get_five_images()` or `get_four_images()` depending on construction settings.

        Parameters
        ----------
        data_transform : Callable, optional
            Optional transformation applied to images (e.g., normalization or resizing).

        Returns
        -------
        list
            List of 4 or 5 images as PIL.Image or transformed object ordered by SOURCE_NUMBER.
        """
        raise NotImplementedError("Use the instance method bound at construction time.")
    
    def get_five_images(self, data_transform=None):
        """
        Retrieve and decode all 5 raw fisheye camera images from the robot.

        Parameters
        ----------
        data_transform : Callable, optional
            Optional transformation applied to images (e.g., normalization or resizing).

        Returns
        -------
        list
            List of 5 images (PIL.Image or transformed format) ordered by SOURCE_NUMBER.
        """
        images = [None] * 5
        image_responses = self._image_client.get_image(self._image_request)

        for image_response in image_responses:
            image_number, image = self._decrypt_image(image_response, data_transform=data_transform)
            images[image_number] = image
        
        if data_transform:
            images = torch.stack(images).unsqueeze(0)
            
        return images
    
    def get_four_images(self, data_transform=None):
        """
        Retrieve 3 raw side/back images and a stitched front image.

        This method uses OpenGL to stitch front-left and front-right images into a
        single front image, and returns 3 remaining images (back, left, right) directly.

        Parameters
        ----------
        data_transform : Callable, optional
            Optional transformation applied to images (e.g., normalization or resizing).

        Returns
        -------
        list
            List of 4 images where the stitched front image is placed at index 0.

            The remaining images (back, left, right) follow in order according to `SOURCE_NUMBER`,
            shifted forward by one to account for the stitched front image.
        """
        images = [None] * 4
        image_responses = self._image_client.get_image(self._image_request)

        for image_response in image_responses:

            if image_response.source.name == "frontright_fisheye_image":
                front_right = ImagePreppedForOpenGL(image_response)

            elif image_response.source.name == "frontleft_fisheye_image":
                front_left = ImagePreppedForOpenGL(image_response)

            else:
                image_number, image = self._decrypt_image(image_response, data_transform=data_transform)
                images[image_number - 1] = image

        images[0] = self.stitching_camera.stitch_images(front_right, front_left)
        
        if data_transform:
            images = torch.stack(images).unsqueeze(0)
            
        return images

    def get_images_with_fisheye_pair(self, data_transform=None):
        """
        Retrieve all 5 raw images and extract the raw front-left/right OpenGL prepped image pair.

        This method is useful if you want access to both the full image list and the
        raw front image pair for manual or custom stitching later.

        Parameters
        ----------
        data_transform : Callable, optional
            Optional transformation applied to images (e.g., normalization or resizing).

        Returns
        -------
        list
            List of 5 images (PIL.Image or transformed format) ordered by SOURCE_NUMBER.
        tuple of ImagePreppedForOpenGL
            Tuple of (front_right, front_left) OpenGL-ready images.
        """
        images = [None] * 5
        image_responses = self._image_client.get_image(self._image_request)

        for image_response in image_responses:
            image_number, image = self._decrypt_image(image_response)
            images[image_number] = image

            # Collect Data for Stitching Front Images
            if image_response.source.name == "frontright_fisheye_image":
                front_right = ImagePreppedForOpenGL(image_response)
            if image_response.source.name == "frontleft_fisheye_image":
                front_left = ImagePreppedForOpenGL(image_response)
        
        if data_transform:
            images = torch.stack(images).unsqueeze(0)

        if front_right is not None and front_left is not None:
                image_pair = (front_right, front_left)
            
        return images, image_pair
    
# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, sys, os, time, select, termios, tty

    def key_pressed():
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        return dr != []
    def get_key():
        return sys.stdin.read(1) if key_pressed() else None

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('ImageFetcher')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    image_fetcher = ImageFetcher(robot, use_front_stitching=True)
    
    # The example observation will be stored under the 'test' folder under the image module
    save_path = os.path.join(os.path.dirname(__file__), 'test')
    if not os.path.exists(save_path):
        os.mkdir(save_path)
    index = 0

    # Showing how to get front_image_pair
    import pickle
    _, pair = image_fetcher.get_images_with_fisheye_pair()
    img_pair_save_path = os.path.join(save_path, 'image_pair.pkl')
    with open(img_pair_save_path, "wb") as file:
        pickle.dump(pair, file)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        past_time = time.time()
        while True:
            key = get_key()
            if key == 'q':
                print("Quitting...")
                break
            
            # Get images from the robot
            images = image_fetcher.get_images()

            image_save_dir = os.path.join(save_path, f'{index:03}')
            os.mkdir(image_save_dir)
            i = 0
            
            for image in images:
                image_save_path = os.path.join(image_save_dir, f'{i}.jpg')
                image.save(image_save_path)
                i += 1

            index += 1
            duration = time.time() - past_time
            print(duration)
            time.sleep(0.2 - duration)
            past_time = time.time()

    except Exception as exc:  # pylint: disable=broad-except
                print("ImageFetcher threw an error.")
                print(exc)

    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, old_settings)
        