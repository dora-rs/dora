import random
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from carla import Client, Location, Rotation, Transform, command
from lanenet_model import lanenet, lanenet_postprocess

LANENET_MODEL_PATH = "dependencies/models/lane_detection/lanenet/carla_town_1_2/tusimple"
tf.compat.v1.disable_eager_execution()

class LanePredictor():
    def __init__(self, weights):
        """ Initializes a LanePredictor which is used to register a callback
        for the RGB images and predict lanes.
        Args:
            weights: The path of the weights to be used in the prediction.
            config: The config to be used for tensorflow.
        """
        self._input_tensor = tf.compat.v1.placeholder(dtype=tf.float32,
                                                      shape=[1, 256, 512, 3],
                                                      name='input_tensor')
        net = lanenet.LaneNet(phase='test')
        self._binary_seg_ret, self._instance_seg_ret = net.inference(
            input_tensor=self._input_tensor, name='LaneNet')
        self._gpu_options = tf.compat.v1.GPUOptions(
            allow_growth=True,
            per_process_gpu_memory_fraction=0.5,
            allocator_type='BFC')
        self._tf_session = tf.compat.v1.Session(
            config=tf.compat.v1.ConfigProto(gpu_options=self._gpu_options,
                                            allow_soft_placement=True))
        with tf.compat.v1.variable_scope(name_or_scope='moving_avg'):
            variable_averages = tf.train.ExponentialMovingAverage(0.9995)
            variables_to_restore = variable_averages.variables_to_restore()

        self._postprocessor = lanenet_postprocess.LaneNetPostProcessor()
        saver = tf.compat.v1.train.Saver(variables_to_restore)
        with self._tf_session.as_default():
            saver.restore(sess=self._tf_session,
                          save_path=LANENET_MODEL_PATH)

    def process_images(self, msg):
        """ This function runs the LaneNet model on each of the image retrieved
        from the simulator.
        Args:
            msg: The RGB image received from the camera.
        """
        # Convert the BGRA image to BGR.
        image = np.frombuffer(msg.raw_data, dtype=np.dtype('uint8'))
        image = np.reshape(image, (msg.height, msg.width, 4))[:, :, :3]
        image_vis = image
        image = cv2.resize(image, (512, 256), interpolation=cv2.INTER_LINEAR)
        resized_image = image
        image = image / 127.5 - 1.0

        binary_seg_image, instance_seg_image = self._tf_session.run(
            [self._binary_seg_ret, self._instance_seg_ret],
            feed_dict={self._input_tensor: [image]})

        postprocess_result = self._postprocessor.postprocess(
            binary_seg_result=binary_seg_image[0],
            instance_seg_result=instance_seg_image[0],
            source_image=image_vis)
        mask_image = postprocess_result['mask_image']
        if mask_image is not None:
            plt.figure('mask_image')
            plt.imshow(
                 cv2.addWeighted(resized_image[:, :, (2, 1, 0)], 1,
                                 mask_image[:, :, (2, 1, 0)], 0.3, 0)
            )
            plt.show()


def spawn_driving_vehicle(client, world):
    """ This function spawns the driving vehicle and puts it into
    an autopilot mode.
    Args:
        client: The Client instance representing the simulation to
          connect to.
        world: The world inside the current simulation.
    Returns:
        A Actor instance representing the vehicle that was just spawned.
    """
    # Get the blueprint of the vehicle and set it to AutoPilot.
    vehicle_bp = random.choice(
        world.get_blueprint_library().filter('vehicle.*'))
    while not vehicle_bp.has_attribute('number_of_wheels') or not int(
            vehicle_bp.get_attribute('number_of_wheels')) == 4:
        vehicle_bp = random.choice(
            world.get_blueprint_library().filter('vehicle.*'))
    vehicle_bp.set_attribute('role_name', 'autopilot')

    # Get the spawn point of the vehicle.
    start_pose = random.choice(world.get_map().get_spawn_points())

    # Spawn the vehicle.
    batch = [
        command.SpawnActor(vehicle_bp, start_pose).then(
            command.SetAutopilot(command.FutureActor, True))
    ]
    vehicle_id = client.apply_batch_sync(batch)[0].actor_id
    while world.get_actors().find(vehicle_id) is None:
        
        # Find the vehicle and return the Actor instance.
        time.sleep(5)  # This is so that the vehicle gets registered in the actors.
    return world.get_actors().find(vehicle_id)


def spawn_rgb_camera(world, location, rotation, vehicle):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1280')
    camera_bp.set_attribute('image_size_y', '720')
    transform = Transform(location=location, rotation=rotation)
    return world.spawn_actor(camera_bp, transform, attach_to=vehicle)


def main(lane_predictor):
    client = Client('192.168.1.15', 2000)
    world = client.get_world()

    # Spawn the vehicle.
    vehicle = spawn_driving_vehicle(client, world)
    assert vehicle is not None, "Vehicle is None"
    # Spawn the camera and register a function to listen to the images.
    camera = spawn_rgb_camera(world, Location(x=2.0, y=0.0, z=1.8),
                              Rotation(roll=0, pitch=0, yaw=0), vehicle)
    camera.listen(lane_predictor.process_images)

    return vehicle, camera, world


if __name__ == "__main__":
    vehicle, camera, world = main(
        LanePredictor(LANENET_MODEL_PATH))
    try:
        while True:
            time.sleep(1 / 100.0)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        # Destroy the actors.
        vehicle.destroy()
        camera.destroy()
