#!/usr/bin/env python

################################################################################
#
# Generates images + keypoint annotations of a 3D model rendered in Gazebo.
# Compiles multiple instances into an HDF5 dataset.
#   Model's pose is changed randomly each instance.
#
# See README for more info, including dataset schema, etc.
#
# Author: Jack Rasiel
# Contact: j{last name} {at} cs {dot} umd {dot} edu
#
################################################################################

import numpy as np
import h5py
import cv2 # For testing only
import matplotlib.pyplot as plt # For testing only
import xml.etree.ElementTree as ET # For loading keypoints from .pp files.
from time import sleep, time
from os.path import isfile
from subprocess import Popen

import rospy
from rospkg.rospack import RosPack
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from transformer_manager import TransformerManager
from gazebo_model_manager import GazeboModelManager, GazeboCameraManager
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point, Quaternion, Pose


# Rand sample from SO(3): planning.cs.uiuc.edu/node198.html
def rand_orientation():
    u1, u2, u3 = np.random.rand(3)
    a, b = np.sqrt((1-u1, u1))
    tau = 2*np.pi
    x,y,z,w = a*np.sin(tau * u2), a*np.cos(tau * u2), b*np.sin(tau * u3), b*np.cos(tau * u3)
    return x,y,z,w

# Save matplotlib image to jpeg, then load jpeg file binary data as np uint8[].
# (The array can then be written to the dataset)
def img_compress_and_get_binary(img):
    plt.imsave('/tmp/img_compressed.jpg', img)
    fin = open('/tmp/img_compressed.jpg','rb')
    binary_data = fin.read()
    binary_data = np.fromstring(binary_data,dtype='uint8')
    fin.close()
    return binary_data

# Converts jpeg file as uint8 arry to an actual jpeg img:
def write_binary_to_jpeg(jpeg_binary, filepath = "./", filename = "dset_img.jpeg"):
    # Convert uint8 binary to bytestring:
    file_string = jpeg_binary.tostring()
    outfile = open(filepath + filename,'wb')
    outfile.write(file_string)
    outfile.close()
    return

# Given h5py dataset (e.g. dset['image'] in main()), writes all images to jpeg
#  Files are written to dest_dir with filenames img%d
def write_all_images_to_files(dset, dest_dir):
    if len(dest_dir) == 0 or dest_dir[-1] != "/":
        raise Exception("dest_dir must end in \"/\"")
    jpeg_binary = dset[0]
    i = 0
    while len(jpeg_binary) >0:
        filename = "img%03d.jpeg" % i
        write_binary_to_jpeg(jpeg_binary, filepath = dest_dir,
                             filename = filename)
        if i % 25 == 0:
            print "Saved img %d" % i
        i+=1
        jpeg_binary = dset[i]
    print "Finished after saving %d imgs." % i
    print "Imgs saved to %s" % dest_dir

# Loads keypoints from meshlab .pp file.  Returns list of (x,y,z) tuples.
def load_keypoints(keypoints_file):
    tree = ET.parse(keypoints_file)
    points = tree.findall('point')
    points = [(float(pt.attrib['x']), float(pt.attrib['y']), float(pt.attrib['z'])) 
              for pt in points]
    return points

#TODO move to separate tests module.
# For debugging: load model and spawn sphere at each position in keypoints.
def test_keypoint_locs(keypoints, model_name):
    model_manager = GazeboModelManager(models_dir="/home/jack/.gazebo/models")
    raw_input("Be sure to disable physics in gazebo! (Press enter to continue)")
    model_manager.spawn_model(model_name,model_name)
    sleep(0.5)
    for i in range(len(keypoints)):
        x, y, z = keypoints[i]
        model_manager.spawn_point_marker(str(i),x=x,y=y,z=z)
        raw_input("Spawned marker %d (press enter)\t\t%f\t%f\t%f" % (i,x,y,z))
        sleep(0.25)
    print "All markers spawned."
    return

# For debugging: draw circles at img coordinates of keypoints:
def show_keypoint_locs(keypoints, img):
    img = np.uint8(img)
    for (u,v) in keypoints:
        cv2.circle(img, (u,v), 6, (0,0,255),2)
        cv2.circle(img, (u,v), 1, (0,0,255),-1)
    # DEBUG: show annotated img.  WARNING, opencv imshoe causes program to hang.
    #cv2.imshow( "oy vey",img)
    #cv2.waitKey(0)
    return img

class ModelKeypointDatasetGenerator(object):
    """
    TODO class description
    """

    def __init__(self, dset_name, model_name, num_images, camera_dist,
                 do_clutter = False, clutter_exclude_region = None,
                 model_scale_factor = 1,
                 camera_info = CameraInfo(width=640, height=480, 
                                          K=[554.3827128226441, 0.0, 320.5, 
                                             0.0, 554.3827128226441, 240.5, 
                                             0.0, 0.0, 1.0]),
                 ):
        self.DEBUG = False

        self.model_name = model_name
        # Model scale factor must match <scale> element in model's sdf file:
        self.model_scale_factor = model_scale_factor
        # Number of images to be added to the dataset:
        self.num_images = num_images
        self.cam_dist = camera_dist
        self.cam_info = camera_info
        # Toggle addition of other objects to clutter the images:
        self.do_clutter = do_clutter
        # Bounding rectangle, in the object's frame, within which clutter is not allowed:
        #   (max,min) for x,y,z axes
        self.clutter_exclude_region = clutter_exclude_region 
        # cam info for RGB cam in simulated kinect Top bottom-left

        # Full path+filename of dataset.
        # By default puts it in the package's "datasets" dir:
        package_path = RosPack().get_path('model_pose_dataset_generation') 
        self.dset_dir = package_path + "/datasets/" + dset_name

        ### Load keypoints and create dataset: ###
        keypoints_file = package_path + "/models/" + self.model_name + "/keypoints.pp"
        if isfile(keypoints_file):
            self.keypoints = load_keypoints(keypoints_file)
            SF = self.model_scale_factor
            self.keypoints = [(x*SF,y*SF,z*SF) for (x,y,z) in self.keypoints]
        else:
            raise Exception("Keypoints file not found! %s" % keypoints_file)

        # Create and configure new dataset:
        if not isfile(self.dset_dir):
            self.dset = h5py.File(self.dset_dir, mode='a')
            self.configure_dataset()
            rospy.on_shutdown(self.close_dset)
        else:
            # TODO kluge, use a loop or prompt.
            raw_input("Dset exists! rm it and then come back to me (and press enter).")
            self.dset = h5py.File(self.dset_dir, mode='a')
            self.dset = self.configure_dataset()
            #rospy.on_shutdown(self.close_dset)
            #raise Exception("Dataset already exists: %s" % self.dset_dir)

        ### Set up gazebo interfaces:
        # Spawn camera, model, and tf manager:
        self.tf_manager = TransformerManager()
        self.model_manager = GazeboModelManager()
        #TODO REP with service proxy and waiting for service:
        self.cam_manager = GazeboCameraManager()


    def close_dset(self):
        if self.dset is not None:
            self.dset.close()
            print "Datset safely closed."
        return


    def generate_dataset(self):

        raw_input("Be sure to disable physics in gazebo! (Press enter to continue)")

        self.model_manager.spawn_model(self.model_name,self.model_name) 
        self.cam_manager.spawn_camera() # TODO REP remove.
        # TODO DEBUG:
        # For some reason the camera sensor view will unpredictably reset to Pose(),
        #   but the little red cube model appears to remain in place.
        # Mysteriously this is prevented by leaving rqt_image_view running:
        Popen("rqt_image_view") 
        sleep(0.7)
        # TODO REP set pose srv
        self.cam_manager.set_model_state(pose=Pose(Point(self.cam_dist,0,0),Quaternion(0,0,1,0)))

        ### Main loop: gen rand camera pose.  Capture image and keypoint loc.s (in camera frame)
        model_loc = Point()
        model_pose = Pose(model_loc, Quaternion(0,0,0,1))

        for i in range(self.num_images):
            sleep(0.2)
            print "%d\tCalling set_scene_and_get_img" % i
            # Randomizes pose and captures image:
            rgb, new_pose = self.set_scene_and_get_img()
            rgb = cv2.cvtColor(np.uint8(rgb), cv2.COLOR_BGR2RGB)        
            rgb = np.uint8(rgb)

            # Transform points into camera's image, and coordinate frame:
            points_in_image, points_in_camera =  self.project_keypoints_into_image()

            ###   Store everything to dataset.
            # Get object pose in cam frame:
            pose = self.tf_manager.transform_pose(new_pose, "Model","Camera")
            pose = pose.pose # actual pose from PoseStamped msg
            #print "Object in camera:", pose
            q, t = pose.orientation, pose.position
            pose_array = np.array((q.x,q.y,q.z,q.w, t.x, t.y, t.z))

            print "Compressing image %d and saving to dataset..." % i
            if self.dset == None:
                import IPython; IPython.embed()
            self.dset['image'][i] = img_compress_and_get_binary(rgb)
            p_i_i_array = [(a[0],b[0]) for (a,b) in points_in_image]
            self.dset['keypoints_in_image'][i,:] = p_i_i_array
            self.dset['keypoints_in_world'][i,:] = points_in_camera
            self.dset['model_pose'][i,:] = pose_array
            print "\t...saved."

            if self.DEBUG:
                print "%d\t DEBUG: keypoints in image:", (i,points_in_image)
                print "%d\t DEBUG: model pose (in cam frame):", (i,pose)
                print "%d\t DEBUG:  Showing result image" % i
                plt.imshow(rgb)
                plt.show()
                print "%d\t DEBUG:  Showing keypt locs in image" % i
                img_kpt_locs = show_keypoint_locs(points_in_image,rgb)
                plt.imshow(img_kpt_locs)
                plt.show()
                raw_input("press enter to continue")

        self.close_dset()
        print "Done!"
        return

    # Set random model pose, add clutter (if specified), take image of the scene.
    #
    # Camera and object must already be in the world.
    # do_clutter toggles addition of other objects as clutter.
    #
    # Returns rgb_image, new_camera_pose
    def set_scene_and_get_img(self):

        cam_pose = self.cam_manager.get_model_state().pose
        model_pose = self.model_manager.get_model_state(self.model_name).pose

        self.tf_manager.add_transform(cam_pose, "World", "Camera")
        self.tf_manager.add_transform(model_pose, "World", "Model")

        # Generate new object pose and apply it:
        x,y,z,w = rand_orientation()
        new_pose = Pose(Point(), Quaternion(x,y,z,w))
        self.model_manager.set_model_state(model_name=self.model_name, pose=new_pose)
        self.tf_manager.add_transform(new_pose, "World", "Model")

        # Add clutter, if specified:
        clutter_objects = []
        if do_clutter:
            clutter_objects = self.add_clutter()
            
        sleep(5) # Wait a bit for object meshes to appear in camera

        # Get RGBD image from cam; split color and depth imgs:
        rgb, stamp = self.cam_manager.get_RGB_and_stamp()
        print 'trying to get frames a bunch of times...'
        for i in range(10):
            rgb, stamp = self.cam_manager.get_RGB_and_stamp()
            now = rospy.timer.time.time()
            print "frame, now\t",stamp,now



        # Clean up the clutter:
        if do_clutter:
            print "removing clutter of %s" % clutter_objects
            self.remove_clutter(clutter_objects)

        return rgb, new_pose


    # Project points in world frame into image coordinates.
    # Also returns the points in the camera's frame.
    # TODO vectorize tf (instead of looping over each keypt.
    def project_keypoints_into_image(self):
        points_in_camera = []
        points_in_image = []
        for i in range(len(self.keypoints)):
            pt = self.keypoints[i]
            q = Quaternion(); q.z = 1 # dummy orientation
            in_model = Pose(Point(pt[0],pt[1],pt[2]),q)
            in_camera = self.tf_manager.transform_pose(in_model,"Model","Camera")
            in_camera = in_camera.pose.position

            # project point into image plane:
            width, height = self.cam_info.width, self.cam_info.height
            # Convert to camera coordinate axes from Gazebo's right-handed system:
            x, y, z = in_camera.y*-1.0, in_camera.z*-1.0, in_camera.x
            norm = np.array([[x],[y],[z]]) / float(z)
            k = np.array(self.cam_info.K).reshape((3,3))
            u, v, _ = np.dot(k, norm)
            # Append results:
            points_in_camera.append((x,y,z))
            points_in_image.append((u,v))
        return points_in_image, points_in_camera

    # Adds objects to the camera view to clutter its view of the target model.
    # Returns the names of the added objects, so they can be removed later.
    #
    # min_dist is the min distance from the model for clutter to be spawned
    #   (to avoid overlapping with it in visually unrealistic ways)
    # excluded_region is a rectange within which objects are excluded.
    #   ^^ TODO need to do this based on object's bounding box... not just its origin.
    # 
    def add_clutter(self, num_objects = 7, min_dist = .25):
        # A variety of objects from bigbird:
        # TODO what if user doesn't have bigbird?
        # Also TODO, load model list from file.
        model_prefix = "bigbird/" # subdirectory of gazebo models path
        object_pool = [
            "advil_liqui_gels",
            "band_aid_sheer_strips",
            "blue_clover_baby_toy",
            "bumblebee_albacore",
            "campbells_soup_at_hand_creamy_tomato",
            "cheez_it_white_cheddar",
            "chewy_dipps_peanut_butter",
            "cholula_chipotle_hot_sauce",
            "cinnamon_toast_crunch",
            "clif_z_bar_chocolate_chip",
            "coffee_mate_french_vanilla",
            "colgate_cool_mint",
            "crayola_24_crayons",
            "crest_complete_minty_fresh",
            "crystal_hot_sauce",
            "cup_noodles_shrimp_picante",
            "dove_go_fresh_burst",
            "eating_right_for_healthy_living_apple",
            "expo_marker_red",
            "haagen_dazs_butter_pecan",
            "honey_bunches_of_oats_honey_roasted",
            "hunts_paste",
            "ikea_table_leg_blue",
            "krylon_short_cuts",
            "mom_to_mom_butternut_squash_pear",
            "motts_original_assorted_fruit",
            "nature_valley_crunchy_oats_n_honey",
            "nice_honey_roasted_almonds",
            "nutrigrain_apple_cinnamon",
            "pepto_bismol",
            "pop_secret_butter",
            "pringles_bbq",
            "progresso_new_england_clam_chowder",
            "quaker_big_chewy_chocolate_chip", "red_bull",
            "red_cup",
            "ritz_crackers",
            "softsoap_white",
            "spongebob_squarepants_fruit_snaks",
            "suave_sweet_guava_nectar_body_wash",
            "tapatio_hot_sauce",
            "v8_fusion_strawberry_banana",
            "vo5_tea_therapy_healthful_green_tea_smoothing_shampoo",
            "white_rain_sensations_apple_blossom_hydrating_body_wash",
            "zilla_night_black_heat"
        ] 
        np.random.shuffle(object_pool)
        objects = object_pool[0:min(num_objects, len(object_pool)-1)]

        # Gen all spawn points.  Chosen randomly on XY plane.  Sampled in polar:
        theta = np.pi * np.random.randn(len(objects)) * 0.2
        outer_bound = cam_dist - 1
        inner_bound = min_dist
        rho = ((outer_bound - inner_bound) * np.random.rand(len(objects))) + inner_bound
        #z = -0.1  # Negative z offset accounts for bigbird model origin locations.
        z_lower, z_upper = -.3, .05
        Z = np.random.randn(len(objects)) * (z_upper - z_lower) - 0.175
        Z[Z > z_upper] = z_upper; Z[Z < z_lower] = z_lower # Cap Z within bounds
        X = np.multiply( rho, np.cos(theta))
        Y = np.multiply( rho, np.sin(theta))
        q = Quaternion(0,0,0,1)  # No rotation...

        x_bounds, y_bounds, z_bounds = None, None, None

        excluded_region = self.clutter_exclude_region
        check_excluded_region = False
        if excluded_region is not None:
            x_bounds, y_bounds, z_bounds = excluded_region
            check_excluded_region = True

        print "\tSpawning clutter (%d objects)" % len(objects)
        for i in range(len(objects)):
            name = model_prefix + objects[i]
            pose = Pose(Point(X[i], Y[i], Z[i]), q)
            
            # Check that spawnpoint is outside exclusion zone:
            while check_excluded_region:
                pose_m = self.tf_manager.transform_pose(pose,"World","Model")
                p_m = pose_m.pose.position
                object_in_region = (p_m.x > x_bounds[0] and p_m.x < x_bounds[1]
                                    and p_m.y > y_bounds[0] and p_m.y < y_bounds[1]
                                    and p_m.z > z_bounds[0] and p_m.z < z_bounds[1])
                # Exit loop if object isn't in region, else get new pose:
                if not object_in_region:
                    break
                else:
                    print "\t   Moving object out of excluded region."
                    theta = 2*np.pi * np.random.rand()
                    rho = ((outer_bound - inner_bound) * np.random.rand()) + inner_bound
                    pose.position.x, pose.position.y = rho*np.cos(theta), rho*np.sin(theta) 

            print "\tpose for %s" % name
            print pose
            self.model_manager.spawn_model(name,name, model_pose = pose)
            print "\t%d  spawned %s" % (i,name)

        print "\tDone spawning clutter!"

        return objects

    # Removes objects added by add_clutter
    # TODO objects prefix when not using bigbird
    def remove_clutter(self, objects, objects_prefix = "bigbird/"):
        for name in objects:
            print "trying to remove %s" % (objects_prefix + name)
            self.model_manager.remove_model(model_name=objects_prefix+name)
            print "\tremoved %s" % (objects_prefix + name)

    def configure_dataset(self):
        # SEE README for info on dset schema.
        # TODO maybe take a dict of misc info (e.g., cam matrix, model name, notes)
        #   and save as attributes?  See implementation in grasp_dataset.py...
        num_k = len(self.keypoints)

        # uint8 string for storing images as the binary data from jpeg files:
        dt =  h5py.special_dtype(vlen=np.dtype('uint8'))
        img_dset = self.dset.create_dataset(name = 'image', 
                                       shape = (self.num_images,),
                                       dtype=dt)

        self.dset.create_dataset(name="keypoints_in_image",
                            shape = (self.num_images, num_k, 2),
                            maxshape = (None, num_k, 2)) 

        self.dset.create_dataset(name="keypoints_in_world",
                            shape = (self.num_images, num_k, 3),
                            maxshape = (None, num_k, 3)) 

        self.dset.create_dataset(name="model_pose",
                            shape = (self.num_images,  7),
                            maxshape = (None,  7)) 

    #this method from:
    #https://github.com/rll/sushichallenge/blob/master/python/brett2/ros_utils.py
    # ...by way of J. Varley's GazeboCameraManager class.
    def image2numpy(self, image):
        if image.encoding == 'rgb8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)[:, :, ::-1]
        if image.encoding == 'bgr8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)
        elif image.encoding == 'mono8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width)
        elif image.encoding == '32FC1':
            return np.fromstring(image.data, dtype=np.float32).reshape(image.height, image.width)
        else:
            raise Exception

# TODO implement cam info parameter.
def get_node_params():
    dset_name = rospy.get_param("~dataset_name")
    model_name = rospy.get_param("~model_name")
    num_images = rospy.get_param("~num_images")
    cam_dist = rospy.get_param("~cam_distance")
    do_clutter = rospy.get_param("~do_clutter")
    exclude_region = rospy.get_param("~clutter_exclude_region")
    scale_factor = rospy.get_param("~model_scale_factor")

    return dset_name, model_name, num_images, cam_dist, do_clutter, exclude_region, scale_factor

if __name__ == "__main__":
    starttime = time()

    rospy.init_node('dataset_generator')

    (dset_name, model_name, 
     num_images, cam_dist, 
     do_clutter, exclude_region, scale_factor) = get_node_params()

    gen = ModelKeypointDatasetGenerator(dset_name, model_name, num_images, cam_dist,
                 do_clutter = do_clutter, clutter_exclude_region = exclude_region,
                 model_scale_factor = scale_factor)
    gen.generate_dataset()

    endtime = time()
    print "\n TOTAL RUNNING TIME: %d seconds" % (endtime-starttime)
