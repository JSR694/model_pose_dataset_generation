Package for creating a dataset of synthetically-rended object images, annotated with 
   the pose of the object and locations of keypoints (both in image and in the world).

Objects are rendered in Gazebo.  The dataset is stored in the HDF5 format-- see "REFERENCE" for
   details on the datset's schema.

NOTE: this code is currently rough, but functional.
      Features to make it more flexible+useful will be added when possible.
      See "todo.txt" for a list of outstanding problems and planned features.


=== Additional features: ===
-Can add other objects randomly near the target object, as clutter.
-Keypoints loaded from meshlab-compatible .pp file.  
  Keypoints can be created easily using Meshlab's "pick points" tool.


=== SETUP AND USAGE: ===
Currently, important parameters (e.g. dataset's name, location, number of images, etc.) must be set
 manually in the script.  Will be replaced by launch file params etc. in the future.

Must add this directory to GAZEBO_RESOURCE_PATH and GAZEBO_MODEL_PATH.
  To do so, source gazebo_media_setup.sh (in the 'scripts' dir).

Make sure that the specified model is present in the 'models' dir.

Keypoints must be stored in a file "keypoints.pp" in the model's root dir.

Roslaunch the included launch file, and then run the 'model_views.py' script.

To extract images from the dataset, run the script "extract_images.py".

=== REFERENCES: ===

See "gastank_diagram.pdf" for a labeled diagram of keypoint locations on the object.

HDF5 dataset schema:
    NOTE that image is indexed [row, col], while keypoints given [x,y].

    =DSET NAME:=            =SHAPE=                         =NOTE=

    images                  [?, img_height, img_width, 3]    "?" = number of images.
    keypoints_in_image      [?, ??, 2]                       "??"= num keypoints.  [x, y].
    keypoints_in_world      [?, ??, 3]                       3D pose of keypts, in camera frame.
    model_pose              [?, 7]                           (Quaternion, Point) in camera frame.

