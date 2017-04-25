# Extracts jpeg images from model pose dataset.

import numpy as np
import h5py
import matplotlib.pyplot as plt
from rospkg.rospack import RosPack

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
    i = -1
    while i+1 < dset.shape[0] and len(dset[i+1]) >0:
        i += 1
        jpeg_binary = dset[i]
        filename = "img%03d.jpeg" % i
        write_binary_to_jpeg(jpeg_binary, filepath = dest_dir,
                             filename = filename)
        if i % 25 == 0:
            print "Saved img %d" % i
    print "Finished after saving %d imgs." % (i+1)
    print "Imgs saved to %s" % dest_dir


if __name__ == "__main__":
    pack_root = RosPack().get_path("model_pose_dataset_generation")
    DSET_PATH = pack_root + "/datasets/wall_texture_gastank_texture_CLUTTER.h5"
    DESTINATION_DIR = "/tmp/imgs/"
    
    dset = h5py.File(DSET_PATH, 'a')
    images = dset['image']
    write_all_images_to_files(images, DESTINATION_DIR)
    dset.close()
